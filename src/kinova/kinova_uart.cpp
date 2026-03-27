/*
 * kinova_robotiq_uart_bridge.cpp
 *
 * Drives a Robotiq 2F gripper connected to the Gen3 Interconnect UART
 * expansion port (via RS485-UART converter) using the existing
 * robotiq_driver_noros DefaultDriver unchanged.
 *
 * The only new piece is KortexSerial: a Serial implementation that
 * talks through the Kortex UART bridge TCP socket instead of a local
 * /dev/ttyUSB* device.  DefaultDriver never knows the difference.
 *
 * Hardware:
 *   Gripper RS485 <-> RS485-UART converter <-> Gen3 Interconnect UART expansion
 *   UART: 115200 8N1, Robotiq Modbus RTU slave address 0x09
 *
 * Build: link against kortex_api and robotiq_driver_noros.
 *        Same CMakeLists structure as 103-Gen3_uart_bridge example.
 */

#include <BaseClientRpc.h>
#include <DeviceManagerClientRpc.h>
#include <InterconnectConfigClientRpc.h>
#include <RouterClient.h>
#include <SessionManager.h>
#include <TransportClientTcp.h>

#include <robotiq_driver_noros/default_driver.hpp>
#include <robotiq_driver_noros/serial.hpp>

#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <Ws2tcpip.h>
#include <winsock2.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/time.h>
#define SOCKET int
#endif

namespace k_api = Kinova::Api;
using robotiq_driver::DefaultDriver;
using robotiq_driver::Serial;

#define IP_ADDRESS "192.168.1.10"
#define PORT 10000

constexpr auto kSlaveAddress = 0x09;

// ============================================================
// Socket helpers (from Kinova UART bridge example)
// ============================================================

static int init_wsa() {
#ifdef _WIN32
  WSADATA d;
  return WSAStartup(MAKEWORD(1, 1), &d);
#else
  return 0;
#endif
}

static int quit_wsa() {
#ifdef _WIN32
  return WSACleanup();
#else
  return 0;
#endif
}

static void close_sock(SOCKET s) {
#ifdef _WIN32
  shutdown(s, SD_BOTH);
  closesocket(s);
#else
  shutdown(s, SHUT_RDWR);
  close(s);
#endif
}

static SOCKET make_tcp_socket(const char *ip, int port) {
  SOCKET sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0)
    return -1;

  sockaddr_in srv{};
  srv.sin_family = AF_INET;
  srv.sin_addr.s_addr = inet_addr(ip);
  srv.sin_port = htons(static_cast<uint16_t>(port));

  if (connect(sock, reinterpret_cast<sockaddr *>(&srv), sizeof(srv)) < 0) {
    std::cout << "TCP connect to bridge port " << port << " failed."
              << std::endl;
    return -1;
  }
  return sock;
}

// ============================================================
// KortexSerial
//
// Implements robotiq_driver::Serial over the Kortex UART bridge
// TCP socket.  The socket is injected after the bridge is opened;
// open()/close() manage only the internal open flag, not the
// Kortex bridge lifecycle (that is handled by KortexBridge below).
// ============================================================

class KortexSerial : public Serial {
public:
  explicit KortexSerial(int timeout_ms = 1000)
      : m_sock(-1), m_open(false), m_timeout_ms(timeout_ms) {}

  // Called by KortexBridge once the TCP socket is ready
  void set_socket(SOCKET sock) {
    m_sock = sock;
    m_open = (sock >= 0);
  }

  void open() override {
    if (m_sock < 0)
      throw std::runtime_error(
          "KortexSerial::open() called before socket was set.");
    m_open = true;
  }

  bool is_open() const override { return m_open; }

  void close() override { m_open = false; }

  // Blocking read of exactly `size` bytes with timeout
  std::vector<uint8_t> read(size_t size) override {
    std::vector<uint8_t> buf(size);
    size_t received = 0;
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(m_timeout_ms);

    while (received < size) {
      auto now = std::chrono::steady_clock::now();
      auto us =
          std::chrono::duration_cast<std::chrono::microseconds>(deadline - now)
              .count();
      if (us <= 0)
        throw std::runtime_error("KortexSerial::read() timeout.");

      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(m_sock, &fds);
      timeval tv{static_cast<long>(us / 1'000'000),
                 static_cast<long>(us % 1'000'000)};

      int ready =
          select(static_cast<int>(m_sock) + 1, &fds, nullptr, nullptr, &tv);
      if (ready <= 0)
        continue;

      int got = static_cast<int>(
          recv(m_sock, reinterpret_cast<char *>(buf.data() + received),
               static_cast<int>(size - received), 0));
      if (got <= 0)
        throw std::runtime_error("KortexSerial::read() socket closed.");
      received += static_cast<size_t>(got);
    }
    return buf;
  }

  // Write all bytes to the socket
  void write(const std::vector<uint8_t> &data) override {
    size_t sent = 0;
    while (sent < data.size()) {
      int n = static_cast<int>(
          ::send(m_sock, reinterpret_cast<const char *>(data.data() + sent),
                 static_cast<int>(data.size() - sent), 0));
      if (n < 0)
        throw std::runtime_error("KortexSerial::write() failed.");
      sent += static_cast<size_t>(n);
    }
  }

  // Port/baudrate/timeout setters are no-ops: the Kortex bridge owns
  // UART configuration; we just talk over the TCP socket it gives us.
  void set_port(const std::string & /*port*/) override {}
  std::string get_port() const override { return "kortex_uart_bridge"; }

  void set_timeout(std::chrono::milliseconds timeout) override {
    m_timeout_ms = static_cast<int>(timeout.count());
  }
  std::chrono::milliseconds get_timeout() const override {
    return std::chrono::milliseconds(m_timeout_ms);
  }

  void set_baudrate(uint32_t /*baudrate*/) override {}
  uint32_t get_baudrate() const override { return 115200; }

private:
  SOCKET m_sock;
  bool m_open;
  int m_timeout_ms;
};

// ============================================================
// KortexBridge
//
// Owns the Kortex API session and the UART bridge.
// Constructs KortexSerial + DefaultDriver and exposes the driver.
// ============================================================

class KortexBridge {
public:
  KortexBridge(const std::string &ip = IP_ADDRESS, int port = PORT,
               const std::string &user = "admin",
               const std::string &pass = "admin")
      : m_ip(ip), m_port(port), m_user(user), m_pass(pass),
        m_interconnect_id(0), m_bridge_id(0), m_sock(-1), m_transport(nullptr),
        m_router(nullptr), m_session(nullptr), m_base(nullptr),
        m_dev_mgr(nullptr), m_ic_config(nullptr) {}

  ~KortexBridge() { shutdown(); }

  // Connect to robot, open UART bridge, build driver
  DefaultDriver &init() {
    init_wsa();

    m_transport = new k_api::TransportClientTcp();
    m_transport->connect(m_ip, m_port);

    m_router = new k_api::RouterClient(m_transport, [](k_api::KError e) {
      std::cout << "Kortex error: " << e.toString();
    });

    auto si = k_api::Session::CreateSessionInfo();
    si.set_username(m_user);
    si.set_password(m_pass);
    si.set_session_inactivity_timeout(60000);
    si.set_connection_inactivity_timeout(2000);

    m_session = new k_api::SessionManager(m_router);
    m_session->CreateSession(si);

    m_dev_mgr = new k_api::DeviceManager::DeviceManagerClient(m_router);
    m_ic_config =
        new k_api::InterconnectConfig::InterconnectConfigClient(m_router);
    m_base = new k_api::Base::BaseClient(m_router);

    m_interconnect_id = find_device(k_api::Common::INTERCONNECT, 0);
    if (m_interconnect_id == 0)
      throw std::runtime_error("No Interconnect module found.");

    std::cout << "Interconnect device ID: " << m_interconnect_id << std::endl;

    // Configure UART on Interconnect: 115200 8N1
    k_api::Common::UARTConfiguration uart_cfg;
    uart_cfg.set_port_id(k_api::InterconnectConfig::UART_PORT_EXPANSION);
    uart_cfg.set_enabled(true);
    uart_cfg.set_speed(k_api::Common::UART_SPEED_115200);
    uart_cfg.set_word_length(k_api::Common::UART_WORD_LENGTH_8);
    uart_cfg.set_stop_bits(k_api::Common::UART_STOP_BITS_1);
    uart_cfg.set_parity(k_api::Common::UART_PARITY_NONE);
    m_ic_config->SetUARTConfiguration(uart_cfg, m_interconnect_id);

    // Enable bridge on Base
    k_api::Base::BridgeConfig bc;
    bc.set_device_identifier(m_interconnect_id);
    bc.set_bridgetype(k_api::Base::BRIDGE_TYPE_UART);
    auto result = m_base->EnableBridge(bc);

    if (result.status() != k_api::Base::BRIDGE_STATUS_OK)
      throw std::runtime_error("EnableBridge failed.");

    m_bridge_id = result.bridge_id().bridge_id();

    k_api::Base::BridgeIdentifier bid;
    bid.set_bridge_id(m_bridge_id);
    bc = m_base->GetBridgeConfig(bid);
    uint16_t base_port = static_cast<uint16_t>(bc.port_config().out_port());

    std::cout << "UART bridge ready on TCP port " << base_port << " (bridge ID "
              << m_bridge_id << ")" << std::endl;

    m_sock = make_tcp_socket(IP_ADDRESS, base_port);
    if (m_sock < 0)
      throw std::runtime_error("Cannot connect to bridge TCP socket.");

    // Build KortexSerial and hand to DefaultDriver
    auto serial = std::make_unique<KortexSerial>(/*timeout_ms=*/1000);
    serial->set_socket(m_sock);

    m_driver = std::make_unique<DefaultDriver>(std::move(serial));
    m_driver->set_slave_address(kSlaveAddress);

    return *m_driver;
  }

  void shutdown() {
    if (m_sock >= 0) {
      close_sock(m_sock);
      m_sock = -1;
    }
    if (m_base && m_bridge_id != 0) {
      k_api::Base::BridgeIdentifier bid;
      bid.set_bridge_id(m_bridge_id);
      m_base->DisableBridge(bid);
      m_bridge_id = 0;
    }
    if (m_ic_config && m_interconnect_id != 0) {
      k_api::Common::UARTConfiguration uart_cfg;
      uart_cfg.set_port_id(k_api::InterconnectConfig::UART_PORT_EXPANSION);
      uart_cfg.set_enabled(false);
      m_ic_config->SetUARTConfiguration(uart_cfg, m_interconnect_id);
    }
    if (m_session) {
      m_session->CloseSession();
      m_router->SetActivationStatus(false);
      m_transport->disconnect();
    }
    delete m_ic_config;
    m_ic_config = nullptr;
    delete m_dev_mgr;
    m_dev_mgr = nullptr;
    delete m_session;
    m_session = nullptr;
    delete m_base;
    m_base = nullptr;
    delete m_router;
    m_router = nullptr;
    delete m_transport;
    m_transport = nullptr;
    quit_wsa();
  }

private:
  std::string m_ip, m_user, m_pass;
  int m_port;
  uint32_t m_interconnect_id;
  uint32_t m_bridge_id;
  SOCKET m_sock;

  k_api::TransportClientTcp *m_transport;
  k_api::RouterClient *m_router;
  k_api::SessionManager *m_session;
  k_api::Base::BaseClient *m_base;
  k_api::DeviceManager::DeviceManagerClient *m_dev_mgr;
  k_api::InterconnectConfig::InterconnectConfigClient *m_ic_config;

  std::unique_ptr<DefaultDriver> m_driver;

  uint32_t find_device(k_api::Common::DeviceTypes type, uint32_t index) {
    auto handles = m_dev_mgr->ReadAllDevices();
    uint32_t cur = 0;
    for (auto &h : handles.device_handle())
      if (h.device_type() == type && cur++ == index)
        return h.device_identifier();
    return 0;
  }
};

// ============================================================
// main — identical sequence to gripper_interface_test.cpp
// ============================================================

int main(int argc, char **argv) {
  std::string ip = IP_ADDRESS;
  if (argc > 1)
    ip = argv[1];

  std::cout << "Connecting to Gen3 at " << ip << " ..." << std::endl;

  try {
    KortexBridge bridge(ip, PORT, "admin", "admin");
    DefaultDriver &driver = bridge.init();

    if (!driver.connect()) {
      std::cout << "Driver could not connect to gripper." << std::endl;
      return 1;
    }
    std::cout << "Gripper connected." << std::endl;

    std::cout << "Deactivating..." << std::endl;
    driver.deactivate();

    std::cout << "Activating..." << std::endl;
    driver.activate();
    std::cout << "Gripper activated." << std::endl;

    std::cout << "Closing (0xFF)..." << std::endl;
    driver.set_gripper_position(0xFF);
    while (driver.gripper_is_moving())
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "Opening (0x00)..." << std::endl;
    driver.set_gripper_position(0x00);
    while (driver.gripper_is_moving())
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "Half closing (0x80)..." << std::endl;
    driver.set_gripper_position(0x80);
    while (driver.gripper_is_moving())
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "Opening..." << std::endl;
    driver.set_gripper_position(0x00);
    while (driver.gripper_is_moving())
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "Reducing speed (0x0F) and closing slowly..." << std::endl;
    driver.set_speed(0x0F);
    driver.set_gripper_position(0xFF);
    while (driver.gripper_is_moving())
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "Full speed (0xFF) and opening fast..." << std::endl;
    driver.set_speed(0xFF);
    driver.set_gripper_position(0x00);
    while (driver.gripper_is_moving())
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "Test complete." << std::endl;
    bridge.shutdown();
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
