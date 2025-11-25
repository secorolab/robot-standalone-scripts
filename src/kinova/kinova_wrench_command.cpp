#include <cstddef>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <chrono>
#include <csignal>

#include "Base.pb.h"
#include "kortex_api/common/KDetailedException.h"

#include "kortex_api/client_stubs/BaseClientRpc.h"
#include "kortex_api/client_stubs/BaseCyclicClientRpc.h"
#include "kortex_api/client_stubs/ActuatorConfigClientRpc.h"

#include "kortex_api/client/SessionManager.h"
#include "kortex_api/client/RouterClient.h"
#include "kortex_api/client/TransportClientUdp.h"
#include "kortex_api/client/TransportClientTcp.h"


volatile sig_atomic_t kill_flag = 0;

#define DEG_TO_RAD(x) (x) * M_PI / 180.0
#define RAD_TO_DEG(x) (x) * 180.0 / M_PI

namespace k_api = Kinova::Api;

#define PORT 10000
#define PORT_REAL_TIME 10001
#define IP_ADDRESS "192.168.1.10"
#define NUM_JOINTS 7


void handle_kill_signal(int sig) {
  static int signal_caught = 0;
  if (!signal_caught) {
    signal_caught = 1;
    kill_flag = 1;
    printf("Caught kill signal %d (%s)\n", sig, strsignal(sig));
  }
}

int main(int argc, char ** argv)
{
  signal(SIGINT, handle_kill_signal);

  // --------------------- kinova config ----------------------
  auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };

  std::cout << "Creating transport objects" << std::endl;
  auto transport = new k_api::TransportClientTcp();
  auto router = new k_api::RouterClient(transport, error_callback);
  transport->connect(IP_ADDRESS, PORT);

  std::cout << "Creating transport real time objects" << std::endl;
  auto transport_real_time = new k_api::TransportClientUdp();
  auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
  transport_real_time->connect(IP_ADDRESS, PORT_REAL_TIME);

  // Set session data connection information
  auto create_session_info = k_api::Session::CreateSessionInfo();
  create_session_info.set_username("admin");
  create_session_info.set_password("admin");
  create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
  create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

  // Session manager service wrapper
  std::cout << "Creating sessions for communication" << std::endl;
  auto session_manager = new k_api::SessionManager(router);
  session_manager->CreateSession(create_session_info);
  auto session_manager_real_time = new k_api::SessionManager(router_real_time);
  session_manager_real_time->CreateSession(create_session_info);
  std::cout << "Sessions created" << std::endl;

  // Create services
  auto base = new k_api::Base::BaseClient(router);
  auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
  auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);

  // clearing faults
  try {
      base->ClearFaults();
  } catch(...) {
      std::cout << "Unable to clear robot faults" << std::endl;
      return 1;
  }

  // --------------------- wrench command -----------------------

  k_api::BaseCyclic::Feedback base_feedback;
  
  auto servoing_mode = k_api::Base::ServoingModeInformation();
  servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  base->SetServoingMode(servoing_mode);

  auto ref_frame = k_api::Common::CARTESIAN_REFERENCE_FRAME_MIXED;
  auto wrench_command = k_api::Base::WrenchCommand();
  wrench_command.set_reference_frame(ref_frame);
  wrench_command.set_mode(k_api::Base::WrenchMode::WRENCH_NORMAL);
  
  auto wrench = wrench_command.mutable_wrench();
  wrench->set_force_x(0.0);
  wrench->set_force_y(0.0);
  wrench->set_force_z(0.0);
  wrench->set_torque_x(0.0);
  wrench->set_torque_y(0.0);
  wrench->set_torque_z(10.0); // Apply constant torque around Z axis - rotates bracelet link

  base->SendWrenchCommand(wrench_command);

  for (;!kill_flag;) {
    base_feedback = base_cyclic->RefreshFeedback();
    printf("measured tau z: %.2f\n", base_feedback.base().tool_external_wrench_torque_z());
  }

  wrench->set_torque_z(0.0); // Stop applying torque
  base->SendWrenchCommand(wrench_command);

  // Wait for a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  std::cout << "Clean exit" << std::endl;

  // close and clean up

  // Close API session
  session_manager->CloseSession();
  session_manager_real_time->CloseSession();

  // Deactivate the router and cleanly disconnect from the transport object
  router->SetActivationStatus(false);
  transport->disconnect();
  router_real_time->SetActivationStatus(false);
  transport_real_time->disconnect();

  // Destroy the API
  delete base;
  delete base_cyclic;
  delete actuator_config;
  delete session_manager;
  delete session_manager_real_time;
  delete router;
  delete router_real_time;
  delete transport;
  delete transport_real_time;

  return 0;
}
