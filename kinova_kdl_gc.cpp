#include <cstddef>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <chrono>
#include <csignal>

#include "kortex_api/common/KDetailedException.h"

#include "kortex_api/client_stubs/BaseClientRpc.h"
#include "kortex_api/client_stubs/BaseCyclicClientRpc.h"
#include "kortex_api/client_stubs/ActuatorConfigClientRpc.h"
#include "kortex_api/client_stubs/SessionClientRpc.h"

#include "kortex_api/client/SessionManager.h"
#include "kortex_api/client/RouterClient.h"
#include "kortex_api/client/TransportClientUdp.h"
#include "kortex_api/client/TransportClientTcp.h"

#include "kdl_parser/kdl_parser.hpp"

#include "kdl/chain.hpp"
#include "kdl/kinfam_io.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

volatile sig_atomic_t kill_flag = 0;

#define DEG_TO_RAD(x) (x) * M_PI / 180.0
#define RAD_TO_DEG(x) (x) * 180.0 / M_PI

namespace k_api = Kinova::Api;

#define PORT 10000
#define PORT_REAL_TIME 10001
#define IP_ADDRESS "192.168.1.12"
#define NUM_JOINTS 7

void handle_kill_signal(int sig) {
  static int signal_caught = 0;
  if (!signal_caught) {
    signal_caught = 1;
    kill_flag = 1;
    printf("Caught kill signal %d (%s)\n", sig, strsignal(sig));
  }
}

// Convert KDL::JntArray to CSV string
std::string jntArrayToCSV(const KDL::JntArray& arr) {
    std::ostringstream oss;
    for (unsigned int i = 0; i < arr.rows(); ++i) {
        oss << arr(i);
        if (i != arr.rows() - 1) oss << ",";
    }
    return oss.str();
}

// Convert KDL::Wrench to CSV string (6 elements: force + torque)
std::string wrenchToCSV(const KDL::Wrench& wrench) {
    std::ostringstream oss;
    oss << wrench.force.x() << "," << wrench.force.y() << "," << wrench.force.z()
        << "," << wrench.torque.x() << "," << wrench.torque.y() << "," << wrench.torque.z();
    return oss.str();
}

int main(int argc, char ** argv)
{
  struct sigaction sa;
  sa.sa_handler = handle_kill_signal;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;

  for (int i = 1; i < NSIG; ++i) {
    if (sigaction(i, &sa, NULL) == -1) {
      perror("sigaction error");
    }
  }

  // std::shared_ptr<spdlog::logger> logger;
  //
  // try {
  //   logger = spdlog::basic_logger_mt("kinova_logger", "logs/kinova-kdl-gc-log.csv");
  // } catch (const spdlog::spdlog_ex &ex) {
  //   std::cout << "Log init failed: " << ex.what() << std::endl;
  // }
  // logger->set_pattern("%v");

  // --------------------- kinova ----------------------
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

  // --------------------- kinova ----------------------

  const std::string urdf_filename = std::string("../gen3.urdf");
  
  KDL::Tree tree;
  if (!kdl_parser::treeFromFile(urdf_filename, tree)) {
    std::cerr << "Failed to construct KDL tree from URDF file: " << urdf_filename << std::endl;
    return -1;
  }
  KDL::Chain chain;
  if (!tree.getChain("gen3_base_link", "gen3_end_effector_link", chain)) {
    std::cerr << "Failed to get KDL chain from tree." << std::endl;
    return -1;
  }

  KDL::Frame transforms[8] = {
    KDL::Frame(
      KDL::Rotation(
          0.9999934, -0.0034287, 0.0011825,
          0.0034252,  0.9999899, 0.0028966,
        -0.0011924, -0.0028925, 0.9999951
      ),
      KDL::Vector(-0.0009869, -0.0006710, -0.0007926)
    ),
    KDL::Frame(
      KDL::Rotation(
          0.9999951, 0.0016397, -0.0026737,
        -0.0016348, 0.9999970, 0.0018380,
          0.0026767, -0.0018336, 0.9999947
      ),
      KDL::Vector(0.0001692, -0.0007934, 0.0002771)
    ),
    KDL::Frame(
      KDL::Rotation(
          0.9999958, -0.0028914, 0.0002131,
          0.0028914,  0.9999958, 0.0001711,
        -0.0002136, -0.0001705, 1.0000000
      ),
      KDL::Vector(-0.0000965, -0.0002349, -0.0009027)
    ),
    KDL::Frame(
      KDL::Rotation(
          0.9999885, -0.0043961, 0.0019156,
          0.0043967,  0.9999903, -0.0003092,
        -0.0019142, 0.0003176, 0.9999981
      ),
      KDL::Vector(0.0001928, -0.0009025, 0.0003440)
    ),
    KDL::Frame(
      KDL::Rotation(
          0.9999879, 0.0035842, -0.0033711,
        -0.0035816, 0.9999933, 0.0007867,
          0.0033739, -0.0007746, 0.9999940
      ),
      KDL::Vector(0.0001264, -0.0002953, -0.0008272)
    ),
    KDL::Frame(
      KDL::Rotation(
          0.9999933, -0.0004036, -0.0036404,
          0.0003839,  0.9999852, -0.0054272,
          0.0036425,  0.0054257, 0.9999786
      ),
      KDL::Vector(-0.0000924, -0.0008274, 0.0000606)
    ),
    KDL::Frame(
      KDL::Rotation(
          0.9999814, 0.0060969, 0.0003375,
        -0.0060981, 0.9999743, 0.0037794,
        -0.0003144, -0.0037814, 0.9999928
      ),
      KDL::Vector(0.0000838, -0.0001367, -0.0000495)
    ),
    KDL::Frame(
      KDL::Rotation(
          0.9999782, -0.0060921, 0.0025659,
          0.0060982,  0.9999786, -0.0023815,
        -0.0025513, 0.0023971, 0.9999939
      ),
      KDL::Vector(-0.0003371, 0.0007925, 0.0000552)
    )
  };

  printf("\n\n");

  int num_joints = chain.getNrOfJoints();
  std::cout << "Number of joints in the chain: " << num_joints << std::endl;

  int num_segments = chain.getNrOfSegments();
  std::cout << "Number of segments in the chain: " << num_segments << std::endl;

  KDL::Chain nChain;

  for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
    const KDL::Segment& segment = chain.getSegment(i);
    const std::string& name = segment.getName();
    const KDL::Frame& f_tip = segment.getFrameToTip();
    
    // std::cout << "Segment [" << i << "] " << name << ":\n" << f_tip << "\n\n";

    KDL::Frame f_new = f_tip * transforms[i];

    // std::cout << "nSegment [" << i << "] " << name << ":\n" << f_new << "\n\n";

    KDL::Segment updated_segment(
      name,                          // Same name
      segment.getJoint(),            // Same joint
      f_new,                     // Updated fixed transform
      segment.getInertia()           // Same inertia
    );

    nChain.addSegment(updated_segment);
  }

  // KDL::ChainIdSolver_RNE id_solver(chain, KDL::Vector(0, 0, -9.81));
  // KDL::ChainIdSolver_RNE nid_solver(nChain, KDL::Vector(0, 0, -9.81));

  KDL::ChainIdSolver_RNE id_solver(chain, KDL::Vector(-9.5, -0.99, 1.11));
  KDL::ChainIdSolver_RNE nid_solver(nChain, KDL::Vector(-9.5, -0.99, 1.11));


  KDL::JntArray q(num_joints);
  KDL::JntArray qd(num_joints);
  KDL::JntArray qdd(num_joints);
  KDL::JntArray tau(num_joints);

  KDL::Wrenches f_ext(num_segments);
  for (int i = 0; i < num_segments; ++i) {
    f_ext[i] = KDL::Wrench::Zero();
  }

  id_solver.CartToJnt(q, qd, qdd, f_ext, tau);
  std::cout << "Tau : " << tau << std::endl;

  KDL::JntArray ntau(num_joints);
  nid_solver.CartToJnt(q, qd, qdd, f_ext, ntau);
  std::cout << "nTau: " << ntau << std::endl;

  // clearing faults
  try {
      base->ClearFaults();
  } catch(...) {
      std::cout << "Unable to clear robot faults" << std::endl;
      return 1;
  }

  // ---------------------- torque mode ----------------------
  k_api::BaseCyclic::Feedback base_feedback;
  k_api::BaseCyclic::Command  base_command;

  auto servoing_mode = k_api::Base::ServoingModeInformation();

  try {
    // Set the base in low-level servoing mode
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);
    base_feedback = base_cyclic->RefreshFeedback();

    // Initialize each actuator to their current position
    for (unsigned int i = 0; i < NUM_JOINTS; i++)
    {
      base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
    }

    // Send a first frame
    base_feedback = base_cyclic->Refresh(base_command);

    // Set actuators in torque mode now that the command is equal to measure
    auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
    control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);

    std::cout << "CONTROL MESSAGE: " << control_mode_message.control_mode() << std::endl;

    for (int id = 0; id < NUM_JOINTS; id++)
    {
      actuator_config->SetControlMode(control_mode_message, id+1);
    }
  } catch (k_api::KDetailedException& ex) {
      std::cout << "API error: " << ex.what() << std::endl;

      // Set the servoing mode back to Single Level
      servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
      base->SetServoingMode(servoing_mode);

      std::this_thread::sleep_for(std::chrono::milliseconds(2000));

      return 1;
  } catch (std::runtime_error& ex2) {
      std::cout << "Error: " << ex2.what() << std::endl;

      // Set the servoing mode back to Single Level
      servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
      base->SetServoingMode(servoing_mode);

      std::this_thread::sleep_for(std::chrono::milliseconds(2000));

      return 1;
  }

  float control_freq = 1000.0; // Control frequency in Hz

  // logger->info("q0,q1,q2,q3,q4,q5,qd0,qd1,qd2,qd3,qd4,qd5,"
                 // "qdd0,qdd1,qdd2,qdd3,qdd4,qdd5,"
                 // "tau0,tau1,tau2,tau3,tau4,tau5");

  // ---------------------- control loop ----------------------
  std::cout << "starting gravity comp" << std::endl;
  while(true) {
    if (kill_flag) {
      break;
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    // update joint positions and velocities
    for (unsigned int i = 0; i < NUM_JOINTS; i++)
    {
      q(i) = DEG_TO_RAD(base_feedback.actuators(i).position());
      qd(i) = DEG_TO_RAD(base_feedback.actuators(i).velocity());
    }

    // compute joint torques for gravity compensation
    // id_solver.CartToJnt(q, qd, qdd, f_ext, tau);
    nid_solver.CartToJnt(q, qd, qdd, f_ext, tau);

    // logger->info("{},{},{},{}", 
    //     jntArrayToCSV(q),
    //     jntArrayToCSV(qd),
    //     jntArrayToCSV(qdd),
    //     jntArrayToCSV(tau));

    // update the base command with the computed torques
    for (unsigned int i = 0; i < NUM_JOINTS; i++)
    {
      base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
      base_command.mutable_actuators(i)->set_torque_joint(tau(i));
    }

    // Incrementing identifier ensures actuators can reject out of time frames
    base_command.set_frame_id(base_command.frame_id() + 1);
    if (base_command.frame_id() > 65535)
      base_command.set_frame_id(0);

    for (int idx = 0; idx < NUM_JOINTS; idx++)
    {
      base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
    }

    try
    {
      base_feedback = base_cyclic->Refresh(base_command, 0);
    }
    catch (k_api::KDetailedException& ex)
    {
      std::cout << "Kortex exception: " << ex.what() << std::endl;

      std::cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
    }
    catch (std::runtime_error& ex2)
    {
      std::cout << "runtime error: " << ex2.what() << std::endl;
    }
    catch(...)
    {
      std::cout << "Unknown error." << std::endl;
    }

    // wait to maintain control frequency
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    while (elapsed.count() < 1.0 / control_freq)
    {
        end_time = std::chrono::high_resolution_clock::now();
        elapsed = end_time - start_time;
    }
  }

  // Set actuatorS back in position mode before exiting
  auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
  control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);

  std::cout << "Setting actuators back in position mode" << std::endl;

  for (int id = 0; id < NUM_JOINTS; id++)
  {
    actuator_config->SetControlMode(control_mode_message, id+1);
  }

  // Set the servoing mode back to Single Level
  servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  base->SetServoingMode(servoing_mode);

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
