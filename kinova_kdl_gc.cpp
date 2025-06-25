
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <unistd.h>
#include <time.h>

#include "kortex_api/common/KDetailedException.h"

#include "kortex_api/client_stubs/BaseClientRpc.h"
#include "kortex_api/client_stubs/BaseCyclicClientRpc.h"
#include "kortex_api/client_stubs/ActuatorConfigClientRpc.h"
#include "kortex_api/client_stubs/SessionClientRpc.h"

#include "kortex_api/client/SessionManager.h"
#include "kortex_api/client/RouterClient.h"
#include "kortex_api/client/TransportClientUdp.h"
#include "kortex_api/client/TransportClientTcp.h"

#include "kdl_parser.hpp"

#include "kdl/chain.hpp"
#include "kdl/kinfam_io.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"

#define DEG_TO_RAD(x) (x) * M_PI / 180.0
#define RAD_TO_DEG(x) (x) * 180.0 / M_PI

namespace k_api = Kinova::Api;

#define PORT 10000
#define PORT_REAL_TIME 10001
#define IP_ADDRESS "192.168.1.12"
#define NUM_JOINTS 7
 
int main(int argc, char ** argv)
{
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

  // const std::string urdf_filename = std::string("../gen3.urdf");
  
  // KDL::Tree tree;
  // if (!kdl_parser::treeFromFile(urdf_filename, tree)) {
  //   std::cerr << "Failed to construct KDL tree from URDF file: " << urdf_filename << std::endl;
  //   return -1;
  // }
  // KDL::Chain chain;
  // if (!tree.getChain("gen3_base_link", "gen3_end_effector_link", chain)) {
  //   std::cerr << "Failed to get KDL chain from tree." << std::endl;
  //   return -1;
  // }

  const std::string urdf_filename = std::string("../eddie.urdf");

  KDL::Tree tree;
  if (!kdl_parser::treeFromFile(urdf_filename, tree)) {
    std::cerr << "Failed to construct KDL tree from URDF file: " << urdf_filename << std::endl;
    return -1;
  }
  KDL::Chain chain;
  if (!tree.getChain("kinova_right_base_link", "kinova_right_end_effector_link", chain)) {
    std::cerr << "Failed to get KDL chain from tree." << std::endl;
    return -1;
  }

  int num_joints = chain.getNrOfJoints();
  std::cout << "Number of joints in the chain: " << num_joints << std::endl;

  int num_segments = chain.getNrOfSegments();
  std::cout << "Number of segments in the chain: " << num_segments << std::endl;

  // KDL::ChainIdSolver_RNE id_solver(chain, KDL::Vector(0, 0, -9.81));
  KDL::ChainIdSolver_RNE id_solver(chain, KDL::Vector(-9.1, -1.01, 1.1));

  KDL::JntArray q(num_joints);
  KDL::JntArray qd(num_joints);
  KDL::JntArray qdd(num_joints);
  KDL::JntArray tau(num_joints);

  KDL::Wrenches f_ext(num_segments);
  for (int i = 0; i < num_segments; ++i) {
    f_ext[i] = KDL::Wrench::Zero();
  }

  id_solver.CartToJnt(q, qd, qdd, f_ext, tau);

  std::cout << "Joint torques: " << tau << std::endl;

  // clearing faults
  base->ClearFaults();
  try {
      base->ClearFaults();
  } catch(...) {
      std::cout << "Unable to clear robot faults" << std::endl;
      return false;
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

      return false;
  } catch (std::runtime_error& ex2) {
      std::cout << "Error: " << ex2.what() << std::endl;

      // Set the servoing mode back to Single Level
      servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
      base->SetServoingMode(servoing_mode);

      std::this_thread::sleep_for(std::chrono::milliseconds(2000));

      return false;
  }

  float control_freq = 1000.0; // Control frequency in Hz

  // ---------------------- control loop ----------------------
  while(true) {
    auto start_time = std::chrono::high_resolution_clock::now();

    // update joint positions and velocities
    for (unsigned int i = 0; i < NUM_JOINTS; i++)
    {
      q(i) = DEG_TO_RAD(base_feedback.actuators(i).position());
      qd(i) = DEG_TO_RAD(base_feedback.actuators(i).velocity());
    }

    // compute joint torques for gravity compensation
    id_solver.CartToJnt(q, qd, qdd, f_ext, tau);

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
