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

#define DEG_TO_RAD(x) (x * M_PI / 180.0)
#define RAD_TO_DEG(x) (x * 180.0 / M_PI)

namespace k_api = Kinova::Api;

#define PORT 10000
#define PORT_REAL_TIME 10001
#define IP_ADDRESS "192.168.1.12"
#define NUM_JOINTS 7

int main()
{
    auto error_callback = [](k_api::KError err) {
        std::cout << "API error: " << err.toString() << std::endl;
    };

    // Create TCP transport and router
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(IP_ADDRESS, PORT);

    // Create UDP transport and router for real-time feedback
    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(IP_ADDRESS, PORT_REAL_TIME);

    // Create and authenticate sessions
    auto session_info = k_api::Session::CreateSessionInfo();
    session_info.set_username("admin");
    session_info.set_password("admin");
    session_info.set_session_inactivity_timeout(60000);
    session_info.set_connection_inactivity_timeout(2000);

    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(session_info);

    auto session_manager_rt = new k_api::SessionManager(router_real_time);
    session_manager_rt->CreateSession(session_info);

    // Create base cyclic client for real-time joint feedback
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);

    std::cout << "Reading joint positions..." << std::endl;

    float control_freq = 1000.0; // Control frequency in Hz

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

    int num_joints = chain.getNrOfJoints();
    std::cout << "Number of joints in the chain: " << num_joints << std::endl;

    int num_segments = chain.getNrOfSegments();
    std::cout << "Number of segments in the chain: " << num_segments << std::endl;

    KDL::ChainIdSolver_RNE id_solver(chain, KDL::Vector(-9.5, -1.1, 1.23));

    KDL::JntArray q(num_joints);
    KDL::JntArray qd(num_joints);
    KDL::JntArray qdd(num_joints);
    KDL::JntArray tau(num_joints);

    KDL::Wrenches f_ext(num_segments);
    for (int i = 0; i < num_segments; ++i) {
        f_ext[i] = KDL::Wrench::Zero();
    }

    while (true)
    {
        auto start_time = std::chrono::high_resolution_clock::now();
        try
        {
            auto feedback = base_cyclic->RefreshFeedback();
            // update joint positions and velocities
            for (unsigned int i = 0; i < NUM_JOINTS; i++)
            {
                q(i) = DEG_TO_RAD(feedback.actuators(i).position());
                qd(i) = DEG_TO_RAD(feedback.actuators(i).velocity());
            }

            // compute joint torques for gravity compensation
            id_solver.CartToJnt(q, qd, qdd, f_ext, tau);

            std::cout << "taus: " << tau << std::endl;
        }
        catch (const k_api::KDetailedException& ex)
        {
            std::cerr << "Kortex exception: " << ex.what() << std::endl;
        }
        catch (const std::runtime_error& ex)
        {
            std::cerr << "Runtime error: " << ex.what() << std::endl;
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

    // Cleanup
    session_manager->CloseSession();
    session_manager_rt->CloseSession();

    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    delete base_cyclic;
    delete session_manager;
    delete session_manager_rt;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;

    return 0;
}
