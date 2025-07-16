#include <sstream>
#include <string>
#include <iostream>

#include "kdl_parser/kdl_parser.hpp"

#include "kdl/chain.hpp"
#include "kdl/kinfam_io.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"


int main(int argc, char ** argv)
{
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
    
    std::cout << "Segment [" << i << "] " << name << ":\n" << f_tip << "\n\n";

    KDL::Frame f_new = f_tip * transforms[i];

    std::cout << "nSegment [" << i << "] " << name << ":\n" << f_new << "\n\n";

    KDL::Segment updated_segment(
      name,                          // Same name
      segment.getJoint(),            // Same joint
      f_new,                     // Updated fixed transform
      segment.getInertia()           // Same inertia
    );

    nChain.addSegment(updated_segment);
  }

  KDL::ChainIdSolver_RNE id_solver(chain, KDL::Vector(0, 0, -9.81));

  KDL::ChainIdSolver_RNE nid_solver(nChain, KDL::Vector(0, 0, -9.81));

  KDL::JntArray q(num_joints);
  KDL::JntArray qd(num_joints);
  KDL::JntArray qdd(num_joints);
  KDL::JntArray tau(num_joints);

  q(1) = 0.5;
  q(3) = 0.5;

  KDL::JntArray ntau(num_joints);

  KDL::Wrenches f_ext(num_segments);
  for (int i = 0; i < num_segments; ++i) {
    f_ext[i] = KDL::Wrench::Zero();
  }

  id_solver.CartToJnt(q, qd, qdd, f_ext, tau);

  nid_solver.CartToJnt(q, qd, qdd, f_ext, ntau);

  std::cout << "Tau : " << tau << std::endl;
  std::cout << "nTau: " << ntau << std::endl;

  return 0;
}
