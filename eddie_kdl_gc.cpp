
#include <string>
#include <iostream>

#include "kdl_parser.hpp"

#include "kdl/chain.hpp"
#include "kdl/kinfam_io.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"
 
int main(int argc, char ** argv)
{ 
  const std::string urdf_filename = std::string("eddie.urdf");
 
  KDL::Tree tree;
  if (!kdl_parser::treeFromFile(urdf_filename, tree)) {
    std::cerr << "Failed to construct KDL tree from URDF file: " << urdf_filename << std::endl;
    return -1;
  }
  KDL::Chain chain;
  if (!tree.getChain("kinova_left_base_link", "kinova_left_end_effector_link", chain)) {
    std::cerr << "Failed to get KDL chain from tree." << std::endl;
    return -1;
  }

  int num_joints = chain.getNrOfJoints();
  std::cout << "Number of joints in the chain: " << num_joints << std::endl;

  int num_segments = chain.getNrOfSegments();
  std::cout << "Number of segments in the chain: " << num_segments << std::endl;

  KDL::ChainIdSolver_RNE id_solver(chain, KDL::Vector(0, 0, -9.81));

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

  return 0;
}
