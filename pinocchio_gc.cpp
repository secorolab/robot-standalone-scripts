#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/model.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include <iostream>

using namespace pinocchio;

int main() {
  const std::string urdf_filename = "gen3.urdf";

  Model full_model;
  pinocchio::urdf::buildModel(urdf_filename, full_model);
  std::cout << "Full model name: " << full_model.name << std::endl;

  // List of gripper joints to lock
  std::vector<std::string> joints_to_lock_names = {
    "gen3_robotiq_85_left_inner_knuckle_joint",
    "gen3_robotiq_85_left_knuckle_joint",
    "gen3_robotiq_85_left_finger_tip_joint",
    "gen3_robotiq_85_right_inner_knuckle_joint",
    "gen3_robotiq_85_right_knuckle_joint",
    "gen3_robotiq_85_right_finger_tip_joint"
  };

  // Convert joint names to JointIndex
  std::vector<Model::JointIndex> joints_to_lock;
  for (const auto& joint_name : joints_to_lock_names) {
    if (full_model.existJointName(joint_name)) {
      joints_to_lock.push_back(full_model.getJointId(joint_name));
    } else {
      std::cerr << "Warning: Joint not found - " << joint_name << std::endl;
    }
  }

  // Build the reduced model by locking gripper joints
  Eigen::VectorXd reference_configuration = Eigen::VectorXd::Zero(full_model.nq);  // You can customize this
  Model reduced_model = buildReducedModel(full_model, joints_to_lock, reference_configuration);

  reduced_model.gravity = Motion(Eigen::Vector3d(0, 0, -9.81), Eigen::Vector3d::Zero());

  std::cout << "Reduced model name: " << reduced_model.name << std::endl;
  std::cout << "Joints in reduced model:" << std::endl;
  
  for (std::size_t i = 0; i < reduced_model.joints.size(); ++i) {
    std::cout << i << ": " << reduced_model.names[i] << std::endl;
  }

  // recursive newton-euler solver
  Data data(reduced_model);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(reduced_model.nq);
  Eigen::VectorXd qd = Eigen::VectorXd::Zero(reduced_model.nv);
  Eigen::VectorXd qdd = Eigen::VectorXd::Zero(reduced_model.nv);
  // f_ext
  pinocchio::container::aligned_vector<pinocchio::Force> f_ext(reduced_model.joints.size(), Force::Zero());

  q[0] = 1.5;
  q[1] = 0.5;
  q[2] = -0.5;

  const Eigen::VectorXd tau = pinocchio::rnea(reduced_model, data, q, qd, qdd, f_ext);
  std::cout << "Joint torques: " << tau.transpose() << std::endl;

  return 0;
}

