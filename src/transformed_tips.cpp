#include "transformed_tips.h"
#include "forward_kinematics.h"

Eigen::VectorXd transformed_tips(
  const Skeleton & skeleton, 
  const Eigen::VectorXi & b)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  Eigen::VectorXd  tip_pos = Eigen::VectorXd::Zero(3*b.size());
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> T; // transformation matrices
  forward_kinematics(skeleton, T); 
  for (int i = 0; i < b.size(); i++) {
    Bone bone = skeleton[b[i]];
    // equation from slide rest bone
    Eigen::Vector4d origin_tip = bone.rest_T * Eigen::Vector4d(skeleton[b[i]].length, 0, 0, 1);
    // tip after trans
    Eigen::VectorXd cur_tip = T[b[i]]* origin_tip;
    tip_pos[i * 3] = cur_tip.x();
    tip_pos[i * 3 + 1] = cur_tip.y();
    tip_pos[i * 3 + 2] = cur_tip.z();


  }
  return tip_pos;
  /////////////////////////////////////////////////////////////////////////////
}
