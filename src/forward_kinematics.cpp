#include "forward_kinematics.h"
#include "euler_angles_to_transform.h"
#include <functional> // std::function

void forward_kinematics(
  const Skeleton & skeleton,
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  T.resize(skeleton.size(),Eigen::Affine3d::Identity());
  Eigen::Affine3d recur(int i, const Skeleton & skeleton);

//   Eigen::Affine3d recur = [&](int i, const Skeleton & skeleton){
//   if (skeleton[i].parent_index == -1){ //root
//     return Eigen::Affine3d::Identity(); //not moving
//   }

//   //Skeletons: Forward Kinematics from the slide
//   Eigen::Affine3d Tp = recur(skeleton[i].parent_index,skeleton); // Parent joint's Transformation
//   Eigen::Affine3d Te = euler_angles_to_transform(skeleton[i].xzx); // Euler Angle Rotations
//   Eigen::Affine3d Ti = skeleton[i].rest_T; // rest bone trans
//   return Tp * Ti * Te * Ti.inverse();
// };

  for (int i = 0; i < skeleton.size(); i++) {//for each bone in skeleton
      T[i]= recur(i, skeleton);

      }


  }
  /////////////////////////////////////////////////////////////////////////////

Eigen::Affine3d recur(int i, const Skeleton & skeleton){
  if (skeleton[i].parent_index == -1){ //root
    return Eigen::Affine3d::Identity(); //not moving
  }

  //  equation from the slide 

  //Skeletons: Forward Kinematics from the slide
  Eigen::Affine3d Tp = recur(skeleton[i].parent_index,skeleton); // Parent joint's Transformation
  Eigen::Affine3d Te = euler_angles_to_transform(skeleton[i].xzx); // Euler Angle Rotations
  Eigen::Affine3d Ti = skeleton[i].rest_T; // rest bone trans
  return Tp * Ti * Te * Ti.inverse();
}

