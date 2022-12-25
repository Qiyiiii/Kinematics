#include "copy_skeleton_at.h"
Skeleton copy_skeleton_at(
  const Skeleton & skeleton, 
  const Eigen::VectorXd & A)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  Skeleton s = skeleton; //copy of skeleton
  for (int i = 0; i < skeleton.size(); ++i) {
     // joint angles set to A
      s[i].xzx.x() = A(i * 3);
      s[i].xzx.y() = A(i * 3 + 1);
      s[i].xzx.z() = A(i * 3 + 2);
  }
  return s;

 
  /////////////////////////////////////////////////////////////////////////////
}
