#include "end_effectors_objective_and_gradient.h"
#include "transformed_tips.h"
#include "kinematics_jacobian.h"
#include "copy_skeleton_at.h"
#include <iostream>

void end_effectors_objective_and_gradient(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  const Eigen::VectorXd & xb0,
  std::function<double(const Eigen::VectorXd &)> & f,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & grad_f,
  std::function<void(Eigen::VectorXd &)> & proj_z)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code

  //eqations from the slides
  f = [&](const Eigen::VectorXd & A)->double
  {
    Skeleton c = copy_skeleton_at(skeleton, A); 
    Eigen::VectorXd xb = transformed_tips(c, b); //pose tip 
    return (xb - xb0).squaredNorm()/2;  // why do we need "1/2 factor in front here"
    

  };

  grad_f = [&](const Eigen::VectorXd & A)->Eigen::VectorXd
  {
    Skeleton c = copy_skeleton_at(skeleton, A);
    Eigen::VectorXd xb = transformed_tips(c, b);
    Eigen::MatrixXd J; //dx/da
    kinematics_jacobian(c, b, J);
    Eigen::VectorXd dexdx = (xb - xb0);
    return J.transpose() * dexdx; //dx/da^T * de/dx
  };


  proj_z = [&](Eigen::VectorXd & A)
  {
    for (int i = 0; i < skeleton.size(); i++) {
      for (int j = 0; j < 3; j++) { //loop through bones*3 list of Euler angles euler angles
        A(i * 3 + j) = std::max(skeleton[i].xzx_min(j), std::min(skeleton[i].xzx_max(j), A(i * 3 + j)));
      }
    }
  };

  /////////////////////////////////////////////////////////////////////////////
}
