#include "kinematics_jacobian.h"
#include "transformed_tips.h"
#include <iostream>

void kinematics_jacobian(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  Eigen::MatrixXd & J)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code

  //central difference equation from the tut slide posted
  J = Eigen::MatrixXd::Zero(b.size()*3,skeleton.size()*3);

  double h = 0.0000001;
  Eigen::VectorXd trans1;
  Eigen::VectorXd trans2;

  for (int i = 0; i < skeleton.size(); ++i) {
      for (int j = 0; j < 3; ++j) {
          Skeleton s1 = skeleton; //copy1
          Skeleton s2 = skeleton; //copy2

          s1[i].xzx[j] += h;
          s2[i].xzx[j] -= h;
          trans1 = transformed_tips(s1, b); 
          trans2 = transformed_tips(s2, b);

          for (int k = 0; k < J.rows(); ++k) { //Each i,j index gives a 3x3 submatrix
              J(k, 3 * i +j) = (trans1 -  trans2)[k] / (2*h);
          }
      }
  }

  /////////////////////////////////////////////////////////////////////////////
}
