#include "line_search.h"
#include <iostream>

double line_search(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const Eigen::VectorXd & z,
  const Eigen::VectorXd & dz,
  const double max_step)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code

  // equation from line_search slide and tut slide
  double sigma = max_step;
  
for (int i=0; i<= 20; i++){
    Eigen::VectorXd z1 = z - sigma * dz; //a + delta * a
    proj_z(z1);
    if (f(z1) < f(z)){ // if the energy is decreasing
      return sigma; 
    }
    sigma/=2; // decrease by 1/2 
  }
  return 0;
  /////////////////////////////////////////////////////////////////////////////
}
