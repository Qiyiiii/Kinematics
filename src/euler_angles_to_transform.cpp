#include "euler_angles_to_transform.h"

Eigen::Affine3d euler_angles_to_transform(
  const Eigen::Vector3d & xzx)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  Eigen::Affine3d A;
  // xzx in degrees but AngleAxis expects radians
  //xzx(2) is the first angle to rotate by
  double x1_rad  = xzx.z()/180 * M_PI;
  double z_rad  = xzx.y()/180 * M_PI;
  double x2_rad  = xzx.x()/180 * M_PI;
  //https://eigen.tuxfamily.org/dox/classEigen_1_1AngleAxis.html
  // A.matrix() << Eigen::AngleAxisf(x1_rad , Eigen::Vector3f::UnitX()).toRotationMatrix()
  // * Eigen::AngleAxisf(z_rad,  Eigen::Vector3f::UnitZ()).toRotationMatrix()
  // * Eigen::AngleAxisf(x2_rad , Eigen::Vector3f::UnitX()).toRotationMatrix();
  
  // Eigen::Affine3d r1= Eigen::Affine3d(Eigen::AngleAxisf(x1_rad , Eigen::Vector3d::UnitX()).toRotationMatrix());
  // Eigen::Affine3d r2= Eigen::Affine3d(Eigen::AngleAxisf(z_rad , Eigen::Vector3d::UnitX()).toRotationMatrix());
  // Eigen::Affine3d r3= Eigen::Affine3d(Eigen::AngleAxisf(x2_rad , Eigen::Vector3d::UnitX()).toRotationMatrix());
  // return Eigen::Affine3d(r3 * r2 * r1);

  return  Eigen::Affine3d(Eigen::AngleAxis<double>(x1_rad, Eigen::Vector3d::UnitX())*
  Eigen::AngleAxis<double>(z_rad, Eigen::Vector3d::UnitZ())*
  Eigen::AngleAxis<double>(x2_rad, Eigen::Vector3d::UnitX()));

  

  
  







  /////////////////////////////////////////////////////////////////////////////
}
