#include "linear_blend_skinning.h"

void linear_blend_skinning(
  const Eigen::MatrixXd & V,
  const Skeleton & skeleton,
  const std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T,
  const Eigen::MatrixXd & W,
  Eigen::MatrixXd & U)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  // eqation from tut slide
  U.resize(V.size(), 3); 
  for (int i = 0; i < V.rows(); i++){
    Eigen::Vector4d s(0,0,0,0), nv(V(i, 0), V(i, 1), V(i, 2), 1); // change V.row(i) to 4d
     for (int j=0; j < skeleton.size(); j++) {
       if (skeleton[j].weight_index != -1){ //if has associated weights
        double wei = W(i, skeleton[j].weight_index); //weight 
        s += wei * (T[j] * nv); //sum the weighted contributions of all the bones

       }
     }
      U.row(i) << Eigen::Vector3d(s.x(), s.y(),s.z()).transpose(); 

  }
  /////////////////////////////////////////////////////////////////////////////
}


