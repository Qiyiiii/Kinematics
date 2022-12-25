#include "catmull_rom_interpolation.h"
#include <Eigen/Dense>

Eigen::Vector3d catmull_rom_interpolation(
  const std::vector<std::pair<double, Eigen::Vector3d> > & keyframes,
  double t)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  if (keyframes.size() == 0) { // no frame
  	return Eigen::Vector3d(0, 0, 0);
  }
  if (keyframes.size() == 1) { // only one frame
  	return keyframes[0].second; //keep as it is
  }
  // check if t is larger than last fram time
  t = std::fmin(t, keyframes[keyframes.size()-1].first);
  if (keyframes.size() == 2) { // only two frames
  	return (keyframes[1].second - keyframes[0].second)/(keyframes[1].first - keyframes[0].first) * t; //linear interpolation
  }

   int ind = 0;
  for (int i = 0; i < keyframes.size() - 1; i++){
      if ((keyframes[ind].first >= t)){
        break;
      } //get the last index before t
          ind ++;
  }

  if (keyframes.size() == 3) { // only three frames

  	return (keyframes[ind].second - keyframes[ind-1].second)/(keyframes[ind].first - keyframes[ind-1].first) * t; //linear interpolation
  
  }

  //four frame
  Eigen::Vector3d p0, p1, p2, p3;
  double t0, t1, t2, t3;
  if (ind - 2 >= 0 && ind + 1 < keyframes.size()){ //if all ind valid
    Eigen::Vector3d p0, p1, p2, p3;
    double t0, t1, t2, t3;
    p0 = keyframes[ind-2].second;  
    p1 = keyframes[ind-1].second;
    p2 = keyframes[ind].second; 
    p3 = keyframes[ind+1].second; 

    t0 = keyframes[ind-2].first;
    t1 = keyframes[ind-1].first;
    t2 = keyframes[ind].first;
    t3 = keyframes[ind+1].first;


    //Directly adopted from https://en.wikipedia.org/wiki/Centripetal_Catmull–Rom_spline

    Eigen::Vector3d A1 = (t1 - t)/(t1 - t0) * p0 + (t - t0)/(t1 - t0) * p1;
    Eigen::Vector3d A2 = (t2 - t)/(t2 - t1) * p1 + (t - t1)/(t2 - t1) * p2;
    Eigen::Vector3d A3 = (t3 - t)/(t3 - t2) * p2 + (t - t2)/(t3 - t2) * p3;
    Eigen::Vector3d B1 = (t2 - t)/(t2 - t0) * A1 + (t - t0)/(t2 - t0) * A2;
    Eigen::Vector3d B2 = (t3 - t)/(t3 - t1) * A2 + (t - t1)/(t3 - t1) * A3;

    Eigen::Vector3d C = (t2 - t)/(t2 - t1) * B1 + (t - t1)/(t2 - t1) * B2;

    return C;

  }
  else{ //if 4 ind not valid then linear interpolation
    return (keyframes[ind].second - keyframes[ind-1].second)/(keyframes[ind].first - keyframes[ind-1].first) * t; //linear interpolation

  }





  return Eigen::Vector3d(0,0,0);
  /////////////////////////////////////////////////////////////////////////////
}
