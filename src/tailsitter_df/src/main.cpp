#include <stdio.h>      /* printf */
#include <math.h>       /* atan */
#include <tailsitter_df/df.hpp>
#define PI 3.14159265

int main ()
{
  // // double param, result;
  // // param = 10000000.0;
  // // result = atan (param) * 180 / PI;
  // // printf ("The arc tangent of %f is %f degrees\n", param, result );

  // // tailsitter_df::Model& tf = tailsitter_df::Model::getInstance();
  // // std::cout << tf.getPhi();
  // // std::cout << '---------------------';

  // // // tailsitter_df::df_test();

  // // std::cout << '---------------------\n';
  Eigen::Vector3i mm ;



  mm << 1,2,3;
  std::cout << mm.transpose() << "\n";
  std::cout << mm.prod();
  return 0;
}