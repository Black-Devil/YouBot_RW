 
#include <iostream>
#include <sys/time.h>
#include "inverse_kinematics.cpp"

int main(int argc, char **argv) {
   std::vector<double> lower_limits;
   std::vector<double> upper_limits;
   lower_limits.push_back(-2.949606435870417);
   lower_limits.push_back(-1.1344640137963142);
   lower_limits.push_back(-2.6354471705114375);
   lower_limits.push_back(-1.7802358370342162);
   lower_limits.push_back(-2.91469985083053);
   upper_limits.push_back(2.949606435870417);
   upper_limits.push_back(1.5707963267948966);
   upper_limits.push_back(2.548180707911721);
   upper_limits.push_back( 1.7802358370342162);
   upper_limits.push_back(2.91469985083053);

   InverseKinematics *tmp = new InverseKinematics(lower_limits, upper_limits);
   KDL::Frame frame(KDL::Rotation::Identity(), KDL::Vector(0, 0, 0.5));
   std::vector<KDL::JntArray> *test = new std::vector<KDL::JntArray>();
   bool erg = tmp->CartToJnt(frame, *test);
   std::cout<<test[0][0](0)<<std::endl;
   std::cout<<test[0][0](1)<<std::endl;
   std::cout<<test[0][0](2)<<std::endl;
   std::cout<<test[0][0](3)<<std::endl;
   std::cout<<test[0][0](4)<<std::endl;
}
