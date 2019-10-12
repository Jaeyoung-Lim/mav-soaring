//  October/2019, Auterion AG, Jaeyoung Lim, jaeyoung@auterion.com

#include "thermal_soaring/thermal_soaring.h"

//using namespace RAI;
int main(int argc, char** argv) {
  ros::init(argc,argv,"thermal_soaring_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ThermalSoaring *thermal_soaring_controller = new ThermalSoaring(nh, nh_private);

  ros::spin();
  return 0;
}
