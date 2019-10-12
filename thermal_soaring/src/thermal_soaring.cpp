//  October/2019, Auterion AG, Jaeyoung Lim, jaeyoung@auterion.com

#include "thermal_soaring/thermal_soaring.h"

using namespace Eigen;
using namespace std;
//Constructor
ThermalSoaring::ThermalSoaring(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {


}
ThermalSoaring::~ThermalSoaring() {
  //Destructor
}

void ThermalSoaring::cmdloopCallback(const ros::TimerEvent& event){
}

void ThermalSoaring::statusloopCallback(const ros::TimerEvent& event){
}