#include <iostream>
#include <iomanip>
#include <HAL/Posys/PosysDevice.h>

hal::PoseMsg prev_pose_;

void optitrack_pose_handler(hal::PoseMsg& PoseData) {
  std::cout << "Received Pose Data ->\t";
  std::cout << std::fixed;
  std::cout << std::setprecision(8);
  std::cout << "\tx: " << PoseData.pose().data(0);
  std::cout << "\ty: " << PoseData.pose().data(1);
  std::cout << "\tz: " << PoseData.pose().data(2);
  std::cout << "\tq1: " << PoseData.pose().data(3);
  std::cout << "\tq2: " << PoseData.pose().data(4);
  std::cout << "\tq3: " << PoseData.pose().data(5);
  std::cout << "\tq4: " << PoseData.pose().data(6);
  std::cout << "\tdev_t: " << PoseData.device_time();
  std::cout << "\tsys_t: " << PoseData.system_time();

  // Calculate Velocities
  if(prev_pose_.has_type()){
    double inv_time_dif = 1/(PoseData.device_time()-prev_pose_.device_time());
    double vx = inv_time_dif*(PoseData.pose().data(0)-prev_pose_.pose().data(0));
    double vy = inv_time_dif*(PoseData.pose().data(1)-prev_pose_.pose().data(1));
    double vz = inv_time_dif*(PoseData.pose().data(2)-prev_pose_.pose().data(2));
    std::cout << "\tvx: " << vx;
    std::cout << "\tvy: " << vy;
    std::cout << "\tvz: " << vz;
  }
  std::cout << std::endl;

  // system time vs Device time diff
  std::cout << "dev-diff: " << PoseData.device_time()-prev_pose_.device_time();
  std::cout << "\tsys-diff: " << PoseData.system_time()-prev_pose_.system_time() << std::endl;

  prev_pose_ = PoseData;
}

int main() {
  std::cout << "Msg: Make sure you are on same network as Optitrack system." << std::endl;
  std::cout << "Connecting to tracker object with name 'dummy' ..." << std::endl;
  hal::Posys vicon("vicon://tracker:[dummy]");

  std::cout << "Registering pose callback ..." << std::endl;
  vicon.RegisterPosysDataCallback(&optitrack_pose_handler);

  std::cout << "Waiting for pose data ..." << std::endl;

  while(1) {}

  return 0;
}
