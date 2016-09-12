#include <HAL/Gamepad.pb.h>
#include <HAL/Gamepad/GamepadDevice.h>
#include <HAL/Car.pb.h>
#include <HAL/Car/CarDevice.h>

hal::CarCommandMsg commandMSG;

void GamepadCallback(hal::GamepadMsg& _msg) {

//  std::cout << "-> "
//            << _msg.axes().data(0) << ", "
//            << _msg.axes().data(1) << ", "
//            << _msg.axes().data(2) << ", "
//            << _msg.axes().data(3) << ", "
//            << _msg.axes().data(4) << ", "
//            << _msg.axes().data(5) << ", "
//            << _msg.axes().data(6) << ", "
//            << _msg.axes().data(7) << ", "
//            << _msg.axes().data(8) << ", "
//            << _msg.axes().data(9) << ", "
//            << _msg.axes().data(10) << ", "
//            << _msg.axes().data(11) << ", "
//            << _msg.axes().data(12) << ", "
//            << _msg.axes().data(13) << ", "
//            << _msg.axes().data(14) << " -  "
//            << _msg.buttons().data(0) << ","
//            << _msg.buttons().data(1) << ","
//            << _msg.buttons().data(2) << ","
//            << _msg.buttons().data(3) << ","
//            << _msg.buttons().data(4) << ","
//            << _msg.buttons().data(5) << ","
//            << _msg.buttons().data(6) << ","
//            << _msg.buttons().data(7) << ","
//            << _msg.buttons().data(8) << ","
//            << _msg.buttons().data(9) << ","
//            << _msg.buttons().data(10) << ","
//            << _msg.buttons().data(11) << ","
//            << std::endl;

  // update transmit command with gamepad data
  commandMSG.set_throttle_percent(-_msg.axes().data(1));
  commandMSG.set_steering_angle(-_msg.axes().data(2));
//  std::cout << "Jostick Command Set" << std::endl;
}

void CarSensorCallback(hal::CarStateMsg msg) {
//  std::cout << "***********************" << std::endl;
//  std::cout << "Wheel Speed FL: " << msg.wheel_speed_fl() << std::endl;
//  std::cout << "Wheel Speed FR: " << msg.wheel_speed_fr() << std::endl;
//  std::cout << "Wheel Speed RL: " << msg.wheel_speed_rl() << std::endl;
//  std::cout << "Wheel Speed RR: " << msg.wheel_speed_rr() << std::endl;
//  std::cout << "Swing Arm Angle FL :" << msg.swing_angle_fl() << std::endl;
//  std::cout << "Swing Arm Angle FR :" << msg.swing_angle_fr() << std::endl;
//  std::cout << "Swing Arm Angle RL :" << msg.swing_angle_rl() << std::endl;
//  std::cout << "Swing Arm Angle RR :" << msg.swing_angle_rr() << std::endl;
//  std::cout << "Steering Angle :" << msg.steer_angle() << std::endl;
//  std::cout << "Motor Current :" << msg.motor_current() << std::endl;
}

int main(int argc, char** argv) {
  // connect to a gamepad
  hal::Gamepad gamepad("gamepad:/");
  gamepad.RegisterGamepadDataCallback(&GamepadCallback);

  // Connect to NinjaV3Car
  hal::Car ninja_car("ninja_v3:[baud=115200,dev=/dev/cu.usbserial-00001014A]//");

  ninja_car.RegisterCarStateDataCallback(&CarSensorCallback);

  // initialize command packet
  commandMSG.set_steering_angle(5.1);
  commandMSG.set_throttle_percent(6.1);
  commandMSG.set_device_time(7.1);

  while(1) {
    // keep updating transmit buffer
    ninja_car.UpdateCarCommand(commandMSG);
  }
  return 0;
}

