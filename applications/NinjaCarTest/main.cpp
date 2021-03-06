#include <HAL/Gamepad.pb.h>
#include <HAL/Gamepad/GamepadDevice.h>
#include <HAL/Car.pb.h>
#include <HAL/Car/CarDevice.h>
#include <cmath>
#include <thread>
#include <HAL/Utils/GetPot>

hal::CarCommandMsg commandMSG;
bool use_gamepad_ = false;

void GamepadCallback(hal::GamepadMsg& _msg) {
  std::cout << "steeromg command is " << _msg.axes().data(2) << std::endl;
  std::cout << "throttle command is " << _msg.axes().data(1)*20 << std::endl;
  // update transmit command with gamepad data
  if(use_gamepad_) {
    commandMSG.set_steering_angle(_msg.axes().data(0));
    commandMSG.set_rear_steering_angle(_msg.axes().data(2));
    commandMSG.set_throttle_percent(_msg.axes().data(1)*40);
  }
}

void CarSensorCallback(hal::CarStateMsg msg) {
  // here is an example of how to read data from the car
  std::cout << "state data received: " << msg.batt_volt() << std::endl;
}


int main(int argc, char** argv) {
  GetPot cl_args(argc, argv);

  std::string car_uri = cl_args.follow("", "-car");
  use_gamepad_ = cl_args.search("-gamepad");

  hal::Gamepad gamepad("gamepad:/");
  // register gamepad event callback
  gamepad.RegisterGamepadDataCallback(&GamepadCallback);

  // Connect to NinjaV3Car
  // sample uri -> "ninja_v3:[baud=115200,dev=/dev/cu.usbserial-00002014A]//
  hal::Car ninja_car("ninja_v3:[baud=115200,dev=/dev/ttyUSB0]");

  // register callback to receive sensory information from the car
  ninja_car.RegisterCarStateDataCallback(&CarSensorCallback);

  // initialize command packet
  commandMSG.set_steering_angle(0);
  commandMSG.set_throttle_percent(5);
  commandMSG.set_device_time(9);
  int wave = 0;
  float freq = 1;
  while(1) {
    // if joystick not used then send a sine wave as steering signal
    if(!use_gamepad_) {
      // set sine wave period to 100*10ms
      if(wave >= 628) {
        wave = 0;
      }
      std::cout << "wave is -> " << std::sin(freq*(wave/100.0)) << std::endl;
      commandMSG.set_steering_angle(std::sin(freq*(wave/100.0)));
      wave++;
    }
    // keep updating transmit buffer
    ninja_car.UpdateCarCommand(commandMSG);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}
