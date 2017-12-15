#include <HAL/Gamepad.pb.h>
#include <HAL/Gamepad/GamepadDevice.h>
#include <HAL/Gamepad/Drivers/GamepadMultiDevice/GSDK/Gamepad.h>
#include <HAL/Car.pb.h>
#include <HAL/Car/CarDevice.h>
#include <cmath>
#include <thread>
#include <HAL/Utils/GetPot>

#define graph 0
#define wireless_logitech 1
#define wired_logitech 0

int watch;
hal::CarCommandMsg commandMSG;
bool use_gamepad_ = false;
//bool stop_running_flag = false;
double throttle;

void GamepadCallback(hal::GamepadMsg& _msg) {
#if wireless_logitech
   double right = _msg.axes().data(5) + 1;
   double left = _msg.axes().data(2) + 1;
   std::cout << "Right: " << right*0.5 << "   Left: " << left*0.5 << std::endl;
  // update transmit command with gamepad data
  if(use_gamepad_) {

    if((right==1)||(left==1))
    {
         commandMSG.set_throttle_percent(0);
    }
    else if(right && !left)
    {
        commandMSG.set_throttle_percent(-(right*0.5)*25);
    }
    else if(left && !right)
    {
        commandMSG.set_throttle_percent((left*0.5)*25);
    }
    else
    {
        commandMSG.set_throttle_percent(0);
    }
    commandMSG.set_steering_angle(-_msg.axes().data(0));
    commandMSG.set_rear_steering_angle(-_msg.axes().data(3));
   watch=0;
  }
#endif
#if wired_logitech
    if(use_gamepad_)
    {
        throttle = _msg.axes().data(1);
        commandMSG.set_throttle_percent(_msg.axes().data(1)*25);
        commandMSG.set_steering_angle(-_msg.axes().data(0));
        commandMSG.set_rear_steering_angle(-_msg.axes().data(2));
        watch = 0;
    }
#endif

}

void CarSensorCallback(hal::CarStateMsg msg) {
  // here is an example of how to read data from the car
  if(1)
  {
#if !graph
   // std::cout << "PWM Duty Cycle - " << (int)msg.wheel_speed_fl() << "   Motor Speed - " << (int)msg.wheel_speed_fr()<< "   Error - "<< (int)msg.wheel_speed_rl() << "  Desired Speed - " << (int)msg.wheel_speed_rr() <<std::endl;//(int)msg.wheel_speed_fl() << "   " << (int)msg.wheel_speed_fr() << "   " << (int)msg.wheel_speed_rl() << "   "<< (int)msg.wheel_speed_rr()<< std::endl;
/*
    std::cout << "ADC ST:";
    std::cout.width(13);
    std::cout << msg.steer_angle();
    std::cout << "  RS:";
    std::cout.width(13);
    std::cout << msg.rear_steer_angle();

    std::cout << "  FL:  " <<msg.swing_angle_fl() << "  RL:  " << msg.swing_angle_rl() << "  RR:  " << msg.swing_angle_rr() << "  FR:  " << msg.swing_angle_fr() << "  MC:  " << msg.motor_current() << "  BV:  " << msg.batt_volt() ;//std::endl;
    std::cout << "    ENCODERS     FL:  ";
    std::cout.width(4);
    std::cout << (int)msg.wheel_speed_fl();
    std::cout << "  FR:  " << (int)msg.wheel_speed_fr() << "  RL:  " << (int)msg.wheel_speed_rl( ) << "  RR:  " <<  (int)msg.wheel_speed_rr() << std::endl;*/
    //std::cout << "     Motor Speed:  " <<  msg.device_time() << std::endl;
#endif
#if graph
    for(int i = 0;i<((abs(msg.wheel_speed_fr())/3));i++)
    {
        std::cout << " ";
    }
    std::cout << "|" << std::endl;

   int value =  throttle*600;
    for(int i = 0;i<((abs(value)/3));i++)
    {
        std::cout<< " ";
    }
    std::cout << "|" << std::endl;
#endif
  }
}


int main(int argc, char** argv) {
  GetPot cl_args(argc, argv);

  std::string car_uri = cl_args.follow("", "-car");
  use_gamepad_ = cl_args.search("-gamepad");

  hal::Gamepad gamepad("gamepad:/");
  // register gamepad event callback
  gamepad.RegisterGamepadDataCallback(&GamepadCallback);
  unsigned int devs;
  // Connect to NinjaV3Car
  // sample uri -> "ninja_v3:[baud=115200,dev=/dev/cu.usbserial-00002014A]//
  hal::Car ninja_car(car_uri);

  // register callback to receive sensory information from the car
  ninja_car.RegisterCarStateDataCallback(&CarSensorCallback);

  // initialize command packet
  commandMSG.set_steering_angle(0);
  commandMSG.set_throttle_percent(0);
  commandMSG.set_device_time(9);
  while(1) {
    // keep updating transmit buffer
     devs  =  Gamepad_numDevices();
    if(!devs)
    {
       commandMSG.set_throttle_percent(0);
       commandMSG.set_steering_angle(0);
       commandMSG.set_rear_steering_angle(0);
    }
    ninja_car.UpdateCarCommand(commandMSG);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return 0;
}
