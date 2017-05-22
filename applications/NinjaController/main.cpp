#include <HAL/Gamepad.pb.h>
#include <HAL/Gamepad/GamepadDevice.h>
#include <HAL/Car.pb.h>
#include <HAL/Car/CarDevice.h>
#include <cmath>
#include <thread>
#include <HAL/Utils/GetPot>
//#include <spirit/spirit.h>
//#include <spirit/Types/spTypes.h>
#include "spPID.h"

//#define USE_SIMULATION_CAR
#define TrajL 5
#define TrajR 2
#define TrajB 1
#define MAX_THROTTLE 30

hal::CarCommandMsg commandMSG;

void GamepadCallback(hal::GamepadMsg& _msg) {
    commandMSG.set_throttle_percent(_msg.axes().data(2)*40);
}

int GetArea(double x, double y) {
  if((x>=-TrajB) && (x<TrajR) && (y>=0) && (y<TrajL)) {
    return 0;
  } else if((x>=-TrajB) && (x<TrajR) && (y>=TrajL) && (y<TrajL+TrajR+TrajB)) {
    return 1;
  } else if((x>=TrajR) && (x<TrajR+TrajB) && (y>=TrajL) && (y<TrajL+TrajR+TrajB)) {
    return 2;
  } else if((x>=TrajR) && (x<TrajR+TrajB) && (y>=0) && (y<TrajL)) {
    return 3;
  } else if((x>=TrajR) && (x<TrajR+TrajB) && (y>=-TrajR-TrajB) && (y<0)) {
    return 4;
  } else if((x>=-TrajB) && (x<TrajR) && (y>=-TrajR-TrajB) && (y<0)) {
    return 5;
  } else {
    return -1;
  }
}

double GetCrossTrackError(double x, double y, int current_area) {
  if(current_area == 0) {
    return x;
  } else if(current_area == 1) {
    // in cordinates of T1 we have
    double tx = x-TrajR;
    double ty = y-TrajL;
    double tdist = sqrt(tx*tx+ty*ty);
    return -(tdist-TrajR);
  } else if(current_area == 2) {
    // in cordinates of T2 we have
    double tx = -y+TrajL;
    double ty = x-TrajR;
    double tdist = sqrt(tx*tx+ty*ty);
    return -(tdist-TrajR);
  } else if(current_area == 3) {
    // in cordinates of T3 we have
    double tx = 2*TrajR-x;
//    double ty = TrajL-y;
    return tx;
  } else if(current_area == 4) {
    // in cordinates of T4 we have
    double tx = -x+TrajR;
    double ty = -y;
    double tdist = sqrt(tx*tx+ty*ty);
    return -(tdist-TrajR);
  } else if(current_area == 5) {
    // in cordinates of T5 we have
    double tx = y;
    double ty = -x+TrajR;
    double tdist = sqrt(tx*tx+ty*ty);
    return -(tdist-TrajR);
  } else {
    return 0;
  }
}

int main(int argc, char** argv) {
  GetPot cl_args(argc, argv);

  // capture command line arguments
  std::string car_uri = cl_args.follow("", "-car");
  use_gamepad_ = cl_args.search("-gamepad");

  // create gamepad object
  hal::Gamepad gamepad("gamepad:/");

  // register gamepad event callback
  gamepad.RegisterGamepadDataCallback(&GamepadCallback);

#ifndef USE_SIMULATION_CAR
  // Connect to NinjaV3Car
  // sample uri -> "ninja_v3:[baud=115200,dev=/dev/cu.usbserial-00002014A]//
  hal::Car ninja_car(car_uri);
#endif

  // register callback to receive sensory information from the car
//  ninja_car.RegisterCarStateDataCallback(&CarSensorCallback);

  // initialize command packet
  commandMSG.set_steering_angle(0);
  commandMSG.set_throttle_percent(0);

  spPID controller;
  controller.SetGainP(10);
  controller.SetGainD(0.2);
  controller.SetGainI(0.01);

  while(1) {
    // get current pose of the vehicle
    double x;
    double y;
    //////////////////////////
    /// VI Tracker code here
    ///
    //////////////////////////

    // find which area the car is in
    double curr_area = GetArea(x,y);
    // calculate crosstrack error
    double cte = GetCrossTrackError(x,y,curr_area);
    // calculate control signal
    controller.SetPlantError(cte);
    // apply control signal to the vehicle
    double cv = controller.GetControlOutput();
    // trim control signal
    if(cv>1) {
      cv = 1;
    } else if(cv<-1) {
      cv = -1;
    }
    commandMSG.set_steering_angle(cv);
#ifndef USE_SIMULATION_CAR
    ninja_car.UpdateCarCommand(commandMSG);
#endif
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }


  return 0;
}
