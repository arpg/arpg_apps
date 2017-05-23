#include <HAL/Gamepad.pb.h>
#include <HAL/Gamepad/GamepadDevice.h>
#include <HAL/Car.pb.h>
#include <HAL/Car/CarDevice.h>
#include <cmath>
#include <thread>
#include <HAL/Utils/GetPot>
#include <spirit/spirit.h>
#include <spirit/Types/spTypes.h>

#define USE_SIMULATION_CAR
#define TrajL 4
#define TrajR 3
#define TrajB 2
#define MAX_THROTTLE 60

hal::CarCommandMsg commandMSG;

void GamepadCallback(hal::GamepadMsg& _msg) {
    commandMSG.set_throttle_percent(_msg.axes().data(2)*40);
}

int GetArea(double x, double y) {
  if((x>=-TrajB) && (x<TrajR) && (y>=0) && (y<TrajL)) {
    return 0;
  } else if((x>=-TrajB) && (x<TrajR) && (y>=TrajL) && (y<TrajL+TrajR+TrajB)) {
    return 1;
  } else if((x>=TrajR) && (x<2*TrajR+TrajB) && (y>=TrajL) && (y<TrajL+TrajR+TrajB)) {
    return 2;
  } else if((x>=TrajR) && (x<2*TrajR+TrajB) && (y>=0) && (y<TrajL)) {
    return 3;
  } else if((x>=TrajR) && (x<2*TrajR+TrajB) && (y>=-TrajR-TrajB) && (y<0)) {
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


#ifdef USE_SIMULATION_CAR
 /// create spirit simulated car
 // create a gui
  Gui gui_;
  gui_.Create(spGuiType::GUI_PANGOSCENEGRAPH);
  // create an empty set of objects
  Objects objects_;
  // create floor as a box and add to both gui and objects set
  spPose gnd_pose_ = spPose::Identity();
  gnd_pose_.translate(spTranslation(0,0,-0.5));
  spObjectHandle gnd_handle = objects_.CreateBox(gnd_pose_,spBoxSize(20,20,1),0,spColor(0,1,0));
  gui_.AddObject(objects_.GetObject(gnd_handle));
  // set friction coefficent of ground (btw 0-1)
  ((spBox&)objects_.GetObject(gnd_handle)).SetFriction(1);
  ((spBox&)objects_.GetObject(gnd_handle)).SetRollingFriction(0.1);
  // create a AWD car
  // first create an object for parameters of vehicle
  spVehicleConstructionInfo simcar_param;
  simcar_param.vehicle_type = spObjectType::VEHICLE_AWSD;
  simcar_param.wheels_anchor.push_back(spTranslation(-0.13, 0.17, -0.003));
  simcar_param.wheels_anchor.push_back(spTranslation(-0.13, -0.17, -0.003));
  simcar_param.wheels_anchor.push_back(spTranslation(0.13, -0.17, -0.003));
  simcar_param.wheels_anchor.push_back(spTranslation(0.13, 0.17, -0.003));
  simcar_param.chassis_size = spBoxSize(0.2, 0.42, 0.05);
  simcar_param.cog = spTranslation(0, 0, 0);
  simcar_param.chassis_friction = 0;
  simcar_param.wheel_rollingfriction = 0.1;
  simcar_param.wheel_friction = 0.4;
  simcar_param.wheel_width = 0.04;
  simcar_param.wheel_radius = 0.057;
  simcar_param.susp_damping = 10;
  simcar_param.susp_stiffness = 100;
  simcar_param.susp_preloading_spacer = 0.1;
  simcar_param.susp_upper_limit = 0.013;
  simcar_param.susp_lower_limit = -0.028;
  simcar_param.wheel_mass = 0.4;
  simcar_param.chassis_mass = 5;
  simcar_param.steering_servo_lower_limit = -SP_PI_QUART;
  simcar_param.steering_servo_upper_limit = SP_PI_QUART;
  // create a car with parameters above
  spObjectHandle car_handle = objects_.CreateVehicle(simcar_param);
  gui_.AddObject(objects_.GetObject(car_handle));
  // set the car in center of world and above ground
  spAWSDCar& simcar = (spAWSDCar&) objects_.GetObject(car_handle);
  spPose carpose(spPose::Identity());
  carpose.translate(spTranslation(0,0,0.06));
  simcar.SetPose(carpose);
  // set current engine torques and fix back steering angle
  simcar.SetEngineTorque(20);
  simcar.SetEngineMaxVel(50);
  simcar.SetSteeringServoMaxVel(100);
  simcar.SetSteeringServoTorque(100);
  simcar.SetRearSteeringAngle(0);
  // now car is ready to drive
#endif

  // create PIDcontroller
  spPID controller;
  controller.SetGainP(50);
  controller.SetGainD(0.2);
  controller.SetGainI(0.01);

  while(1) {
#ifdef USE_SIMULATION_CAR
    if(gui_.ShouldQuit()) {
      return 0;
    }
#endif
    // get current pose of the vehicle
    double x;
    double y;
#ifdef USE_SIMULATION_CAR
    x = simcar.GetPose().translation()[0];
    y = simcar.GetPose().translation()[1];
#else
    //////////////////////////
    /// VI Tracker code here
    ///
    //////////////////////////
#endif
    // find which area the car is in based on its position
    double curr_area = GetArea(x,y);
    // calculate crosstrack error from trajectory
    double cte = GetCrossTrackError(x,y,curr_area);
    std::cout << "cte is " << cte << std::endl;
    // calculate control signal
    controller.SetPlantError(cte);
    // apply control signal to the vehicle
    double cv = controller.GetControlOutput();
    // trim control signal since NinjaECU only accepts in range [-1,1]
    if(cv>1) {
      cv = 1;
    } else if(cv<-1) {
      cv = -1;
    }
    // set throttle command to constant if safity area has not been passed
    double throttle_cmd;
    if(curr_area == -1) {
      // if safity area passed then stop vehicle
      throttle_cmd = 0;
    } else {
      throttle_cmd = MAX_THROTTLE;
    }

#ifdef USE_SIMULATION_CAR
    // update cars control commands
    simcar.SetFrontSteeringAngle(-simcar_param.steering_servo_upper_limit*cv);
    simcar.SetEngineMaxVel(throttle_cmd);
    // iterate simulation forward for 10ms
    objects_.StepPhySimulation(0.01);
    // update all affected objects in world
    gui_.Iterate(objects_);
#else
    commandMSG.set_steering_angle(cv);
    commandMSG.set_throttle_percent(throttle_cmd);
    ninja_car.UpdateCarCommand(commandMSG);
#endif
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }


  return 0;
}
