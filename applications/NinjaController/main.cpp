#include <HAL/Gamepad.pb.h>
#include <HAL/Gamepad/GamepadDevice.h>
#include <HAL/Car.pb.h>
#include <HAL/Car/CarDevice.h>
#include <cmath>
#include <thread>
#include <HAL/Utils/GetPot>
#include "spPID.h"

//#define USE_SIMULATION_CAR
#define TrajL 4
#define TrajR 3
#define TrajB 2
#define MAX_THROTTLE 0

#ifdef USE_SIMULATION_CAR
#include <spirit/spirit.h>
#include <spirit/Types/spTypes.h>
#endif

// COMPASS includes
#include <compass/processing/System.h>
#include <compass/common/ParameterReader.h>
#include <HAL/Camera/CameraDevice.h>
#include <HAL/IMU/IMUDevice.h>
#include <HAL/Messages/Matrix.h>
#include "InterpolationBuffer.h"
#include <iomanip>


using namespace compass;
using namespace std;

struct measurement {
    measurement() {}

    measurement(const Eigen::Matrix<double, 3, 1>& w,
                const Eigen::Matrix<double, 3, 1>& a, const double time)
        : w(w), a(a), timestamp(time) {
    }

    /// \brief angular rates in inertial coordinates
    Eigen::Matrix<double, 3, 1> w;
    /// \brief accelerations in inertial coordinates
    Eigen::Matrix<double, 3, 1> a;
    double timestamp;

    measurement operator*(const double &rhs) const {
        return measurement(w * rhs, a * rhs, timestamp);
    }

    measurement operator+(const measurement &rhs) const {
        return measurement(w + rhs.w, a + rhs.a, timestamp);
    }
};

hal::Camera camera_device;
hal::IMU imu_device;
std::shared_ptr<hal::Image> camera_img;
int image_width;
int image_height;
compass::Time image_timestamp;
compass::Duration imu_time_offset;
compass::Time prev_frame_time =  compass::Time(0.0);
compass::InterpolationBufferT<measurement,
double> imu_buffer;
bool first_imu_window_ = true;
bool use_system_time = true;
bool should_capture = true;
std::mutex latest_position_mutex;
std::mutex imu_buffer_mutex;
cv::Mat im;
bool capture_success = false;

std::shared_ptr<hal::ImageArray> images = hal::ImageArray::Create();
std::shared_ptr<std::thread> measurement_consumer_thread;
struct Position{
    Position():
        x(0.0), y(0.0){}
    double x;
    double y;
};

Position latest_position;

compass::CompassParameters params;
std::shared_ptr<compass::System> SLAMSystem;

bool LoadDevices(string cam_uri, string imu_uri);
bool LoadIMU(string imu_uri);
void ImuCallback(const hal::ImuMsg &ref);
void StateCallback(const compass::Time & t,
                   const compass::kinematics::Transformation & T_w_v,
                   const Eigen::Matrix<double, 9, 1> &speed_and_bias,
                   const Eigen::Matrix<double, 3, 1> &omega_S);
bool InitializeCompass(std::string settings, std::string cam, std::string imu);
//void MeasurementConsumerLoop();
void ConsumeMeasurements();


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
    std::string cam_uri = cl_args.follow("", "-cam");
    std::string imu_uri = cl_args.follow("", "-imu");
    std::string settings_uri = cl_args.follow("", "-settings");

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

    // setup compass and start processing measurements
    InitializeCompass(settings_uri, cam_uri, imu_uri);

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
        {
            ConsumeMeasurements();
            std::lock_guard<std::mutex>lck(latest_position_mutex);
            x = latest_position.x;
            y = latest_position.y;
        }
        //////////////////////////
#endif
        // find which area the car is in based on its position
        double curr_area = GetArea(x,y);
        // calculate crosstrack error from trajectory
        double cte = GetCrossTrackError(x,y,curr_area);
        std::cout << "cte is " << cte << std::endl;
        // calculate control signal
        //controller.SetPlantError(cte);
        // apply control signal to the vehicle
        double cv = controller.GetControlOutput();
        // trim control signal since NinjaECU only accepts in range [-1,1]
        if(cv>0.7) {
            cv = 0.7;
        } else if(cv<-0.7) {
            cv = -0.7;
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
	std::cout << "cv is -> " << cv << std::endl;
        commandMSG.set_steering_angle(cv);
        commandMSG.set_throttle_percent(throttle_cmd);
        ninja_car.UpdateCarCommand(commandMSG);
#endif
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }


    return 0;
}


bool InitializeCompass(std::string settings, std::string cam, std::string imu){
    cout << "Initializing Compass..." << endl;
    compass::ParameterReader param_reader(settings);
    param_reader.GetParameters(params);

    SLAMSystem = std::make_shared<compass::System>(params);
    SLAMSystem->SetBlocking(true);
    SLAMSystem->SetFullStateCallback(&StateCallback);

    cout  << "Initializing devices..." << endl;
    if(!LoadDevices(cam, imu)){
        cerr << "Error loading devices.";
        return false;
    }

    //    // now start collecting measurements
    //    measurement_consumer_thread =
    //            std::shared_ptr<std::thread>(new std::thread(&MeasurementConsumerLoop));


    return true;

}

/*-------------- START MEASUREMENT CONSUMER THREADS-----------------------*/

void ConsumeMeasurements(){
    std::shared_ptr<hal::ImageArray> images = hal::ImageArray::Create();
    capture_success = false;

    // Capture an image
    capture_success = camera_device.Capture(*images);
    if(!capture_success)
        std::cerr << "Image capture failed..." << std::endl;

    if (capture_success){

        camera_img = images->at(0);
        image_width = camera_img->Width();
        image_height = camera_img->Height();

        image_timestamp = compass::Time(use_system_time ?
                                            images->Ref().system_time():
                                            images->Ref().device_time());

        if(!use_system_time){
            image_timestamp += imu_time_offset;
        }

        std::vector<cv::Mat> cvmat_images;
        for (int ii = 0; ii < images->Size() ; ++ii) {
            cvmat_images.push_back(images->at(ii)->Mat());
        }

        im = cvmat_images.at(0); // just monocular for now

        if(im.empty())
        {
            cerr << "Failed to load image." << endl;
        }


        //if(prev_frame_time.toSec() > 0.0){

            std::vector<measurement> imu_meas=
                    imu_buffer.GetRange(prev_frame_time.toSec(),
                                        image_timestamp.toSec());

            std::vector<measurement>::iterator it = imu_meas.begin();
            if (!first_imu_window_)
                it++;

            double prev_imu_time = -1.0;
            for( ; it != imu_meas.end(); ++it)
            {
                if(((*it).timestamp - prev_imu_time) < 1e-4)
                    continue;

                SLAMSystem->AddImuMeasurement(compass::Time((*it).timestamp),
                                              (*it).a,
                                              (*it).w);

                prev_imu_time = (*it).timestamp;
            }
        //}

        first_imu_window_ = false;
        prev_frame_time = image_timestamp;
        // Pass the image to the SLAM system
        SLAMSystem->AddImage(image_timestamp, 0, im);
    }
}

/*-------------- END MEASUREMENT CONSUMER THREADS-----------------------*/




/*-------------- START LOAD DEVICES ------------------------------*/
bool LoadDevices(std::string cam_uri, std::string imu_uri)
{

    try {
        camera_device = hal::Camera(hal::Uri(cam_uri));
    }
    catch (hal::DeviceException& e) {
        LOG(ERROR) << "Error loading camera device: " << e.what();
        return false;
    }

    if(!LoadIMU(imu_uri))
        return false;


    // Capture an image so we have some IMU data.
    std::shared_ptr<hal::ImageArray> images = hal::ImageArray::Create();
    while (imu_buffer.elements.size() == 0) {
        camera_device.Capture(*images);
    }


    if(!use_system_time){
        imu_time_offset = compass::Duration(imu_buffer.elements.back().timestamp -
                                            images->Ref().device_time());

    }

    return true;
}



bool LoadIMU(std::string imu_uri)
{
    try {
        imu_device = hal::IMU(imu_uri);
    }
    catch (hal::DeviceException& e) {
        LOG(ERROR) << "Error loading IMU device: " << e.what();
        return false;
    }

    VLOG(3) << "registering imu callback.";
    imu_device.RegisterIMUDataCallback(&ImuCallback);

    return true;
}

void ImuCallback(const hal::ImuMsg &ref)
{

    const double timestamp = use_system_time ? ref.system_time() :
                                               ref.device_time();
    Eigen::VectorXd a, w;
    hal::ReadVector(ref.accel(), &a);
    hal::ReadVector(ref.gyro(), &w);

    measurement meas(Eigen::Vector3d(w),
                     Eigen::Vector3d(a),
                     timestamp);

    imu_buffer.AddElement(meas);
}

/*-------------- END LOAD DEVICES ------------------------------*/



/*-------------- START COMPASS STATE CALLBACKS-----------------------*/
void StateCallback(const compass::Time & t,
                   const compass::kinematics::Transformation & T_w_v,
                   const Eigen::Matrix<double, 9, 1> & speed_and_bias,
                   const Eigen::Matrix<double, 3, 1> & omega_S)
{
    std::cout << "position: (" << T_w_v.r()[0] << ", " <<
                 T_w_v.r()[1] << ")" << std::endl;

    {
        std::lock_guard<std::mutex>lck(latest_position_mutex);
        latest_position.x = T_w_v.r()[0];
        latest_position.y = T_w_v.r()[1];
    }



}

/*-------------- END COMPASS STATE CALLBACKS-----------------------*/

