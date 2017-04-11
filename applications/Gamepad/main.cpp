#include <HAL/Gamepad.pb.h>
#include <HAL/Gamepad/GamepadDevice.h>

void GamepadCallback(hal::GamepadMsg& _msg) {
  std::cout << "Gamepad State -> ";
  for(int ii=0; ii<_msg.num_axes(); ii++) {
    std::cout << _msg.axes().data(ii) << ", ";
  }
  std::cout << "| ";
  for(int ii=0; ii<_msg.num_buttons(); ii++) {
    std::cout << _msg.buttons().data(ii) << ",";
  }            
  std::cout << std::endl;
}

int main(int argc, char** argv) {
  // connect to a gamepad
  hal::Gamepad gamepad("gamepad:/");
  gamepad.RegisterGamepadDataCallback(&GamepadCallback);

  while(1) {
  }
  return 0;
}
