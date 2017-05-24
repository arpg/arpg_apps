#include "spPID.h"

spPID::spPID() {
  gain_p_ = 0;
  gain_i_ = 0;
  gain_d_ = 0;
  integral_value = 0;
}

spPID::~spPID() {

}

void spPID::SetGainP(double p) {
  gain_p_ = p;
}

void spPID::SetGainI(double i) {
  gain_i_ = i;
}

void spPID::SetGainD(double d) {
  gain_d_ = d;
}

void spPID::SetPlantError(double error) {
  prev_error_ = curr_error_;
  curr_error_ = error;
  prev_error_time_ = curr_error_time_;
  curr_error_time_ = std::chrono::high_resolution_clock::now();
}

double spPID::TickTock_ms(spTimestamp tick_time,spTimestamp tock_time) {
  return std::chrono::duration_cast<std::chrono::milliseconds>( tock_time - tick_time ).count();
}

double spPID::GetControlOutput() {

  // calculate P control output
  double p_cntrl = gain_p_*(curr_error_);
  // calculate D control output
  double d_cntrl = gain_d_*((curr_error_-prev_error_)/(spPID::TickTock_ms(prev_error_time_,curr_error_time_)*0.001));
  // calculate I control output
  integral_value += prev_error_*(spPID::TickTock_ms(prev_error_time_,curr_error_time_)*0.001);
  double i_cntrl = gain_i_*integral_value;
  return (p_cntrl+i_cntrl+d_cntrl);
}
