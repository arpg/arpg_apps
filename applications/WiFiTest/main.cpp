#include <HAL/Signal.pb.h>
#include <HAL/Signal/SignalDevice.h>

void SignalCallback(hal::SignalMsg& _msg) {
  std::cout << "   WiFi scan result, " << _msg.system_time() << std::endl;
  for(int ii=0; ii<_msg.scan_result_size(); ii++) {
    const hal::SignalScanResult& scan_result = _msg.scan_result(ii);
    std::cout << scan_result.bssid() << " " << scan_result.rssi() << std::endl;
  }
}

int main(int argc, char** argv) {
  // connect to a WiFi scan
  hal::Signal signal("wifi:[scanperiod=3]//en0");
  signal.RegisterSignalDataCallback(&SignalCallback);

  while(1) {
  }
  return 0;
}
