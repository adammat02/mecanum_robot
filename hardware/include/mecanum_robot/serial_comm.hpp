#ifndef SERIAL_COMM_HPP
#define SERIAL_COMM_HPP

#include <libserial/SerialPort.h>
#include <iostream>

class SerialComm
{
  LibSerial::SerialPort serial_;
  int timeout_ms_;

  LibSerial::BaudRate convert_baud(int baud);

  public:
  SerialComm() = default;
  void connect(const std::string &serial_device, int baud_rate, int timeout_ms);
  void disconnect();
  bool is_connected() const;
  std::string send_msg(const std::string &msg_to_send);
  
  void set_speeds(double val_1, double val_2, double val_3, double val_4);
  void set_pid(double kp, double ki, double kd);
  void get_rotations(double &val_1, double &val_2, double &val_3, double &val_4);

};

#endif