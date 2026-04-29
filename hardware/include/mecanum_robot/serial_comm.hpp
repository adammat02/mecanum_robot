#ifndef SERIAL_COMM_HPP
#define SERIAL_COMM_HPP

#include <libserial/SerialPort.h>
#include <iostream>
#include <optional>

class SerialComm
{
  LibSerial::SerialPort serial_;
  int timeout_ms_;
  char terminator_;

  LibSerial::BaudRate convert_baud(int baud);

public:
  SerialComm() = default;
  void connect(const std::string &serial_device, int baud_rate, int timeout_ms, char terminator);
  void disconnect();
  bool is_connected() const;
  std::optional<std::string> read_msg();
  std::optional<std::string> send_msg(const std::string &msg_to_send);
};

#endif