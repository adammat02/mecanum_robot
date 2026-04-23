#include "mecanum_robot/serial_comm.hpp"
#include <sstream>
#include <cmath>

LibSerial::BaudRate SerialComm::convert_baud(int baud)
{
  switch (baud)
  {
  case 1200:
    return LibSerial::BaudRate::BAUD_1200;
  case 1800:
    return LibSerial::BaudRate::BAUD_1800;
  case 2400:
    return LibSerial::BaudRate::BAUD_2400;
  case 4800:
    return LibSerial::BaudRate::BAUD_4800;
  case 9600:
    return LibSerial::BaudRate::BAUD_9600;
  case 19200:
    return LibSerial::BaudRate::BAUD_19200;
  case 38400:
    return LibSerial::BaudRate::BAUD_38400;
  case 57600:
    return LibSerial::BaudRate::BAUD_57600;
  case 115200:
    return LibSerial::BaudRate::BAUD_115200;
  case 230400:
    return LibSerial::BaudRate::BAUD_230400;
  default:
    std::cout << "Error! Baud rate " << baud << " not supported! Default to 57600" << std::endl;
    return LibSerial::BaudRate::BAUD_57600;
  }
}

void SerialComm::connect(const std::string &serial_device, int baud_rate, int timeout_ms)
{
  timeout_ms_ = timeout_ms;
  serial_.Open(serial_device);
  serial_.SetBaudRate(convert_baud(baud_rate));
}

void SerialComm::disconnect()
{
  serial_.Close();
}

bool SerialComm::is_connected() const
{
  return serial_.IsOpen();
}

std::string SerialComm::send_msg(const std::string &msg_to_send)
{
  serial_.FlushIOBuffers();
  serial_.Write(msg_to_send);

  std::string response = "";

  try
  {
    serial_.ReadLine(response, '\r', timeout_ms_);
  }
  catch (const LibSerial::ReadTimeout &e)
  {
    std::cerr << e.what() << '\n';
  }

  return response;
}

void SerialComm::set_speeds(double val_1, double val_2, double val_3, double val_4)
{
  std::stringstream ss;
  ss << "S "
     << static_cast<int>(std::lround(val_1)) << " "
     << static_cast<int>(std::lround(val_2)) << " "
     << static_cast<int>(std::lround(val_3)) << " "
     << static_cast<int>(std::lround(val_4)) << "\r";
  send_msg(ss.str());
}

void SerialComm::set_pid(double kp, double ki, double kd)
{
  std::stringstream ss;
  ss << "P " << kp << " " << ki << " " << kd << "\r";
  send_msg(ss.str());
}

void SerialComm::get_rotations(double &val_1, double &val_2, double &val_3, double &val_4)
{
  std::string response = send_msg("E\r");

  std::stringstream ss(response);
  char tag;

  ss >> tag >> val_1 >> val_2 >> val_3 >> val_4;

  if (ss.fail() || tag != 'E')
  {
    std::cerr << "Invalid response: " << response << std::endl;
    val_1 = val_2 = val_3 = val_4 = 0.0;
  }
}