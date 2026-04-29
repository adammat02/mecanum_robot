#include "mecanum_robot/serial_comm.hpp"

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

void SerialComm::connect(const std::string &serial_device, int baud_rate, int timeout_ms, char terminator)
{
  timeout_ms_ = timeout_ms;
  terminator_ = terminator;
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

std::optional<std::string> SerialComm::read_msg()
{
  std::string response;

  try
  {
    serial_.ReadLine(response, terminator_, timeout_ms_);
  }
  catch (const LibSerial::ReadTimeout &e)
  {
    std::cerr << e.what() << std::endl;
    return std::nullopt;
  }

  return response;
}

std::optional<std::string> SerialComm::send_msg(const std::string &msg_to_send)
{
  serial_.Write(msg_to_send);
  const auto response = read_msg();

  return response;
}