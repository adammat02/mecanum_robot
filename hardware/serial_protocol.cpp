#include "mecanum_robot/serial_protocol.hpp"
#include <sstream>
#include <cmath>


bool SerialProtocol::set_speeds(std::vector<double> speeds)
{
  std::stringstream ss;

  ss << static_cast<char>(SPEEDS) << " "
     << static_cast<float>(speeds[0]) << " "
     << static_cast<float>(speeds[1]) << " "
     << static_cast<float>(speeds[2]) << " "
     << static_cast<float>(speeds[3]) << "\r";

  const auto response = serial_.send_msg(ss.str());
  if (!response || *response != "OK\r")
    return false;
  return true;
}

bool SerialProtocol::set_pid(double kp, double ki, double kd)
{
  std::stringstream ss;

  ss << static_cast<char>(PIDS) << " "
     << static_cast<float>(kp) << " "
     << static_cast<float>(ki) << " "
     << static_cast<float>(kd) << "\r";

  const auto response = serial_.send_msg(ss.str());
  if (!response || *response != "OK\r")
    return false;
  return true;
}

bool SerialProtocol::get_rotations(std::vector<double> &rotations)
{
  const auto response = serial_.send_msg("E\r");
  if (!response || *response == "ERR\r")
    return false;
  
  std::stringstream ss(*response);
  char tag;

  ss >> tag >> rotations[0] >> rotations[1] >> rotations[2] >> rotations[3];
  if (ss.fail() || tag != static_cast<char>(ROTATIONS))
    return false;

  return true;
}

bool SerialProtocol::reset()
{
  const auto response = serial_.send_msg("R\r");
  if (!response || *response != "OK\r")
    return false;
  return true;
}

std::string SerialProtocol::serialize_tx(command_tx tx)
{
  std::stringstream ss;
  ss << static_cast<char>(FULL_FRAME_TX) << " "
     << static_cast<float>(tx.speeds[0]) << " "
     << static_cast<float>(tx.speeds[1]) << " "
     << static_cast<float>(tx.speeds[2]) << " "
     << static_cast<float>(tx.speeds[3]) << "\r";
  return ss.str();
}

std::optional<command_rx> SerialProtocol::deserialize_rx(const std::string &string_rx)
{
  command_rx rx;
  std::stringstream ss(string_rx);
  char tag;
  ss >> tag
     >> rx.rotations[0]
     >> rx.rotations[1]
     >> rx.rotations[2]
     >> rx.rotations[3]
     >> rx.battery_voltage;
  if (ss.fail() || tag != static_cast<char>(FULL_FRAME_RX))
    return std::nullopt;
  return rx;
}


std::optional<command_rx> SerialProtocol::full_transaction(command_tx tx)
{
  std::string msg = serialize_tx(tx);
  const auto response = serial_.send_msg(msg);
  if (!response || *response == "ERR\r")
    return std::nullopt;

  return deserialize_rx(*response);
}