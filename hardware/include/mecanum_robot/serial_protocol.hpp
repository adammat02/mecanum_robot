#ifndef SERIAL_PROTOCOL_HH
#define SERIAL_PROTOCOL_HH

#include "mecanum_robot/serial_comm.hpp"
#include <vector>

struct command_tx
{
  std::vector<double> speeds;
};

struct command_rx
{
  std::vector<double> rotations;
  // there will be another data in the future
};


class SerialProtocol
{
  enum cmdType
  {
    PIDS = 'P',
    SPEEDS = 'S',
    ROTATIONS = 'E',
    RESET = 'R',
    FULL_FRAME_RX = 'F',
    FULL_FRAME_TX = 'A',
  };
  SerialComm &serial_;

  std::string serialize_tx(command_tx tx);
  std::optional<command_rx> deserialize_rx(const std::string &string_rx);

public:
  explicit SerialProtocol(SerialComm &serial) : serial_(serial) {};
  bool reset();
  bool set_speeds(std::vector<double> speeds);
  bool set_pid(double kp, double ki, double kd);
  bool get_rotations(std::vector<double> &rotations);
  std::optional<command_rx> full_transaction(command_tx tx);
};

#endif