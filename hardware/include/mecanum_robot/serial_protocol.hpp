#ifndef SERIAL_PROTOCOL_HH
#define SERIAL_PROTOCOL_HH

#include "mecanum_robot/serial_comm.hpp"
#include <vector>

struct command_tx
{
  std::vector<double> speeds;

  command_tx(): speeds(4) {}
};

struct command_rx
{
  std::vector<double> rotations;
  double battery_voltage;
  // there will be another data in the future
  command_rx(): rotations(4) {}
};


class SerialProtocol
{
  enum cmdType
  {
    PIDS = 'P',
    SPEEDS = 'S',
    ROTATIONS = 'E',
    RESET = 'R',
    FULL_FRAME_RX = 'A',
    FULL_FRAME_TX = 'F',
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