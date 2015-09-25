typedef struct {
  int state_A;
  int state_B;
} RelayStateCmds;

typedef struct {
  int relay_A;
  int relay_B;
} RelayPairs;

typedef struct {
  RelayPairs relay_pair;
  RelayStateCmds state_cmd;
} Command;

enum MotorIndex {FL, FR, RL, RR };
enum MotorDirection {STOP, UP, DOWN};
enum MotorMode {NONE, MANUAL, AUTO};
enum InputChannel {FLup, FLdown, FRup, FRdown, RLup, RLdown, RRup, RRdown};

typedef struct {
  MotorDirection motor_direction;
  unsigned long motor_start_time;
} MotorCommand;

typedef struct  {
  MotorIndex motor_index;
  MotorDirection motor_direction;
  MotorMode motor_mode;
  unsigned long motion_start_time;
} Channel;

