#include "mystructs.h"

int LF_UP = 0;
int LF_DOWN =  0; 
unsigned long time_lf_up;
#define CURRENT_AVERAGING_STEPS 8
#define PRINT_CURRENT_MEASUREMENTS 0
#define START_UP_SPIKE_MS 500
//output relay mapping
#define OUT_FR_A 8
#define OUT_FR_B 6
#define OUT_FL_A 7
#define OUT_FL_B 9
#define OUT_RL_A A4 //A5
#define OUT_RL_B A5 //A4
#define OUT_RR_A MOSI //MISO
#define OUT_RR_B MISO //MOSI

#define IN_FL_UP 2    // input: white wire
#define IN_FL_DWN 3   // input: blue wire
#define IN_FR_UP 4
#define IN_FR_DWN 5
#define IN_RL_UP 10
#define IN_RL_DWN 11
#define IN_RR_UP 12
#define IN_RR_DWN 13

#define CURRENT_LIMIT 8
#define MAX_RUNNING_TIME   12000
#define CURRENT_RANGE 20
#define MV_TO_AMP (CURRENT_RANGE  )/ 2500

RelayStateCmds cmd_up = {1, 0}, cmd_down = { 0,1} , cmd_stop = {0,0};
RelayPairs fl_relays = {OUT_FL_A, OUT_FL_B}, fr_relays = {OUT_FR_A, OUT_FR_B}, rl_relays = {OUT_RL_A, OUT_RL_B}, rr_relays = {OUT_RR_A, OUT_RR_B};
char *channel_name[]= {"FLup", "FLdw", "FRup", "FRdw", "RLup", "RLdn", "RRup", "RRdn"};




Command cmd_fl_up = {fl_relays, cmd_up} ,  cmd_fl_down = {fl_relays, cmd_down} ,  cmd_fl_stop = {fl_relays, cmd_stop};
Command cmd_fr_up = {fr_relays, cmd_up} ,  cmd_fr_down = {fr_relays, cmd_down} ,  cmd_fr_stop = {fr_relays, cmd_stop};
Command cmd_rl_up = {rl_relays, cmd_up} ,  cmd_rl_down = {rl_relays, cmd_down} ,  cmd_rl_stop = {rl_relays, cmd_stop};
Command cmd_rr_up = {rr_relays, cmd_up} ,  cmd_rr_down = {rr_relays, cmd_down} ,  cmd_rr_stop = {rr_relays, cmd_stop};

MotorCommand motor_fl_command, motor_fr_command, motor_rl_command, motor_rr_command;


Channel channels[8];
int input[8], input_old[8];
float motor_current[4];
bool global_command_detected;
unsigned long  time_global;


float raw_current[4][CURRENT_AVERAGING_STEPS];
int current_steps;


void setup() {
  // initialize input pins: window buttons
  pinMode(IN_FL_UP, INPUT);
  pinMode(IN_FL_DWN, INPUT);
  pinMode(IN_FR_UP, INPUT);
  pinMode(IN_FR_DWN, INPUT);
  pinMode(IN_RL_UP, INPUT);
  pinMode(IN_RL_DWN, INPUT);
  pinMode(IN_RR_UP, INPUT);
  pinMode(IN_RR_DWN, INPUT);

  
  // initialize outpit pins: relays
  pinMode(OUT_FL_A, OUTPUT);
  pinMode(OUT_FL_B, OUTPUT);
  pinMode(OUT_FR_A, OUTPUT);
  pinMode(OUT_FR_B, OUTPUT);
  pinMode(OUT_RL_A, OUTPUT);
  pinMode(OUT_RL_B, OUTPUT);
  pinMode(OUT_RR_A, OUTPUT);
  pinMode(OUT_RR_B, OUTPUT);
    
  
  Serial.begin(9600);
  
  channels[FLup].motor_direction = UP;
  channels[FLdown].motor_direction = DOWN;
  channels[FRup].motor_direction = UP;
  channels[FRdown].motor_direction = DOWN;
  channels[RLup].motor_direction = UP;
  channels[RLdown].motor_direction = DOWN;
  channels[RRup].motor_direction = UP;
  channels[RRdown].motor_direction = DOWN;
  global_command_detected = false;      
  time_global = 0;
  
 
}

int flupold_mode_old = NONE;
int FLdownold_mode_old = NONE;
int i,j;
float average;


void loop() {

  // read the pushbutton input pin:
 ReadInputChannel( FLup, IN_FL_UP);
 ReadInputChannel( FLdown, IN_FL_DWN);
 ReadInputChannel( FRup, IN_FR_UP);
 ReadInputChannel( FRdown, IN_FR_DWN);
 ReadInputChannel( RLup, IN_RL_UP);
 ReadInputChannel( RLdown, IN_RL_DWN);
 ReadInputChannel( RRup, IN_RR_UP);
 ReadInputChannel( RRdown, IN_RR_DWN);


 
 
 ManageConflict (channels[FLup], channels[FLdown], true);
 ManageConflict (channels[FRup], channels[FRdown], false);
 ManageConflict (channels[RLup], channels[RLdown], false);
 ManageConflict (channels[RRup], channels[RRdown], false);
  
PrintButtonActions();

// read the analog in values:
for (i = FL; i <= RR; i++)
  raw_current[i] [current_steps] = ADCValueToCurrent(analogRead(i));

current_steps++;

if (current_steps == CURRENT_AVERAGING_STEPS) 
  current_steps = 0;   

//compute averages
for (i = FL; i <= RR; i++)
{
  average = 0;
  for (j = 0; j < CURRENT_AVERAGING_STEPS; j++)
  {
    average = average + raw_current [i][j];
  }
  motor_current[i] = average / CURRENT_AVERAGING_STEPS;
}

if (PRINT_CURRENT_MEASUREMENTS)
{
  Serial.print(     "FL: ");
  Serial.print(motor_current[FL]);
  Serial.print(     " FR: ");
  Serial.print(motor_current[FR]);
  Serial.print(     " RL: ");
  Serial.print( motor_current[RL]);
  Serial.print(     " RR: ");
  Serial.println(     motor_current[RR]);
}

//Identify motor directions
IdentifyMotorDirections (FLup, FLdown, motor_fl_command);
IdentifyMotorDirections (FRup, FRdown, motor_fr_command);
IdentifyMotorDirections (RLup, RLdown, motor_rl_command);
IdentifyMotorDirections (RRup, RRdown, motor_rr_command);


ManageAndControlMotor (FL, motor_fl_command, cmd_fl_stop);
ManageAndControlMotor (FR, motor_fr_command, cmd_fr_stop);
ManageAndControlMotor (RL, motor_rl_command, cmd_rl_stop);
ManageAndControlMotor (RR, motor_rr_command, cmd_rr_stop);
    
}


void ReadInputChannel (int _channel, int _inputpin)
{
  
    input[_channel] = digitalRead(_inputpin);
  
  if (input[_channel] != input_old[_channel])
  {
    if (input[_channel])
    {
      Serial.print ("Channel  is pressed: ");
      Serial.println (_channel);
         if (channels[_channel].motor_mode == AUTO)
        {
          //If already in auto : stop it!
          channels[_channel].motion_start_time = 0;
          channels[_channel].motor_mode = NONE;        
        }
        else
        {
         //Proper manual command  
         channels[_channel].motion_start_time = millis();
         channels[_channel].motor_mode = MANUAL;
         //Serial.println(_channel);
         //Serial.println(channels[_channel].motion_start_time);
         
        }              
    }
    else
    {
      if( (millis() - channels[_channel].motion_start_time <= 200) &&
           (millis() - channels[_channel].motion_start_time >= 60))
      {
       
          channels[_channel].motion_start_time = 0;
          channels[_channel].motor_mode = AUTO;
          channels[_channel].motion_start_time = millis();       
          Serial.print ("Channel  is auto: ");
          Serial.println (_channel);
      }
      else if (channels[_channel].motor_mode == AUTO)
      { // keep it as is: needed for allup all down commands
        channels[_channel].motor_mode = AUTO; 
      }
      else
      {
        channels[_channel].motion_start_time = 0;
        channels[_channel].motor_mode = NONE;
      }
    }
    
  }

  input_old[_channel] = input[_channel];
 return;
}

void ManageConflict (Channel &pressed_first, Channel & pressed_after, bool _verify_global_command)
{
 
  if ( ((pressed_first.motor_mode != NONE) && (pressed_after.motor_mode == NONE)) ||
      ((pressed_first.motor_mode == NONE) && (pressed_after.motor_mode != NONE)) ||
      ((pressed_first.motor_mode == NONE) && (pressed_after.motor_mode == NONE)))
  // No conflict
  return;
    
  Channel temp;
  int has_inverted = 0;
  if (pressed_first.motion_start_time != 0 && pressed_after.motion_start_time != 0 )
  {
    if (pressed_first.motion_start_time >= pressed_after.motion_start_time)
    //Inverted order: correct named variables!
    {
      temp = pressed_first;
      pressed_first = pressed_after;
      pressed_after = temp;
      has_inverted=1;
    } 
  }
  //manage conflict
if (pressed_first.motor_mode == AUTO)

 {   
   pressed_first.motor_mode = NONE; 
   pressed_first.motor_mode = NONE;
   pressed_first.motion_start_time = 0; 
   pressed_first.motion_start_time = 0;
   
   pressed_after.motor_mode = NONE; 
   pressed_after.motor_mode = NONE;
   pressed_after.motion_start_time = 0; 
   pressed_after.motion_start_time = 0;


   Serial.println ("***** autoconflict : STOP");
   return;
 }                

 
 
  if ((_verify_global_command))
  {
     if (!has_inverted)
         AllUp();
         else
         AllDown();
        
      global_command_detected = true;
      time_global = millis();
      return;
         
   } 
}


void AllUp()
{
  channels[FLup].motor_mode = AUTO;
  channels[FRup].motor_mode = AUTO;
  channels[RLup].motor_mode = AUTO;
  channels[RRup].motor_mode = AUTO;
  channels[FLdown].motor_mode = NONE;
  channels[FRdown].motor_mode = NONE;
  channels[RLdown].motor_mode = NONE;
  channels[RRdown].motor_mode = NONE;
  
  channels[FLup].motion_start_time = millis();
  channels[FRup].motion_start_time = millis();
  channels[RLup].motion_start_time = millis();
  channels[RRup].motion_start_time = millis();
  channels[FLdown].motion_start_time = 0;
  channels[FRdown].motion_start_time = 0;
  channels[RLdown].motion_start_time = 0;
  channels[RRdown].motion_start_time = 0;

  
   Serial.println("All UP");
    
}

void AllDown()
{
  channels[FLup].motor_mode = NONE;
  channels[FRup].motor_mode = NONE;
  channels[RLup].motor_mode = NONE;
  channels[RRup].motor_mode = NONE;
  channels[FLdown].motor_mode = AUTO;
  channels[FRdown].motor_mode = AUTO;
  channels[RLdown].motor_mode = AUTO;
  channels[RRdown].motor_mode = AUTO;
  
  channels[FLup].motion_start_time = 0;
  channels[FRup].motion_start_time = 0;
  channels[RLup].motion_start_time = 0;
  channels[RRup].motion_start_time = 0;
  channels[FLdown].motion_start_time = millis();
  channels[FRdown].motion_start_time = millis();
  channels[RLdown].motion_start_time = millis();
  channels[RRdown].motion_start_time = millis();

   Serial.println("All Down");
    
}


void AllStop()
{
  int i;
  for (i = FLup; i< RRdown; i++)
  {
    channels[i].motor_mode = NONE;
    channels[RRdown].motion_start_time  = 0; 
  }
 Serial.println("All STOP");   
}


void SendCommand (Command outcmd)
{
   digitalWrite(outcmd.relay_pair.relay_A, outcmd.state_cmd.state_A);
     digitalWrite(outcmd.relay_pair.relay_B, outcmd.state_cmd.state_B);
}




void IdentifyMotorDirections(int id_up, int id_down, MotorCommand &_motorcommand )
{
   // FL MOTOR
  if ((channels[id_up].motor_mode == NONE) && (channels[id_down].motor_mode == NONE))
    _motorcommand.motor_direction = STOP;
  
  else if (channels[id_up].motor_mode != NONE)
  {
      _motorcommand.motor_direction = UP;
      _motorcommand.motor_start_time = channels[id_up].motion_start_time;
  }
  else
  {
      _motorcommand.motor_direction = DOWN;
      _motorcommand.motor_start_time = channels[id_down].motion_start_time;
  }
 
}


void IdentifyMotorDirections()
{
    
  // FL MOTOR
  if ((channels[FLup].motor_mode == NONE) && (channels[FLdown].motor_mode == NONE))
    motor_fl_command.motor_direction = STOP;
  
  else if (channels[FLup].motor_mode != NONE)
  {
      motor_fl_command.motor_direction = UP;
      motor_fl_command.motor_start_time = channels[FLup].motion_start_time;
  }
  else
  {
      motor_fl_command.motor_direction = DOWN;
      motor_fl_command.motor_start_time = channels[FLdown].motion_start_time;
  }
  
  // FR MOTOR
  if ((channels[FRup].motor_mode == NONE) && (channels[FRdown].motor_mode == NONE))
    motor_fr_command.motor_direction = STOP;
  
  else if (channels[FRup].motor_mode != NONE)
  {
      motor_fr_command.motor_direction = UP;
      motor_fr_command.motor_start_time = channels[FRup].motion_start_time;
  }
  else
  {
      motor_fr_command.motor_direction = DOWN;
      motor_fr_command.motor_start_time = channels[FRdown].motion_start_time;
  }

  // RL MOTOR
  if ((channels[RLup].motor_mode == NONE) && (channels[RLdown].motor_mode == NONE))
    motor_rl_command.motor_direction = STOP;
  
  else if (channels[RLup].motor_mode != NONE)
  {
      motor_rl_command.motor_direction = UP;
      motor_rl_command.motor_start_time = channels[RLup].motion_start_time;
  }
  else
  {
      motor_rl_command.motor_direction = DOWN;
      motor_rl_command.motor_start_time = channels[RLdown].motion_start_time;
  }

  // RR MOTOR

  if ((channels[RRup].motor_mode == NONE) && (channels[RRdown].motor_mode == NONE))
    motor_rr_command.motor_direction = STOP;
  
  else if (channels[RRup].motor_mode != NONE)
  {
      motor_rr_command.motor_direction = UP;
      motor_rr_command.motor_start_time = channels[RRup].motion_start_time;
  }
  else
  {
      motor_rr_command.motor_direction = DOWN;
      motor_rr_command.motor_start_time = channels[RRdown].motion_start_time;
  }

  

}


void ControlMotor(int motor_index )
{

//  Serial.println(" motor fl command ");
  MotorCommand motorcommand;
  Command relaycommand;
  
  if (motor_index == FL)
  {
    if (motor_fl_command.motor_direction == UP)
        relaycommand = cmd_fl_up;
    else
        relaycommand = cmd_fl_down;
    
    motorcommand.motor_start_time = motor_fl_command.motor_start_time;    
    
  }
  
  if (motor_index == FR)
  {
    if (motor_fr_command.motor_direction == UP)
        relaycommand = cmd_fr_up;
    else
        relaycommand = cmd_fr_down;
    
    motorcommand.motor_start_time = motor_fr_command.motor_start_time;
  }
  
  if (motor_index == RL)
  {
    if (motor_rl_command.motor_direction == UP)
        relaycommand = cmd_rl_up;
    else
        relaycommand = cmd_rl_down;
    
    motorcommand.motor_start_time = motor_rl_command.motor_start_time;
  }
  
  if (motor_index == RR)
  {
    if (motor_rr_command.motor_direction == UP)
        relaycommand = cmd_rr_up;
    else
        relaycommand = cmd_rr_down;
    
    motorcommand.motor_start_time = motor_rr_command.motor_start_time;
  }  
  
  
  if (MotorStopOnCurrentCondition(motor_index, motorcommand.motor_start_time) || ((motorcommand.motor_start_time != 0) &&(millis() - motorcommand.motor_start_time > MAX_RUNNING_TIME) ))
  {
    Serial.println( "Bingo");
    Serial.print(millis());
     Serial.print("\t");
    Serial.println( motorcommand.motor_start_time);
    StopMotor(motor_index);
    return;
  }
  
  SendCommand(relaycommand);
  
}



void StopMotor(int motor_index)
{
  
  if (motor_index == FL)
  {
    channels[FLup].motor_mode = NONE;
    channels[FLdown].motor_mode = NONE;
    channels[FLup].motion_start_time  = 0;
    channels[FLdown].motion_start_time  = 0;
  }
  
  if (motor_index == FR)
  {
    channels[FRup].motor_mode = NONE;
    channels[FRdown].motor_mode = NONE;
    channels[FRup].motion_start_time  = 0;
    channels[FRdown].motion_start_time  = 0;

  }
  
  if (motor_index == RL)
  {
    channels[RLup].motor_mode = NONE;
    channels[RLdown].motor_mode = NONE;
    channels[RLup].motion_start_time  = 0;
    channels[RLdown].motion_start_time  = 0;

  }
  
  if (motor_index == RR)
  {
    channels[RRup].motor_mode = NONE;
    channels[RRdown].motor_mode = NONE;
    channels[RRup].motion_start_time  = 0;
    channels[RRdown].motion_start_time  = 0;

  }
}

void PrintButtonActions()
{
  
  if (flupold_mode_old != channels[FLup].motor_mode)
  {
  
  if (channels[FLup].motor_mode == AUTO)
  Serial.println("LF_UP Auto");
  
  if (channels[FLup].motor_mode == MANUAL)
  Serial.println("LF_UP Manual");
  
  if (channels[FLup].motor_mode == NONE)
  Serial.println("LF_UP Stopped");

  }
 flupold_mode_old = channels[FLup].motor_mode  ;
 ///////////

  
  if (FLdownold_mode_old != channels[FLdown].motor_mode)
  {
  
  if (channels[FLdown].motor_mode == AUTO)
  Serial.println("LF_DOWN Auto");
  
  if (channels[FLdown].motor_mode == MANUAL)
  Serial.println("LF_DOWN Manual");
  
  if (channels[FLdown].motor_mode == NONE)
  Serial.println("LF_DOWN Stopped");

  }

 FLdownold_mode_old = channels[FLdown].motor_mode  ;
 
 
}


void ManageAndControlMotor (int _index, MotorCommand _motor_command, Command _stop_cmd)
{
  if (_motor_command.motor_direction == STOP)
  {
    SendCommand(_stop_cmd);
  }
    else
    {
    ControlMotor(_index);
    }

}

int MotorStopOnCurrentCondition( int motor_index, unsigned long int starttime)
{
  
  if (( millis() - starttime ) > START_UP_SPIKE_MS && (motor_current[motor_index] >= CURRENT_LIMIT))
  { 
    Serial.print("Stop on current: ");
    Serial.println(motor_current[motor_index]);
      return 1;
  }
      else return 0;
}

float ADCValueToCurrent (long int adc_in)
{
  float mv = (adc_in / 1023.0) * 5000;

  return (mv - 2500) * MV_TO_AMP;

}

void TestOutputRelaysInSequence()
{  
  while (1)
  {
    digitalWrite(OUT_FL_A, 1);
    delay(1000);
    digitalWrite(OUT_FL_A, 0);
    delay(1000);
        
    digitalWrite(OUT_FL_B, 1);
    delay(1000);
    digitalWrite(OUT_FL_B, 0);
    delay(1000);
        
    digitalWrite(OUT_FR_A, 1);
    delay(1000);
    digitalWrite(OUT_FR_A, 0);
    delay(1000);

    digitalWrite(OUT_FR_B, 1);
    delay(1000);
    digitalWrite(OUT_FR_B, 0);
    delay(1000);


    digitalWrite(OUT_RL_A, 1);
    delay(1000);
    digitalWrite(OUT_RL_A, 0);
    delay(1000);

    digitalWrite(OUT_RL_B, 1);
    delay(1000);
    digitalWrite(OUT_RL_B, 0);
    delay(1000);

    digitalWrite(OUT_RR_A, 1);
    delay(1000);
    digitalWrite(OUT_RR_A, 0);
    delay(1000);
        
    digitalWrite(OUT_RR_B, 1);
    delay(1000);
    digitalWrite(OUT_RR_B, 0);
    delay(1000);
  }
}

