//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
//To create a state machine you need a set of states, inputs and outputs.
//
//States
//
//An embedded system should at any given time be in a defined state, whether it is “moving left”, “door open”, “stopped”, “init”, “error” or any other imaginable state. The states and the transitions between them make up the main structure of the state machine.
//
//Inputs
//
//Inputs are what makes the system switch states and can for instance be switches, buttons and sensors or any other typical embedded input.
//
//Outputs
//
//Outputs in a state machine can be motor movement, lights or any other typical embedded output.
//
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
// exemplu program

enum State_enum {STOP, FORWARD, ROTATE_RIGHT, ROTATE_LEFT};
enum Sensors_enum {NONE, SENSOR_RIGHT, SENSOR_LEFT, BOTH};

void state_machine_run(uint8_t sensors);
void motors_stop();
void motors_forward();
void motors_right();
void motors_left();
uint8_t read_IR();

uint8_t state = STOP;

void setup(){
}

void loop(){
  state_machine_run(read_IR());

  delay(10);
}

void state_machine_run(uint8_t sensors) 
{
  switch(state)
  {
	case STOP:
	  if(sensors == NONE){
		motors_forward();
		state = FORWARD;
	  }
	  else if(sensors == SENSOR_RIGHT){
		motors_left();
		state = ROTATE_LEFT;
	  }
	  else{
		motors_right();
		state = ROTATE_RIGHT;
	  }
	  break;
	  
	case FORWARD:
	  if(sensors != NONE){
		motors_stop();
		state = STOP;
	  }
	  break;

	case ROTATE_RIGHT:
	  if(sensors == NONE || sensors == SENSOR_RIGHT){
		motors_stop();
		state = STOP;
	  }
	  break;

	case ROTATE_LEFT:
	  if(sensors != SENSOR_RIGHT)
	  {
		motors_stop();
		state = STOP; 
	  }
	  break;
  }
}

void motors_stop()
{
  //code for stopping motors
}

void motors_forward()
{
  //code for driving forward  
}

void motors_right()
{
  //code for turning right
}

void motors_left()
{
  //code for turning left
}

uint8_t read_IR()
{
  //code for reading both sensors
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//exemplu 2
//
//Define the states of the machine
#define LED_OFF 0
#define LED_ON 1



//Creating the (global) state variable:

uint8_t fsm_state = LED_OFF;



he state machine is built in the loop():

void loop(){
  //state machine
  switch (fsm_state)
  {
    case LED_OFF:  //Statements to execute every time LED_OFF is reached
      digitalWrite(13, LOW);
      fsm_state = LED_ON;
      break;
    case LED_ON:   //Statements to execute every time LED_ON is reached
      digitalWrite(13, HIGH);
      fsm_state = LED_OFF;
      break;
    default:
      break;
  }

  delay(1000);              // wait for a second
}

//---------------------------------------------------------------------------------------