#include "MyTimer.h"

//defines various state variables. Should be self-explanatory from the variable names
int IRstate = 0; //state of the IR signal from the lead robot
int leftRSstate = 0; //left reflectance sensor state
int rightRSstate = 0; //right reflectance sensor state
double currentSpeed; //variable for the current speed
double distance;  //distance from lead robot
double error;     //error for PID control
double errorSum = 0; //sum of all the error for PID control
double lastError = 0; //previous error
double controlEffort = 0; //the control effort that comes as a result of PID control

//Define some constants in hopes that it makes the code more understandable and easier to follow
#define ENCA 2 //pin for encoder A
#define ENCB 3 //pin for encoder B
#define TRIG 4 //trigger pin for the ultrasonice sensor
#define leftMotor 5
#define rightMotor 6
#define rightRS 7 //right reflectance sensor
#define leftRS A1 //left reflectance sensor

#define leftMotorSwitch1 10 //these next 4 are the pins used to switch the directions of the wheels
#define leftMotorSwitch2 11
#define rightMotorSwitch1 12
#define rightMotorSwitch2 13

#define ECHO A0 //echo pin for the ultrasonice sensor

//starts the left and right motor speeds at a hard coded value
int LMS = 50;
int RMS = 50;

//defines target distance
double targetDistance = 0.50;

int LEFT;
int RIGHT;

//Initializes the proportional, integral, and derivative coefficients for PID control
double Kp = 20;
double Ki = 0.5;

MyTimer t;
MyTimer speedTimer;
int blackWhiteSetPoint; //set point for what is considered black/white. Set during calibration


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //sets various pins to their respective states
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(leftMotor, OUTPUT);
  pinMode(rightMotor, OUTPUT);
  pinMode(rightRS, OUTPUT);
  pinMode(leftRS, OUTPUT);

  speedTimer.startTimer(25);

  // The follow section of code starts a timer for the calibration period, which lasts for 10 seconds. It calls the calibrate function repeatedly for these 10 seconds
  t.startTimer(7000);
  while (!t.checkExpired())
  {
    blackWhiteSetPoint = calibrate(leftRS);
  }

  /* The rest of the code in this setup() function comes straight from the IR lab and the code given to us
      by Professor Lewin. I have written none of this, simply copied and pasted
  */

  //disable interrupts
  cli();

  //set timer1 to Normal mode, pre-scaler of 8
  //use ASSIGNMENT since the bootloader sets up timer1 for 8-bit PWM, which isn't very useful here
  TCCR1A = 0x00;
  TCCR1B = 0x02;

  //enable input capture interrupt
  TIMSK1 |= (1 << ICIE1);

  //enable noise cancelling
  TCCR1B |= (1 << ICNC1);

  //set for falling edge on ICP1 (pin 8 on an Arduino Uno)
  TCCR1B &= ~(1 << ICES1);

  //re-enable interrupts
  sei();
}

/* Variables from Lewin's IR sensor code */
volatile uint8_t dataReady = 0;
volatile uint16_t code = 0;
volatile uint8_t index = 0;

void loop() {

  //powers the motor pins in a fashion that both wheels will turn forward
  digitalWrite(leftMotorSwitch1, HIGH);
  digitalWrite(leftMotorSwitch2, LOW);
  digitalWrite(rightMotorSwitch1, LOW);
  digitalWrite(rightMotorSwitch2, HIGH);

  //calls readSensor fucntions
  int LEFT = readSensor(leftRS);
  int RIGHT = readSensor(rightRS);

  //calls the getDistance function to determine distance behind lead robot
  distance = getDistance();

  //uses the distance from the lead robot and target distance to determine the error, which is then passed in as a parameter to the PID control functions below
  error = targetDistance - distance;

  //sets the reflectance sensor state variables according to the integers returned from the readSensor functions.
  //Calls the setRSState functions
  leftRSstate = setRSState(LEFT);
  rightRSstate = setRSState(RIGHT);

  if (dataReady)
  {
    //if IR sensor reads 147, then it tells the robot to turn left at the next turn
    if (code == 147)
    {
      IRstate = 1;
    }

    //now tells the robot to turn right at the next turn
    if (code == 146)
    {
      IRstate = 2;
    }
    dataReady = 0;
  }


  //sets the motor speeds according to their respective motor speed variables
  analogWrite(leftMotor, LMS);
  analogWrite(rightMotor, RMS);

  /* For the next series of if statements, 1 = white and 0 = black

     Our general approach is that if a sensor is reading white, then that side of the robot needs to speed up, so it increases that motor's speed

     If both wheels are reading white, then it resets both wheels to a certain, identical speed.

     Within each if statement is also a call to the adjustSpeed() function. This function is responsible for PID control and can be found near the end of the code.

     After each state there is also an analogWrite statement for each wheel, which adjusts their speeds
  */


  //left sensor showing white
  if (leftRSstate == 1)
  {
    LMS += 15;
    LMS += (LMS / 5);
  }

  analogWrite(leftMotor, LMS);
  analogWrite(rightMotor, RMS);


  //right sensor showing white
  if (rightRSstate == 1)
  {
    RMS += 15;
    RMS += (RMS / 5);
  }

  analogWrite(leftMotor, LMS);
  analogWrite(rightMotor, RMS);


  //both sensors showing white
  if (leftRSstate == 1 && rightRSstate == 1)
  {
    LMS = 50;
    RMS = 50;
    //adjustSpeed(error);
  }
  analogWrite(leftMotor, LMS);
  analogWrite(rightMotor, RMS);


  //both sensors are showing black (indicating a turn) and the IR signal says to turn left
  if (leftRSstate == 0 && rightRSstate == 0 && IRstate == 1)
  {
    turnLeft();
    IRstate = 0;
  }

  //both sensors are showing black (indicating a turn) and the IR signal says to turn right
  if (leftRSstate == 0 && rightRSstate == 0 && IRstate == 2)
  {
    turnRight();
    IRstate = 0;
  }

  //If the left sensor is black, sets it to a base speed (hard coded)
  if (leftRSstate == 0) {

    LMS = 50;
    //adjustSpeed(error);
  }

  analogWrite(leftMotor, LMS);
  analogWrite(rightMotor, RMS);


  //If the right sensor is black, sets it to a base speed (hard coded)
  if (rightRSstate == 0)
  {
    RMS = 50;
    //adjustSpeed(error);
  }

  analogWrite(leftMotor, LMS);
  analogWrite(rightMotor, RMS);

  adjustSpeed(error);
  if (LMS > 110)
  {
    LMS = 110;
  }

  if (RMS > 110)
  {
    RMS = 110;
  }

  if (RMS < 3)
  {
    RMS = 3;
  }

  if (LMS < 3)
  {
    LMS = 3;
  }
  analogWrite(leftMotor, LMS);
  analogWrite(rightMotor, RMS);

  //Resets the states after the loop
  leftRSstate = 0;
  rightRSstate = 0;

}


//Function that takes in a certain PIN number
int readSensor(int value)
{
  pinMode(value, OUTPUT);
  digitalWrite(value, HIGH);
  delay(2);
  pinMode(value, INPUT);

  int x1 = micros();
  while (digitalRead(value) == 1)
  {}

  int y1 = micros();
  return y1 - x1;
}

double getDistance()
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(20);

  digitalWrite(TRIG, LOW);

  double b1 = pulseIn(ECHO, HIGH);

  b1 = (b1 / 2000000.0) * 340.29;
  return b1;

}

int calibrate(int value)
{

  int w = readSensor(value);

  static int lowest = w;
  static int highest = lowest;

  if (w > highest)
  {
    highest = w;
  }

  if (w < lowest)
  {
    lowest = w;
  }
  return (highest + lowest) / 2;
}

int setRSState(int value)
{
  int returnVal = 0;

  if ( value > blackWhiteSetPoint)
  {
    returnVal = 0;

  }

  if (value < blackWhiteSetPoint)
  {
    returnVal = 1;

  }

  return returnVal;

}

void turnLeft() {

  t.startTimer(250);
  while (!t.checkExpired())
  {}

  digitalWrite(leftMotorSwitch1, LOW);
  digitalWrite(leftMotorSwitch2, HIGH);
  digitalWrite(rightMotorSwitch1, LOW);
  digitalWrite(rightMotorSwitch2, HIGH);

  LMS = 50;
  RMS = 50;

  analogWrite(leftMotor, LMS);
  analogWrite(rightMotor, RMS);

  while (leftRSstate == 0)
  {
    LEFT = readSensor(leftRS);
    leftRSstate = setRSState(LEFT);

  }

  while (leftRSstate == 1)
  {
    LEFT = readSensor(leftRS);
    leftRSstate = setRSState(LEFT);

  }

  while (leftRSstate == 0)
  {
    LEFT = readSensor(leftRS);
    leftRSstate = setRSState(LEFT);

  }

  t.startTimer(400);
  while (!t.checkExpired())
  {}
  digitalWrite(leftMotorSwitch1, HIGH);
  digitalWrite(leftMotorSwitch2, LOW);
  digitalWrite(rightMotorSwitch1, LOW);
  digitalWrite(rightMotorSwitch2, HIGH);
  IRstate = 0;
}



void turnRight()
{
  t.startTimer(250);
  while (!t.checkExpired())
  {}

  digitalWrite(leftMotorSwitch1, HIGH);
  digitalWrite(leftMotorSwitch2, LOW);
  digitalWrite(rightMotorSwitch1, HIGH);
  digitalWrite(rightMotorSwitch2, LOW);

  LMS = 50;
  RMS = 50;

  analogWrite(leftMotor, LMS);
  analogWrite(rightMotor, RMS);

  while (rightRSstate == 0)
  {
    RIGHT = readSensor(rightRS);
    rightRSstate = setRSState(RIGHT);

  }

  while (rightRSstate == 1)
  {
    RIGHT = readSensor(rightRS);
    rightRSstate = setRSState(RIGHT);

  }

  while (rightRSstate == 0)
  {
    RIGHT = readSensor(rightRS);
    rightRSstate = setRSState(RIGHT);

  }

  t.startTimer(400);
  while (!t.checkExpired())
  {}
  digitalWrite(leftMotorSwitch1, HIGH);
  digitalWrite(leftMotorSwitch2, LOW);
  digitalWrite(rightMotorSwitch1, LOW);
  digitalWrite(rightMotorSwitch2, HIGH);
  IRstate = 0;
}

void adjustSpeed(double error)
{
  
    error = error * -1;
    controlEffort = (Kp * error) + (Ki * errorSum);

    LMS += controlEffort;
    RMS += controlEffort;

    if (RMS > 125)
    {
      RMS = 125;
    }

    if (LMS > 125)
    {
      LMS = 125;
    }

    if (LMS < 3)
    {
      LMS = 3;
    }

    if (RMS < 3)
    {
      RMS = 3;
    }

    errorSum += error;

    if (errorSum > 25)
    {
      errorSum = 25.0;
    }

    if (errorSum < -25)
    {
      errorSum = -25.0;
    }
  
}


/* All of the code below comes straight from the IR lab and the code provided to us from Professor Lewin. I
    have written none of the code below.
*/

volatile uint16_t fallingEdge = 0;
volatile uint16_t risingEdge = 0;

//ICR1 is the input capture ("timestamp") register
//capture happens on PB0/Arduino pin 8
ISR(TIMER1_CAPT_vect)
{
  if (!(TCCR1B & (1 << ICES1))) //if we're looking for FALLING edges
  {
    fallingEdge = ICR1;
    TCCR1B |= (1 << ICES1); //now set to look for a rising edge
  }

  else //we must be looking for a RISING edge
  {
    risingEdge = ICR1;
    TCCR1B &= ~(1 << ICES1); //set to look for a falling edge

    //and process
    uint16_t delta = risingEdge - fallingEdge; //length of pulse, in timer counts
    delta /= 2; //scaled to us

    if (delta > 2250 && delta < 2750) //start pulse
    {
      index = 0;
      code = 0; //reset code
      dataReady = 0; //clobber previous read if it wasn't processed
    }

    else if (delta > 500 && delta < 800) //short pulse
    {
      index++;
    }

    else if (delta > 1100 && delta < 1500) //long pulse
    {
      code += (1 << index);
      index++;
    }

    else //error
    {
      index = 0; //start over
      code = 0;
    }

    if (index == 12) dataReady = 1;
  }
}

