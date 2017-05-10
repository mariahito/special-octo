#include <eventTimer.h>

int16_t targetSpeed = 0;

eventTimer pidTimer;

#define ENCA 2
#define ENCB 3

#define PWM 5

volatile int16_t encCount = 0;
int16_t prevCount = 0;

//PID coefficients: you'll want to change these
float Kp = 1;
float Ki = 0;
float Kd = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println("Hej.");

  //attach the encoder interrupts
  attachInterrupt(0, HandleEncoderA, CHANGE);
  attachInterrupt(1, HandleEncoderB, CHANGE);

  //set the pwm pin to output
  pinMode(PWM, OUTPUT);

  pidTimer.Start(25);
}

void loop(void)
{
  if(pidTimer.checkExpired()) HandlePIDControl();
}

void HandlePIDControl(void)
{  
  pidTimer.Start(25);

  static int16_t sumError = 0;
  static int16_t prevError = 0;

  cli(); //turn off interrupts while we grab encCount
  int16_t currCount = encCount; //use a temporary value
  sei(); //turn interrupts back on

  int16_t targetSpeed = map(analogRead(A0), 0, 1023, 0, 100); //reads a potentiometer and maps it to 0 - 100
  int16_t actualSpeed = currCount - prevCount;
  prevCount = currCount;
  float pwmValue = 0.0;

    //put your PID control here!!!!






  Serial.print(actualSpeed);
  Serial.print('\t');
  Serial.print(targetSpeed);
  Serial.print('\t');
  Serial.print(pwmValue);
  Serial.print('\n');
}

void HandleEncoderA(void)
{
    //put your encoder handling logic here
    if(digitalRead(ENCA) == digitalRead(ENCB)){
      encCount++; 
    }
    else{
      encCount--;
    }
}

void HandleEncoderB(void)
{
    //put your encoder handling logic here
     if(digitalRead(ENCA) == digitalRead(ENCB)){
      encCount--; 
    }
    else{
      encCount++;
    }
}

