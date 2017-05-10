// Maria Hito (mh4wt)

int16_t targetSpeed = 0;

#define ENCA 2
#define ENCB 3

#define PWM 5

volatile int16_t encCount = 0;
volatile int16_t currCount = 0;
int16_t prevCount = 0;

volatile uint8_t readyToPID = 0;
//double errSum;
//double lastErr;

float Kp = 5;
float Ki = 2;
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

  cli();
  TCCR1A = 0x00;
  TCCR1B = 0x0B; ///CTC, prescaler of 64!!

  OCR1A = 6249;  //set to make a 25 ms interval
  TIMSK1 = 0x02;
  sei();
}

void loop(void)
{
  if(readyToPID) HandlePIDControl();
}

void HandlePIDControl(void)
{  
  static int16_t sumError = 0;
  static int16_t prevError = 0;

  cli(); //turn off interrupts while we grab currCount
  int16_t actualSpeed = currCount - prevCount;
  prevCount = currCount;
  sei(); //turn interrupts back on

  int16_t targetSpeed = map(analogRead(A0), 0, 1023, 0, 100);

  //add PID stuffs here
  int16_t error = targetSpeed - actualSpeed;
  sumError += error;
  
  int16_t controlEffort = Kp*error+Ki*sumError+Kd*(error -prevError);

  //int16_t integralControl = Ki*sumError;
  
    if (controlEffort > 255){
      controlEffort = 255;
    }
      if (controlEffort < 0){
        controlEffort = 0;
    }
  
  analogWrite(PWM, controlEffort);

  Serial.print(actualSpeed);
  Serial.print('\t');
  Serial.print(targetSpeed);
  Serial.print('\t');
  Serial.print(controlEffort);
  Serial.print('\n');

  readyToPID = 0;
}

ISR(TIMER1_COMPA_vect)
{
  currCount = encCount;
  readyToPID = 1;
}

void HandleEncoderA(void)
{
  if(digitalRead(ENCA) == digitalRead(ENCB)) encCount++;
  else encCount--;
}

void HandleEncoderB(void)
{
  if(digitalRead(ENCA) != digitalRead(ENCB)) encCount++;
  else encCount--;
}

