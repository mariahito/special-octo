class MyTimer
{
  public:
    MyTimer();
    bool checkExpired();
    void cancel();
    void startTimer(unsigned long duration);
  private:
    bool isRunning;
    unsigned long duration;
    unsigned long startTime;
};
MyTimer::MyTimer()
{
  isRunning = false;
}
bool isRunning;
unsigned long duration;
unsigned long startTime;
bool MyTimer::checkExpired()
{
  if ((millis() - startTime >= duration) && isRunning)
  {
    isRunning = false;
    return true;
  }
  return false;
}

void MyTimer::cancel()
{
  isRunning = false;
}

void MyTimer::startTimer(unsigned long dur)
{
 duration = dur;
 isRunning = true;
 startTime = millis();
}
