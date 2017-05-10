

#ifndef EVENTTIMER_H
#define EVENTTIMER_H

using namespace std;


class eventTimer {
  public:
    eventTimer(){};
    boolean checkExpired(){
		if((millis() - startTime) >= duration && isRunning){
			isRunning = false;
			return true;
		}
else return false;
	}
    void Start(unsigned long mil){
		isRunning = true;
		startTime = millis();
		duration = mil;
	}
    void Cancel(){
		isRunning = false;
	}

  private:
    unsigned long startTime;
    unsigned long duration;
    boolean isRunning;
};
#endif