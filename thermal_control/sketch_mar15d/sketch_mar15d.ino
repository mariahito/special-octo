#include <SPI.h>
#include <SD.h>
#include <eventTimer.h>

eventTimer t;
const int chipSelect = 4;
int relay = 7;
boolean heatOn = false;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  pinMode(relay, OUTPUT);

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}

void loop() {
  // make a string for assembling the data to log:
  String dataString = "";
  int mill = millis();
  int sensor1 = analogRead(A0);
  float volt1 = (sensor1*5.0)/1024.0;
  float temp1 = (volt1*1.52-1.375)/0.0225;
  long timer = 30*60*1000UL;
  int sensor2 = analogRead(A1);
  float volt2 = (sensor2*5.0)/1024.0;
  float temp2 = (volt2*1.52-1.375)/0.0225;
 
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  //Toggles the heater on or off
  if(t.checkExpired() == true){
    if(heatOn){
      digitalWrite(relay,LOW);
        t.Cancel();
    }
  }
  else{
    heatOn = false;
    digitalWrite(relay,HIGH);
    t.Start(timer);
  }
  
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.println(mill);
    dataFile.println(sensor1);
    dataFile.println(volt1);
    dataFile.println(temp1);
    dataFile.println(dataString);
    dataFile.println(sensor2);
    dataFile.print(volt2);
    dataFile.println(temp2);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
    Serial.println("sensor1");
    Serial.print(mill);
    Serial.print(',');
    Serial.print(sensor1);
    Serial.print(',');
    Serial.print(volt1);
    Serial.print(',');
    Serial.print(temp1);
    Serial.println(dataString);
    Serial.println("Sensor2");
    Serial.print(sensor2);
    Serial.print(',');
    Serial.print(volt1);
    Serial.print(',');
    Serial.print(temp2);
    
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
  
  delay(1000);   
}
