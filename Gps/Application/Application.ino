//#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial serialPort(8,7);
//Adafruit_GPS GPS(&serialPort);

void setup(){
  Serial.begin(9600);
  delay(5000);

  //GPS.begin(9600);
  serialPort.begin(9600);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  //GPS.sendCommand(PGCMD_ANTENNA);

    //delay(1000);
  // Ask for firmware version
//  serialPort.println(PMTK_Q_RELEASE);
}

//uint32_t timer = millis();

void loop(){
  if(serialPort.available()>0){
    char c = serialPort.read();
    Serial.write(c); 
  }
}
