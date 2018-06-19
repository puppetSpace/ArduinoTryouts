#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#define DEPTH_ECHO_OUTPUT_PORT 2
#define DEPTH_ECHO_INPUT_PORT 3
#define GPS_SERIAL_TX_PORT 7
#define GPS_SERIAL_RX_PORT 8

//after testing this will be used to send to raspberry
SoftwareSerial serialGps(GPS_SERIAL_RX_PORT, GPS_SERIAL_TX_PORT);

Adafruit_GPS GPS(&serialGps);
//Adafruit_GPS GPS(&Serial);

void setup()
{
  Serial.begin(9600);
  GPS.begin(9600);
  delay(5000);

  SetupGPS();
  SetupEcho();
}

void loop()
{
  if (serialGps.available() > 0)
  {
    char c = GPS.read();
    if (GPS.newNMEAreceived())
    {
      auto gpsString = GPS.lastNMEA();
      Serial.println(gpsString); // this also sets the newNMEAreceived() flag to false
      auto depth = GetDepth();
      //send to raspberry gpsString
    }
  }
}

void SetupGPS()
{
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
}

void SetupEcho()
{
  pinMode(DEPTH_ECHO_OUTPUT_PORT, OUTPUT);
  pinMode(DEPTH_ECHO_INPUT_PORT, INPUT);
}

double GetDepth()
{
  digitalWrite(DEPTH_ECHO_OUTPUT_PORT, LOW);
  delayMicroseconds(2);

  digitalWrite(DEPTH_ECHO_OUTPUT_PORT, HIGH);
  delayMicroseconds(10);
  digitalWrite(DEPTH_ECHO_OUTPUT_PORT, LOW);

  long duration = pulseIn(DEPTH_ECHO_INPUT_PORT, HIGH);

  //in cm's (duration in ms * speed of sound in cm/ms) for ping and receive. divide by 2 to only get distance to object
  return (duration * 0.0343 / 2);
}
