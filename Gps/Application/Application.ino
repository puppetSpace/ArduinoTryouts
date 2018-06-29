#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#define DEPTH_ECHO_OUTPUT_PORT 2
#define DEPTH_ECHO_INPUT_PORT 3
#define GPS_SERIAL_TX_PORT 7
#define GPS_SERIAL_RX_PORT 8

SoftwareSerial mySerial(GPS_SERIAL_RX_PORT, GPS_SERIAL_TX_PORT);
Adafruit_GPS GPS(&mySerial);

// If using hardware serial, comment
// out the above two lines and enable these two lines instead:
//Adafruit_GPS GPS(&Serial1);
//HardwareSerial mySerial = Serial1;

void setup()
{
  Serial.begin(115200);
  GPS.begin(9600);
  delay(5000);
  GPS.begin(9600);

  delay(1000);

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
      if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
        return;
    }

    if (timer > millis())
      timer = millis();

    // approximately every 2 seconds or so, print out the current stats
    if (millis() - timer > 2000)
    {
      timer = millis(); // reset the timer
      auto dateTime = String(GPS.year) + "/" + String(GPS.month) + "/" + String(GPS.day) + " " + String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds) + "." + String(GPS.milliseconds);
      Serial.print("\date:");
      Serial.println(dateTime);
      if (GPS.fix)
      {
        auto latLong = String(GPS.latitude, 4) + String(GPS.lat) + ", " + String(GPS.longitude, 4) + String(GPS.lon);
        Serial.print("Location: ");
        Serial.println(latLong);

        auto depth = GetDepth();
        //send to raspberry gpsString
      }
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
