<<<<<<< HEAD
// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

//This code is intended for use with Arduino Leonardo and other ATmega32U4-based Arduinos

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 8
//   Connect the GPS RX (receive) pin to Digital 7
// If using hardware serial:
//   Connect the GPS TX (transmit) pin to Arduino RX1 (Digital 0)
//   Connect the GPS RX (receive) pin to matching TX1 (Digital 1)

// If using software serial, keep these lines enabled
// (you can change the pin numbers to match your wiring):
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

// If using hardware serial, comment
// out the above two lines and enable these two lines instead:
//Adafruit_GPS GPS(&Serial1);
//HardwareSerial mySerial = Serial1;

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO true

void setup()
{
  Serial.begin(115200);
=======
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
>>>>>>> 9d068b8e2f8d1883a89c1778ec0bd9752812c41e
  delay(5000);
  GPS.begin(9600);

<<<<<<< HEAD
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
}

uint32_t timer = millis();
void loop() // run over and over again
{
  char c = GPS.read();
  if (GPS.newNMEAreceived())
  {
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return;                       // we can fail to parse a sentence in which case we should just wait for another
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
    }
    // Serial.print("\nTime: ");
    // Serial.print(GPS.hour, DEC); Serial.print(':');
    // Serial.print(GPS.minute, DEC); Serial.print(':');
    // Serial.print(GPS.seconds, DEC); Serial.print('.');
    // Serial.println(GPS.milliseconds);
    // Serial.print("Date: ");
    // Serial.print(GPS.day, DEC); Serial.print('/');
    // Serial.print(GPS.month, DEC); Serial.print("/20");
    // Serial.println(GPS.year, DEC);
    // Serial.print("Fix: "); Serial.print((int)GPS.fix);
    // Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    // if (GPS.fix) {
    //   Serial.print("Location: ");
    //   Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    //   Serial.print(", ");
    //   Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

    //   Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    //   Serial.print("Angle: "); Serial.println(GPS.angle);
    //   Serial.print("Altitude: "); Serial.println(GPS.altitude);
    //   Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    // }
  }
=======
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
>>>>>>> 9d068b8e2f8d1883a89c1778ec0bd9752812c41e
}
