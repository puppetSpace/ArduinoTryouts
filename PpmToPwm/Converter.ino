
int rcInputPin = 10;
int pwmPin = 9;
// the setup routine runs once when you press reset:
void setup()
{
    Serial.begin(9600);
    pinMode(rcInputPin, INPUT);
    pinMode(pwmPin, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop()
{
    int dur = pulseIn(rcInputPin, HIGH,25000);
    int pwm = pulseToPWM(dur);

    Serial.print("PWM:");
    Serial.print(pwm);
    Serial.println();

    analogWrite(pwmPin,pwm);
}

int pulseToPWM(int pulse) {
  
  // If we're receiving numbers, convert them to motor PWM
  if ( pulse > 1000 ) {
    pulse = map(pulse, 1000, 2000, -500, 500);
    pulse = constrain(pulse, -255, 255);
  } else {
    pulse = 0;
  }

  // Anything in deadzone should stop the motor
  if ( abs(pulse) <= deadzone ) {
    pulse = 0;
  }

  return pulse;
}