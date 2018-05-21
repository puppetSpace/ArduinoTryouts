

void setup(){
    Serial.begin(9600);
    pinMode(2,OUTPUT);
    pinMode(3,INPUT);
}

void loop(){
    digitalWrite(2, LOW);
    delayMicroseconds(2);

    digitalWrite(2, HIGH);
    delayMicroseconds(10);
    digitalWrite(2,LOW);

    long duration = pulseIn(3,HIGH);
    Serial.print("Duration: ");
    Serial.println(duration);

    Serial.print("Distance: ");
    Serial.println(duration * 0.0343 / 2);
}