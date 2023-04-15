#include <Wire.h>
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
 
}

void loop() {
  // put your main code here, to run repeatedly:

  while(Serial.available()>0){
    
 
    int msg = Serial.read();
    int sensorValue = analogRead(A0);
    float voltage = sensorValue * 5.0 / 1023.0;
    Serial.println(voltage);
  }
}
