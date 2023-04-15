const int Trig = 4;
const int Echo = 7;
int time;
String formatted_data;
float distance;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(1);
  pinMode(Trig,OUTPUT);
  pinMode(Echo,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
//  digitalWrite(ledPin, HIGH);
//  delay(1000);
//  digitalWrite(ledPin, LOW);
//  delay(1000);
//  Serial.print("yo");
  while(Serial.available()>0){
    int msg = Serial.read();
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig,LOW);
    time = pulseIn(Echo,HIGH);
    distance = ((float) time / 2) / 29.1;
    formatted_data = String(distance) + "," + String(distance);
    Serial.print(formatted_data);
  }
}
