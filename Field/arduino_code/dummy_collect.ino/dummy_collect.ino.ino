long num;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(1);
}

void loop() {
  while(Serial.available()>0){
    int msg = Serial.read();
    delay(50);
    num = random(0, 20);
    Serial.println(String(num));
  }
}
