#define battery A0
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(battery,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("battery charge: ");
  Serial.print(map(analogRead(battery), 0, 1023, 0, 100));
  Serial.println("%");
}
