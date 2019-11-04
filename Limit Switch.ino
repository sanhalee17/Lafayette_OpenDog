int limit_switch = 3;
bool limit_switch1 = false;
void setup() {
  // put your setup code here, to run once:
pinMode(limit_switch,INPUT_PULLUP);
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
if(digitalRead(limit_switch) == LOW){
Serial.print("yes");
}
else {
 Serial.print("no");

}
 Serial.println();
}
