int u = 7;
int d = 6;
int f = 5;
int b = 4;
int l = 3;
int r = 2;
void setup() {
  pinMode(u, INPUT);
  pinMode(d, INPUT);
  pinMode(f, INPUT);
  pinMode(b, INPUT);
  pinMode(l, INPUT);
  pinMode(r, INPUT);
  Serial.begin(9600);
}
void loop() {
  if (digitalRead(u) == HIGH) {
    Serial.write('u');
  }
  if (digitalRead(d) == HIGH) {
    Serial.write('d');
  }
  if (digitalRead(f) == HIGH) {
      Serial.write('f');
  }
  if (digitalRead(b) == HIGH) {
      Serial.write('b');
  }
  if (digitalRead(l) == HIGH) {
      Serial.write('l');
  }
  if (digitalRead(r) == HIGH) {
      Serial.write('r');
  }
}

