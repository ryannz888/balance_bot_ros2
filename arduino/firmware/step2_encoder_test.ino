const int L_IN1 = 4, L_IN2 = 5, L_ENA = 6;
const int STBY = 10;
const int ENC_A = 2, ENC_B = 3;

volatile long enc_count = 0;

void encISR() {
  enc_count += (digitalRead(ENC_B) == HIGH) ? 1 : -1;
}

void setMotor(int in1, int in2, int en, int pwm) {
  if (pwm > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
    pwm = -pwm;
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  }
  analogWrite(en, constrain(pwm, 0, 255));
}

void setup() {
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT); pinMode(L_ENA, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encISR, RISING);
  Serial.begin(115200);
}

void loop() {
  setMotor(L_IN1, L_IN2, L_ENA, 150);
  Serial.print("enc: ");
  Serial.println(enc_count);
  delay(100);
}
