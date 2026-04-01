const int L_IN1 = 4, L_IN2 = 5, L_ENA = 6;
const int STBY = 10;

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
  digitalWrite(STBY, HIGH);  // 使能 TB6612
  Serial.begin(115200);
  Serial.println("Motor test start");
}

void loop() {
  Serial.println("Forward 150");
  setMotor(L_IN1, L_IN2, L_ENA, 150);
  delay(2000);

  Serial.println("Stop");
  setMotor(L_IN1, L_IN2, L_ENA, 0);
  delay(1000);

  Serial.println("Reverse 150");
  setMotor(L_IN1, L_IN2, L_ENA, -150);
  delay(2000);

  Serial.println("Stop");
  setMotor(L_IN1, L_IN2, L_ENA, 0);
  delay(1000);
}
