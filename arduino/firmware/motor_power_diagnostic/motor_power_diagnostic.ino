// Direct motor-driver check. Keep both wheels elevated before uploading.
const int L_IN1 = 4, L_IN2 = 5, L_ENA = 6;
const int R_IN1 = 7, R_IN2 = 8, R_ENB = 9;
const int STBY = 10;

void setMotor(int in1, int in2, int en, int pwm) {
  digitalWrite(in1, pwm >= 0 ? HIGH : LOW);
  digitalWrite(in2, pwm >= 0 ? LOW : HIGH);
  analogWrite(en, abs(pwm));
}

void stopMotors() {
  analogWrite(L_ENA, 0);
  analogWrite(R_ENB, 0);
  digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
}

void setup() {
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT); pinMode(L_ENA, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT); pinMode(R_ENB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  stopMotors();
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  Serial.println("MOTOR_PULSE");
  setMotor(L_IN1, L_IN2, L_ENA, 150);
  setMotor(R_IN1, R_IN2, R_ENB, 150);
  delay(500);
  stopMotors();
  Serial.println("MOTOR_STOP");
  delay(3000);
}
