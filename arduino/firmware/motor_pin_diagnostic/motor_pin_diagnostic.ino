// Reports exactly how far Mega reaches while initializing motor-control pins.
const int L_IN1 = 4, L_IN2 = 5, L_ENA = 6;
const int R_IN1 = 7, R_IN2 = 8, R_ENB = 9;
const int STBY = 10;

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("S0_SERIAL_OK");

  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT); pinMode(L_ENA, OUTPUT);
  Serial.println("S1_LEFT_PINS_OK");

  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT); pinMode(R_ENB, OUTPUT);
  Serial.println("S2_RIGHT_PINS_OK");

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);  // Keep the motor driver disabled during diagnosis.
  Serial.println("S3_STBY_LOW_OK");
}

void loop() {
  Serial.println("S4_LOOP_ALIVE");
  delay(500);
}
