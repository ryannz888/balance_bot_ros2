const int L_IN1 = 4, L_IN2 = 5, L_ENA = 6;
const int R_IN1 = 7, R_IN2 = 8, R_ENB = 9;
const int STBY = 10;
const int L_ENC_A = 2, L_ENC_B = 3;
const int R_ENC_A = 18, R_ENC_B = 19;

volatile long l_count = 0, r_count = 0;

void lEncISR() { l_count += (digitalRead(L_ENC_B) == HIGH) ? 1 : -1; }
void rEncISR() { r_count += (digitalRead(R_ENC_B) == HIGH) ? 1 : -1; }

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

String cmdBuf = "";

void setup() {
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT); pinMode(L_ENA, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT); pinMode(R_ENB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  pinMode(L_ENC_A, INPUT_PULLUP); pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP); pinMode(R_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), lEncISR, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rEncISR, RISING);
  Serial.begin(115200);
}

unsigned long lastSend = 0;

void loop() {
  // 每10ms发一次编码器数据
  if (millis() - lastSend >= 10) {
    Serial.print("E,");
    Serial.print(l_count);
    Serial.print(",");
    Serial.println(r_count);
    lastSend = millis();
  }

  // 接收串口指令
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      // 解析 M,<left_pwm>,<right_pwm>
      if (cmdBuf.startsWith("M,")) {
        int comma = cmdBuf.indexOf(',', 2);
        int l_pwm = cmdBuf.substring(2, comma).toInt();
        int r_pwm = cmdBuf.substring(comma + 1).toInt();
        setMotor(L_IN1, L_IN2, L_ENA, l_pwm);
        setMotor(R_IN1, R_IN2, R_ENB, r_pwm);
      }
      cmdBuf = "";
    } else {
      cmdBuf += c;
    }
  }
}
