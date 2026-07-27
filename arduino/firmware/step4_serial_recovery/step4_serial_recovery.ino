// Recovery firmware: motor serial protocol plus watchdog, with encoder ISRs
// intentionally disabled while diagnosing an interrupt flood.
const int L_IN1 = 4, L_IN2 = 5, L_ENA = 6;
const int R_IN1 = 7, R_IN2 = 8, R_ENB = 9;
const int STBY = 10;

const unsigned long SEND_PERIOD_MS = 20;
const unsigned long CMD_TIMEOUT_MS = 250;

String cmdBuf = "";
unsigned long lastSend = 0;
unsigned long lastCmd = 0;

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

void stopMotors() {
  setMotor(L_IN1, L_IN2, L_ENA, 0);
  setMotor(R_IN1, R_IN2, R_ENB, 0);
}

void setup() {
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT); pinMode(L_ENA, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT); pinMode(R_ENB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  stopMotors();
  Serial.begin(115200);
  cmdBuf.reserve(24);
  lastCmd = millis();
}

void loop() {
  if (millis() - lastSend >= SEND_PERIOD_MS) {
    Serial.println("E,0,0");
    lastSend = millis();
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (cmdBuf.startsWith("M,")) {
        int comma = cmdBuf.indexOf(',', 2);
        if (comma > 2) {
          int leftPwm = cmdBuf.substring(2, comma).toInt();
          int rightPwm = cmdBuf.substring(comma + 1).toInt();
          setMotor(L_IN1, L_IN2, L_ENA, leftPwm);
          setMotor(R_IN1, R_IN2, R_ENB, rightPwm);
          lastCmd = millis();
        }
      }
      cmdBuf = "";
    } else {
      cmdBuf += c;
    }
  }

  if (millis() - lastCmd > CMD_TIMEOUT_MS) {
    stopMotors();
  }
}
