#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int SERVO_FREQ = 50;

const int BASE_CH = 0;
const int SHOULDER_CH = 1;
const int ELBOW_CH = 2;

const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

String inputLine ="";

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100);

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  
  delay(500);

  moveServo(BASE_CH, 90);
  moveServo(SHOULDER_CH, 90);
  moveServo(ELBOW_CH, 90);

  Serial.println("READY");
}

void loop() {
  if (Serial.available() > 0) {
    inputLine = Serial.readString();

    inputLine.trim();

    if (inputLine == "PING") {
      Serial.println("PONG");
    }

    else if (inputLine == "HOME") {
      moveServo(BASE_CH, 90);
      moveServo(SHOULDER_CH, 90);
      moveServo(ELBOW_CH, 90);

      Serial.println("OK");
    }

    else if (inputLine.startsWith("MOVE")) {
      int a1, a2, a3;

      int count = sscanf(inputLine.c_str(), "MOVE %d %d %d", &a1, &a2, &a3);

      if (count == 3) {
        a1 = constrain(a1, 0, 180);
        a2 = constrain(a2, 0, 180);
        a3 = constrain(a3, 0, 180);

        moveServo(BASE_CH, a1);
        moveServo(SHOULDER_CH, a2);
        moveServo(ELBOW_CH, a3);

        Serial.println("OK");
      }
      else {
        Serial.println("ERROR");
      }
    }
    else {
      Serial.println("UNKNOWN");

    }
  }
}

void moveServo(int channel, int angle) {
  int pulseUS = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);

  int pwmValue = pulseUS*4096/20000;

  pwm.setPWM(channel, 0, pwmValue);
}



