#include <Servo.h>

Servo servo1;
Servo servo2;

const byte SERVO1_PIN = A5;
const byte SERVO2_PIN = A4;

const int INITIAL_ANGLE = 0;
const int TARGET_ANGLE = 90;

void setup() {
  Serial.begin(115200);

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  servo1.write(INITIAL_ANGLE);
  servo2.write(INITIAL_ANGLE);

  Serial.println("Arduino READY");
}

void loop() {
  if (Serial.available()) {

    String command = Serial.readStringUntil('\n');
    command.trim();

    Serial.print("Received: ");
    Serial.println(command);

    if (command == "1") {
      servo1.write(TARGET_ANGLE);
      servo2.write(TARGET_ANGLE);

      Serial.println("SERVO -> 90");
    }
    else if (command == "0") {
      servo1.write(INITIAL_ANGLE);
      servo2.write(INITIAL_ANGLE);

      Serial.println("SERVO -> 0");
    }
  }
}
