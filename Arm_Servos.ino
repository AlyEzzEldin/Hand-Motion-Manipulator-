#include <Servo.h>

// Servo objects for pan, tilt, gripper
Servo panServo;
Servo tiltServo;
Servo gripServo;

// Motor control pins (adjust to your driver)
const int motorEnablePin = 6;   // PWM pin
const int motorDirPin1   = 8;   // Direction pin 1
const int motorDirPin2   = 9;   // Direction pin 2

// Servo pin assignments
const int panPin   = 3;
const int tiltPin  = 5;
const int gripPin  = 7;

// Current positions
int panPos   = 90;
int tiltPos  = 90;
int gripPos  = 0;    // 0 = closed, 90 = open
int sliderSpeed = 0;

void setup() {
  Serial.begin(115200);
  panServo.attach(panPin);
  tiltServo.attach(tiltPin);
  gripServo.attach(gripPin);

  pinMode(motorEnablePin, OUTPUT);
  pinMode(motorDirPin1, OUTPUT);
  pinMode(motorDirPin2, OUTPUT);

  // Initialize positions
  panServo.write(panPos);
  tiltServo.write(tiltPos);
  gripServo.write(gripPos);
  analogWrite(motorEnablePin, 0);
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    // Expect format: S<slider> P<pan> T<tilt> G<grip>
    int s = parseValue(line, 'S');
    int p = parseValue(line, 'P');
    int t = parseValue(line, 'T');
    int g = parseValue(line, 'G');

    // Update servos
    if (p >= 0 && p <= 180) {
      panPos = p;
      panServo.write(panPos);
    }
    if (t >= 0 && t <= 180) {
      tiltPos = t;
      tiltServo.write(tiltPos);
    }
    if (g >= 0 && g <= 90) {
      gripPos = g;
      gripServo.write(gripPos);
    }

    // Slider: map S (0-180) to motor speed and direction
    // Center (90) = stop, above = forward, below = reverse
    sliderSpeed = abs(s - 90);
    if (s > 90) {
      digitalWrite(motorDirPin1, HIGH);
      digitalWrite(motorDirPin2, LOW);
    } else if (s < 90) {
      digitalWrite(motorDirPin1, LOW);
      digitalWrite(motorDirPin2, HIGH);
    } else {
      // stop
      sliderSpeed = 0;
    }
    analogWrite(motorEnablePin, map(sliderSpeed, 0, 90, 0, 255));

    // Echo back status
    Serial.print("OK S"); Serial.print(s);
    Serial.print(" P"); Serial.print(p);
    Serial.print(" T"); Serial.print(t);
    Serial.print(" G"); Serial.println(g);
  }
}

int parseValue(const String &str, char key) {
  int idx = str.indexOf(key);
  if (idx == -1) return -1;
  return str.substring(idx + 1).toInt();
}
