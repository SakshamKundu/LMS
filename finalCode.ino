#define NUM_SENSORS 5
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define ENA_CHANNEL 0
#define ENB_CHANNEL 1
const int sensorPins[NUM_SENSORS] = { 34, 14, 33, 32, 35 };

const int led = 12;

const int ENA = 23;  //enable pin A
const int ENB = 16;  //enable pin B
const int motorRfwd = 5;
const int motorRback = 17;
const int motorLfwd = 19;
const int motorLback = 18
;
int sensors[5];                                         // IR Sensor Array:
int sensors_max[5] = { 0, 0, 0, 0, 0 };                 // IR Sensor Array:
int sensors_min[5] = { 4095, 4095, 4095, 4095, 4095 };  // IR Sensor Array:
int threshhold[5];

void setup() {
  pinMode(motorRfwd, OUTPUT);
  pinMode(motorRback, OUTPUT);
  pinMode(motorLfwd, OUTPUT);
  pinMode(motorLback, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  ledcSetup(ENA_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(ENB_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA, ENA_CHANNEL);
  ledcAttachPin(ENB, ENB_CHANNEL);

  pinMode(led, OUTPUT);

  Serial.begin(115200);
  analogReadResolution(10);

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    for (int i = 0; i < 5; i++) {
      Serial.println("Calibrating...");
      calibrate();
      sensors[i] = analogRead(sensorPins[i]);
      sensors_max[i] = max(sensors[i], sensors_max[i]);
      sensors_min[i] = min(sensors[i], sensors_min[i]);
    }
  }


  moveBackward();
  delay(2000);
}

void loop() {
  for (int i = 0; i < 5; i++) { sensors[i] = analogRead(sensorPins[i]) < threshhold[i] ? 1 : 0; }

  if (sensors[0]) {
    turnLeft();

  }

  else if (!sensors[0] && !sensors[1] && !sensors[2] && !sensors[3] && !sensors[4]) {
    turnRight();
  }

  else {
    straight();
  }
}


void straight() {
  if (sensors[1]) slightLeft();
  else if (sensors[3]) slightRight();
  else moveForward();
}

void moveForward() {
  ledcWrite(ENA_CHANNEL, 210);
  ledcWrite(ENB_CHANNEL, 210);
  digitalWrite(motorRfwd, HIGH);
  digitalWrite(motorRback, LOW);
  digitalWrite(motorLfwd, HIGH);
  digitalWrite(motorLback, LOW);
  delay(10);
}

void moveBackward() {
  ledcWrite(ENA_CHANNEL, 200);
  ledcWrite(ENB_CHANNEL, 200);
  digitalWrite(motorRfwd, LOW);
  digitalWrite(motorRback, HIGH);
  digitalWrite(motorLfwd, LOW);
  digitalWrite(motorLback, HIGH);
}

void slightLeft() {
  ledcWrite(ENA_CHANNEL, 160);  //enable A is left.
  ledcWrite(ENB_CHANNEL, 190);
  digitalWrite(motorRfwd, HIGH);
  digitalWrite(motorRback, LOW);
  digitalWrite(motorLfwd, HIGH);
  digitalWrite(motorLback, LOW);
}


void calibrate() {
  ledcWrite(ENA_CHANNEL, 200);  //enable A is left.
  ledcWrite(ENB_CHANNEL, 200);
  digitalWrite(motorRfwd, HIGH);
  digitalWrite(motorRback, LOW);
  digitalWrite(motorLfwd, LOW);
  digitalWrite(motorLback, HIGH);
}

void slightRight() {
  ledcWrite(ENA_CHANNEL, 190);  //enable A is left.
  ledcWrite(ENB_CHANNEL, 160);
  digitalWrite(motorRfwd, HIGH);
  digitalWrite(motorRback, LOW);
  digitalWrite(motorLfwd, HIGH);
  digitalWrite(motorLback, LOW);
}

void stopMotors(int duration) {
  digitalWrite(motorRfwd, LOW);
  digitalWrite(motorLback, LOW);
  digitalWrite(motorLfwd, LOW);
  digitalWrite(motorRback, LOW);
  delay(duration);
}

void turnLeft() {
  int startMillis = millis();
  while (sensors[0]) {
    for (int i = 0; i < 5; i++) { sensors[i] = analogRead(sensorPins[i]) < threshhold[i] ? 1 : 0; }
    ledcWrite(ENA_CHANNEL, 180);  //enable A is left.
    ledcWrite(ENB_CHANNEL, 180);
    digitalWrite(motorRfwd, HIGH);
    digitalWrite(motorRback, LOW);
    digitalWrite(motorLfwd, LOW);
    digitalWrite(motorLback, HIGH);
    }
  }
  
    if (millis() - startMillis > 400) {
    ledcWrite(ENA_CHANNEL, 180);  //enable A is left.
    ledcWrite(ENB_CHANNEL, 180);
    digitalWrite(motorRfwd, LOW);
    digitalWrite(motorRback, HIGH);
    digitalWrite(motorLfwd, HIGH);
    digitalWrite(motorLback, LOW);
      
      delay(300);
      // moveForward();
      // delay(75);
      digitalWrite(led,HIGH);
      stopMotors(5000);
    }
}

void turnRight() {
  while (!sensors[2]) {
    for (int i = 0; i < 5; i++) { sensors[i] = analogRead(sensorPins[i]) < threshhold[i] ? 1 : 0; }
    ledcWrite(ENA_CHANNEL, 180);  //enable A is left.
    ledcWrite(ENB_CHANNEL, 180);
    digitalWrite(motorRfwd, LOW);
    digitalWrite(motorRback, HIGH);
    digitalWrite(motorLfwd, HIGH);
    digitalWrite(motorLback, LOW);
  }

}