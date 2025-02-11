
// ***************************************Libraries included**************************************
#include <EEPROM.h>
#include <string>


// ***************************************Variable Defination:**************************************
int irArray[8];// IR Sensor Array:
int speed = 150;//motor speed
int rot_speed = 100;

int thres = 400;//threshold above which sensor surely detects white

int turnDelay = 200;//delay for which it should turn left/right
int extraInchDelay = 100;

// PID variables:
float Kp = 45; 
float Kd = 210;
float error = 0, errorLast = 0;

std::string path = "";//store path, test path = "LBSLLBRSLLBSLLRLLBSLLSRLLBLBSLLRLL", soln = "RRLRLLRLRLLSRSLRLL"
std::string optimizedPath = "";

bool scanningMode = true;
bool state = false;


// ***************************************Pins Defination:**************************************
//NOTE: pin numbers are arbitrary as of now, change them according to pins recommended by Bot dynamics/PCB designing team

// Motor Control:
const int enaA = 11;//enable pin A
const int enaB = 12;//enable pin B
const int motorRfwd = 9;
const int motorRback = 10;
const int motorLfwd = 7;
const int motorLback = 8;

// Sensors:
const int s1 = 0;
const int s2 = 1;
const int s3 = 2;
const int s4 = 3;
const int s5 = 4;
const int s6 = 5;
const int s7 = 6;
const int s8 = 16;

// LED pin:
const int led = 13;//led to indicate we reached the end

// Buttons:
const int selection = 14;//button to change mode
const int stateChange = 15;//button to start scanning/solving

// ***************************************Setup**************************************
void setup() {
  pinMode(s1, INPUT);
  pinMode(s2, INPUT);
  pinMode(s3, INPUT);
  pinMode(s4, INPUT);
  pinMode(s5, INPUT);
  pinMode(s6, INPUT);
  pinMode(s7, INPUT);
  pinMode(s8, INPUT);
  pinMode(motorRfwd, OUTPUT);
  pinMode(motorRback, OUTPUT);
  pinMode(motorLfwd, OUTPUT);
  pinMode(motorLback, OUTPUT);
  pinMode(enaA, OUTPUT);
  pinMode(enaB, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(selection, OUTPUT);
  pinMode(stateChange, OUTPUT);

  stopMotors();
}


// ***************************************Functions**************************************

// Function to read irArray
void readIR() {
  irArray[0] = analogRead(s1);
  irArray[1] = analogRead(s2);
  irArray[2] = analogRead(s3);
  irArray[3] = analogRead(s4);
  irArray[4] = analogRead(s5);
  irArray[5] = analogRead(s6);
  irArray[6] = analogRead(s7);
  irArray[7] = analogRead(s8);
}


// Function to check whether the bot is on line
bool onLine(int irArray[8])
{
  return (irArray[0]>thres) && (irArray[7]>thres) && ((irArray[3]<thres) || (irArray[4]<thres));
}


// Function to calculate input based on IR sensor array
float calcInput(int irArray[8]) {
  // Assuming a weighted average approach for line position
  float position = ((-1)*5*irArray[1]) + ((-1)*3*irArray[2]) + ((-1)*1*irArray[3]) + (1*irArray[4]) + (3*irArray[5]) + (5*irArray[6]);
  float totalWeight = 0;

  for (int i = 0; i < 8; i++) {
      totalWeight += irArray[i];
    }
  
  // If no sensors are above the threshold, return 0 as a fallback
  if (totalWeight == 0) {
    return 0;
  }

  return ((position / totalWeight) - 3.5); // Centering around 0
}

// Function to calculate PD value
float calcPd(float input) {
  float errorDiff;
  float output;
  error = error * 0.08 + input * 0.92; // Low-pass filter
  errorDiff = error - errorLast;
  output = Kp * error + Kd * errorDiff;
  errorLast = error;
  return output;
}


// Handle the case when off the line
void handleOffLine() {
  if (irArray[1] < thres && irArray[2] < thres && irArray[3] < thres && irArray[4] < thres && irArray[5] < thres && irArray[6] < thres) {
    extraInch();
    if (irArray[1] < thres && irArray[2] < thres && irArray[3] < thres && irArray[4] < thres && irArray[5] < thres && irArray[6] < thres) {
      stopMotors();
      digitalWrite(led, HIGH);
      path +='E';
      state = false;
    }
  }

  if (irArray[0] < thres && irArray[1] < thres) {
    // Turn left if left sensor detects the line
    turnLeft();
    path += 'L'; // Record the turn
  } 
  else {
    bool rightAvailable = (irArray[6] < thres && irArray[7] < thres);
    extraInch();
    bool straightAvailable = (irArray[3] < thres && irArray[4] < thres);
    bool deadEnd = (irArray[3] > thres && irArray[4] > thres);
    if (straightAvailable) {
    forward(0);
    path += 'S'; // Record the turn
    }
    else if (rightAvailable) {
    turnRight();
    path += 'R'; // Record the turn
    }
    else if (deadEnd) {
    uTurn();
    path += 'B'; // Record the turn
    }
  }
}


// Function to debounce button inputs
bool debounceButton(int pin) {
  static int lastState = LOW;
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50; // debounce time in milliseconds
  int reading = digitalRead(pin);
  
  if (reading != lastState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != lastState) {
      lastState = reading;
      return reading == HIGH; // Return true if the button is pressed
    }
  }
  
  return false; // Return false if the button is not pressed
}

// Function to optimize the path
std::string optimizeThePath(std::string path)
{
  while (true)
  {
    bool found = false;
    for (int i = 0; i < path.length() - 2; i++)
    {
      std::string substring = path.substr(i, 3);

      if (substring == "LBR") {
        path.replace(i, 3, "B");
        found = true;
        break;
      } else if (substring == "RBL") {
        path.replace(i, 3, "B");
        found = true;
        break;
      } else if (substring == "SBL") {
        path.replace(i, 3, "R");
        found = true;
        break;
      } else if (substring == "LBL") {
        path.replace(i, 3, "S");
        found = true;
        break;
      } else if (substring == "RBR") {
        path.replace(i, 3, "S");
        found = true;
        break;
      } else if (substring == "SBR") {
        path.replace(i, 3, "L");
        found = true;
        break;
      } else if (substring == "LBS") {
        path.replace(i, 3, "R");
        found = true;
        break;
      } else if (substring == "RBS") {
        path.replace(i, 3, "L");
        found = true;
        break;
      } else if (substring == "SBS") {
        path.replace(i, 3, "B");
        found = true;
        break;
      }
    }

    if (!found) break;
  }
  return path;
}


//***************************************Movement-Functions**************************************
// Function to move forward based on PD output
void forward(float pdOutput) {
  int leftMotorSpeed = speed + pdOutput;
  int rightMotorSpeed = speed - pdOutput;

  // Constrain motor speeds to avoid overflow
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  // Set motor speeds
  analogWrite(enaA,leftMotorSpeed);
  analogWrite(enaB,rightMotorSpeed);
  digitalWrite(motorRfwd,HIGH);
  digitalWrite(motorLfwd,HIGH);
  digitalWrite(motorLback,LOW);
  digitalWrite(motorRback,LOW);
}

// Function to turn right
void turnRight() {
  analogWrite(enaA,rot_speed);
  analogWrite(enaB,rot_speed);
  digitalWrite(motorRfwd,LOW);
  digitalWrite(motorLfwd,HIGH);
  digitalWrite(motorLback,LOW);
  digitalWrite(motorRback,HIGH);
  delay(turnDelay); // Adjust delay for your needs
}

// Function to turn left
void turnLeft() {
  analogWrite(enaA,rot_speed);
  analogWrite(enaB,rot_speed);
  digitalWrite(motorRfwd,HIGH);
  digitalWrite(motorLfwd,LOW);
  digitalWrite(motorLback,HIGH);
  digitalWrite(motorRback,LOW);
  delay(turnDelay); // Adjust delay for your needs
}


// Function to perform a U-turn
void uTurn() {
  turnLeft();
  turnLeft(); // Two left turns for U-turn
}

// Function to stop motors
void stopMotors() {
  digitalWrite(motorRfwd,LOW);
  digitalWrite(motorLfwd,LOW);
  digitalWrite(motorLback,LOW);
  digitalWrite(motorRback,LOW);
}

void extraInch(){
  forward(0);
  delay(extraInchDelay);
}


void scanMaze()
{
  readIR();
  
  if (onLine(irArray)) {
    float input = calcInput(irArray); // Calculate input for PD
    error = calcPd(input); // Get the PD output
    forward(error); // Move forward based on PD output
    
  } else {
    handleOffLine(); // Handle the case when off line
  }
}

int iTurn = 0;

void solveMaze(){
  readIR();
  
  if (onLine(irArray)) {
    float input = calcInput(irArray); // Calculate input for PD
    error = calcPd(input); // Get the PD output
    forward(error); // Move forward based on PD output
    
  } else {
    char turn = optimizedPath[iTurn];
    if (turn=='S') {
    forward(0);
    delay(extraInchDelay);
    }
    else if (turn =='L') {
    turnLeft();
    }
    else if (turn =='R') {
    turnRight();
    }
    else if (turn =='E') {
    stopMotors();
    digitalWrite(led, HIGH);
    state =false;
    }
    iTurn++;
    iTurn = iTurn.constrain(0, path.length());
  }

}



// ***************************************Main Loop**************************************
void loop() {

  static bool flag = true;

  // Check button states
  bool selectionPressed = debounceButton(selection);
  bool stateChangePressed = debounceButton(stateChange);

  // Handle selection button
  if (selectionPressed) {
    scanningMode = !scanningMode; // Toggle scanning mode
    delay(200); // Debounce delay after pressing selection button
  }

  // Handle state change button
  if (stateChangePressed) {
    state = true; // Set state to true when stateChange button is pressed
    delay(200); // Debounce delay after pressing stateChange button
  }

  if (state) {
    if (selection) {
      flag = true;
      scanMaze();
    }

    else if ((!selection) && (path.length() !=0)){
      if (flag) {
        optimizedPath = optimizeThePath(path);
        digitalWrite(led, LOW);
        iTurn = 0;
        flag = false;
      }
      solveMaze();
    }
    else {
      stopMotors();
    }
  }
}
