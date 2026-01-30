#include <Servo.h>
#include <Arduino.h>
#include <optional>

#define BOLD "\\033[1m"
#define OFF "\\033[0m"

unsigned long t0;
bool motorsCalibrated = false;
int baudRate = 9600;

int tibiaPin = 5;
int femurPin = 3;

//in mm
float l1 = 100;
float l2 = 90.981;

// Tibia servo range is limit from 47 to 113 degrees
const int TIBIA_MIN = 47;
const int TIBIA_MAX = 113;

const int FEMUR_MIN = 0;
const int FEMUR_MAX = 180;

const int FEMUR_ZERO = 80;
const int TIBIA_ZERO = 95;

Servo femur;
Servo tibia; 

//List of all possible errors
enum ERR {
  ERR_NONE,
  ERR_IK_UNREACHABLE,
  ERR_IK_NAN,
  ERR_IK_FK_MISMATCH,
  ERR_SERVO_LIMIT
};

//List of all possible system modes
enum Mode {
  INIT,
  CALIBRATION,
  RUNNING,
  IDLE
};

//Struct to store an input for calculating inverse kinematics
struct Coordinates {
  double x;
  double y;
  std::optional<int> place; //for index if testing many points for gaits
};

//Struct for result of inverse kinematics calculations
struct Angles {
  double theta1;
  double theta2;
};

//Current error state and mode of the robot 
ERR currentERR;
Mode currentMode; 

void setup() {
  // put your setup code here, to run once
  t0 = millis();
  Serial.begin(baudRate);
  
  while((millis() - t0) < 2000)
  {
    ;
  }

  setERR(ERR_NONE);
  setMode(INIT);

  femur.attach(femurPin);
  tibia.attach(tibiaPin);

  if(!motorsCalibrated)
  {
    setMode(CALIBRATION);
    calibrate();
    motorsCalibrated = true;
  }
  
  setMode(IDLE);
}

void loop() {

  delay(1000);
  if(!motorsCalibrated)
  {
    setMode(CALIBRATION);
    calibrate();
    motorsCalibrated = true;
  }

  Coordinates input{0,-50.00};
  Angles angles = calculateInverseKinematics(input);

  if(currentERR != ERR_NONE) {
    setMode(IDLE);
    return;
  }

  double tibiaDEG = TIBIA_ZERO + angles.theta2 * 180/PI;
  double femurDEG = FEMUR_ZERO + angles.theta1 * 180/PI;

  if(tibiaDEG < TIBIA_MIN || tibiaDEG > TIBIA_MAX)
  {
    setERR(ERR_SERVO_LIMIT);
    return;
  }

  if(femurDEG < FEMUR_MIN || femurDEG > FEMUR_MAX)
  {
    setERR(ERR_SERVO_LIMIT);
    return;
  }

  setERR(ERR_NONE);

  //Generally good practice to isolate calculation from used values
  Angles servoAngles = angles;

  servoAngles.theta1 = constrain( femurDEG, FEMUR_MIN, FEMUR_MAX);
  servoAngles.theta2 = constrain( tibiaDEG, TIBIA_MIN, TIBIA_MAX);

   Coordinates checked = calculateForwardKinematics(angles);
   if(!checkIK(input, checked)) {
    setMode(IDLE);
    return;
   }
   setERR(ERR_NONE);

    Serial.print("Femur Angle:  ");
    Serial.println(servoAngles.theta1);
    Serial.print("Tibia Angle: ");
    Serial.println(servoAngles.theta2);

  setMode(RUNNING);
  femur.write(servoAngles.theta1);
  tibia.write(servoAngles.theta2);
  setMode(IDLE);
  delay(1000);
}

//Helper method to calibrate all servo motors
void calibrate() {
  femur.write(FEMUR_ZERO);
  tibia.write(TIBIA_ZERO);
}

//Helper method to provide information on errors
void printERR(ERR currentERR) {

  Serial.print("[ERROR] ");

  switch(currentERR) {
    case ERR_NONE:
      Serial.println("Ok");
      break;
    case ERR_IK_UNREACHABLE:
      Serial.println("Unreachable coordinate");
      break;
    case ERR_IK_NAN:
      Serial.println("Unreachable coordinate : IK Math caused NaN");
      break;
    case ERR_SERVO_LIMIT:
      Serial.println("Angle desired is out of bounds of servo motor");
      break;
    case ERR_IK_FK_MISMATCH:
      Serial.println("Discrepancy in calculated angles and desired coordinates");
      break;
    default:
      Serial.println("Unknown Error");
      break;
  } 
}

void printMode(Mode currentMode) {

  Serial.print("[SYS] ");

  switch(currentMode) {
    case INIT:
      Serial.println("Initializing quadruped robot");
      break;
    case CALIBRATION:
      Serial.println("Calibrating servo motors");
      break;
    case RUNNING:
      Serial.println("System is running");
      break;
    case IDLE:
      Serial.println("System is idle");
      break;
    default:
      Serial.println("Unknown System mode");
      break;
  } 
}

//Helper method to indicate new system mode of robotic system
void setMode(Mode newMode) {
  if(currentMode == newMode) return;
  currentMode = newMode;
  printMode(currentMode);
}

//Helper method to indicate new erro of robotic system
void setERR(ERR newERR) {
  if(currentERR == newERR) return;
  currentERR = newERR;
  printERR(currentERR);
}


//Function to calculate two angles for desired x and y coordiante
Angles calculateInverseKinematics(Coordinates point) {

  Angles result;
  //If the desired coordinates would throw an mathematical error, default to angle 0 for both angles
  double dist = sqrt(point.x*point.x + point.y*point.y);
    if(dist < 1e-6) {
        setERR(ERR_IK_NAN);
        result.theta1 = 0;
        result.theta2 = 0;
        return result;
    }

  //Check if desired coordinates are not reachable for the given femur and tibia length
  if (dist > l1 + l2 || dist < fabs(l1 - l2)) {
      setERR(ERR_IK_UNREACHABLE);
      result.theta1 = 0;
      result.theta2 = 0;
      return result;
  }

  //Calulate theta 2
  double c2 = (point.x*point.x + point.y*point.y - l1*l1 - l2*l2)/(2.0 * l1 * l2);
  c2 = constrain(c2, -1.0, 1.0);
  result.theta2 = acos(c2);
  
  //Calculate theta 1
  double k1 = l1 + l2 * cos(result.theta2);
  double k2 = l2 * sin(result.theta2);

  double c1 = atan2( (point.y * k1 - point.x * k2), (point.x * k1 + point.y * k2));

  result.theta1 = c1;

  return result;
}

//Function to verify the calculations of inverse kinematics
Coordinates calculateForwardKinematics(Angles angles)
{
    Coordinates result; 

    result.x = l1 * cos(angles.theta1) + l2 * cos(angles.theta1 + angles.theta2);
    result.y = l1 * sin(angles.theta1) + l2 * sin(angles.theta1 + angles.theta2);

    return result;
}


//Helper method to compare desired vs calculated coordinates
bool checkIK(Coordinates input, Coordinates checked)
{
  //Include some tolerance for desired vs calculated coordinates
  const double EPS = 2.0;
  
  if(fabs(input.x - checked.x) < EPS && fabs(input.y - checked.y) < EPS)
  {
    return true;
  } else
  {
    setERR(ERR_IK_FK_MISMATCH);
    return false;
  }
}