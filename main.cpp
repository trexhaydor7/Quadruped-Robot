#include <Servo.h>

int tibiaPin = 5;
int femurPin = 3;

//in mm
float l1 = 95.155;
float l2 = 90.155;

Servo femur;
Servo tibia; // Tibia servo range is limit from 47 to 113 degrees

void setup() {
  // put your setup code here, to run once:
  femur.attach(femurPin);
  tibia.attach(tibiaPin);
  Serial.begin(9600);
  calibrate();
}

void calibrate()
{
  femur.write(80);
  tibia.write(90);

}

  int angle = 90;

void loop() {

  delay(5000);
  
  double example[2];

  calculateInverseKinematics(1000, 200, example);

  example[0] = constrain(example[0]* 180/PI, 0, 180);
  example[1] = constrain(example[1]* 180/PI, 47, 113);

    Serial.print("Theta 1: ");
    Serial.println(example[0]);
    Serial.print("Theta 2: ");
    Serial.println(example[1]);

  femur.write(example[0]);
  tibia.write(example[1]);

  delay(5000);
  
}

void calculateInverseKinematics(int x, int y, double arr[2])
{
  //If the desired coordinates would throw an mathematical error, default to angle 0 for both angles
    double dist = sqrt(x*x + y*y);
    if(dist < 1e-6) {
        arr[0] = 0;
        arr[1] = 0;
        return;
    }

  //Calulate theta 2
  double c2 = (c2 = x*x + y*y - l1*l1 - l2*l2)/(2.0 * l1 * l2);

  c2 = constrain(c2, -1, 1);

  arr[1] = acos(c2);
  
  //Calculate theta 1
  double k1 = l1 + l2 * cos(arr[1]);
  double k2 = l2 * sin(arr[1]);

  double c1 = atan2( (y * k1 - x * k2), (x * k1 + y * k2));

  arr[0] = constrain(c1, -1, 1);
}
