#include <Servo.h> // include the servo library

Servo servo1; // create servo object for servo 1 and 2
Servo servo3; // create servo object for servo 3

int pwm1 = 0; // variable to store PWM signal for servo 1 and 2
int pwm3 = 0; // variable to store PWM signal for servo 3

void setup() {
  servo1.attach(10); // attach servo 1 to pin 10
  servo3.attach(11); // attach servo 3 to pin 11
  
  Serial.begin(9600); // initialize serial communication
}

void loop() {
  // Read PWM signal from Pixhawk for servo 1 and 2
  pwm1 = pulseIn(A0, HIGH, 25000); // read PWM signal on pin A0 with a timeout of 25ms
  pwm1 = map(pwm1, 1000, 2000, 0, 180); // map pulse width to servo angle (0-180 degrees)
  
  // Set servo 1 and 2 angles simultaneously
  servo1.write(pwm1);
  
  // Read PWM signal from Pixhawk for servo 3
  pwm3 = pulseIn(A1, HIGH, 25000); // read PWM signal on pin A1 with a timeout of 25ms
  pwm3 = map(pwm3, 1000, 2000, 0, 90); // map pulse width to servo angle (0-90 degrees)
  servo3.write(pwm3); // set servo 3 angle
  
  // Check if a command has been received from the user
  if (Serial.available() > 0) {
    int command = Serial.read(); // read the incoming command
    
    // Map the command to a specific angle for each servo
    if (command == '1') {
      servo1.write(0); // turn servo 1 to 0 degrees
    }
    else if (command == '2') {
      servo1.write(10); // turn servo 1 to 10 degrees
    }
    else if (command == '3') {
      servo3.write(0); // turn servo 3 to 0 degrees
    }
    else if (command == '4') {
      servo3.write(90); // turn servo 3 to 90 degrees
    }
  }
}
