// Overall, this code reads PWM signals from the Pixhawk to control the direction and speed of a BLDC motor
// using a PID controller. The motor speed is kept constant at the desired speed of 200 RPM,
// and the direction of the motor can be controlled using the PWM signals from the Pixhawk.
// Pins the are used in this code - arduino: 2,3,4,5,6,7

// Including the servo library, which allows the Arduino to control servo motors
#include <Servo.h>
#include <Encoder.h>

// The PWM input pins for the Pixhawk are defined as constants.
// These pins will be used to receive PWM signals from the Pixhawk that will control the direc and speed of motor
#define FORWARD_PIN 7
#define REVERSE_PIN 8
#define STOP_PIN 9
#define SPEED_PIN 10

// Define the pins for controlling the motor driver
#define DIR_PIN 4
#define PWM_PIN 5

// Defining the pins for the encoder
#define ENC_A 2
#define ENC_B 3

// Define the encoder resolution and update interval
#define ENC_RESOLUTION 1024
#define ENC_UPDATE_INTERVAL 10

// Define the motor parameters
#define MOTOR_POLES 4
#define MOTOR_GEAR_RATIO 72

// The max and min PWM duty cycles are defined as constants.
// These values will be used to limit the PWM duty cycle output to the BLDC driver
#define MAX_DUTY_CYCLE 255
#define MIN_DUTY_CYCLE 0

// The PID constants for speed control are defined (obviously as constants).
// These are used to tune the PID controller for the desired motor speed.
#define KP 1
#define KI 0.1
#define KD 0.01

// This is the desired motor speed in rpm
#define DESIRED_SPEED 200

// A Servo object is created for the BLDC driver
Servo motor;

// Create an Encoder object for the motor encoder
Encoder motor_encoder(ENC_A, ENC_B);

// Variables for the use of the PID loop.
unsigned long last_time = 0;
float last_error = 0;
float integral = 0;
float current_speed = 0;

// Defining the variables for the speed calculation
unsigned long last_enc_time = 0;
long last_enc_count = 0;

void setup() {
  // PWM input pins for the Pixhawk are initialized as inputs.
  pinMode(FORWARD_PIN, INPUT); 
  pinMode(REVERSE_PIN, INPUT);
  pinMode(SPEED_PIN, INPUT);
  
  // PWM output pins for the BLDC driver are initialized as outputs.
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  // Servo object is initialized
  motor.attach(PWM_PIN);

  // Direction pin for the motor is set to HIGH, indicating forward direction.
  digitalWrite(DIR_PIN, HIGH);

  // Motor speed is set to the minimum duty cycle (1000 microseconds).
  motor.writeMicroseconds(1000);

  // Initializing PID variables
  last_time = millis();
  last_error = 0;
  integral = 0;
}

void loop() {
  unsigned long current_time = millis();  //obtaining the current time
  float delta_time = (float)(current_time - last_time) / 1000.0;  //time since last loop
  current_speed = read_motor_speed(); //current speed of the motor

  //Obtaining the info from PWM signal
  int forward_signal = pulseIn(FORWARD_PIN, HIGH, 25000); 
  int reverse_signal = pulseIn(REVERSE_PIN, HIGH, 25000);
  int speed_signal = pulseIn(SPEED_PIN, HIGH, 25000);

  float desired_speed = map(speed_signal, 1000, 2000, 0, 255); //Calculating speed based on PWM signal

  //Setting direction of motor according to PWM signal
  if (forward_signal > 1500 && reverse_signal < 1500) {
    digitalWrite(DIR_PIN, HIGH);
  
  } else if (reverse_signal > 1500 && forward_signal < 1500) {
    digitalWrite(DIR_PIN, LOW);
  }

  //Calculating error of speed and implimenting the PID control while making sure its within the max/min duty cycle
  float error = desired_speed - current_speed;
  float derivative = (error - last_error) / delta_time;
  integral += error * delta_time;
  float output = KP * error + KI * integral + KD * derivative;
  output = constrain(output, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
  motor.writeMicroseconds(map(output, 0, 255, 1000, 2000)); // Setting new duty cycle 

  last_time = current_time;
  last_error = error;
  
  // Read the PWM signal for the stop command
  int stop_signal = pulseIn(STOP_PIN, HIGH, 25000);
  
  // If the stop signal is detected, set the motor to stop
  if (stop_signal > 1500) {
    motor.writeMicroseconds(1500);
  }
}

// Define the function for reading the motor speed
float read_motor_speed() {
  // Get the current time
  unsigned long current_time = millis();

  // Calculate the time elapsed since the last update
  unsigned long delta_time = current_time - last_enc_time;

  // Only update the speed if enough time has passed
  if (delta_time >= ENC_UPDATE_INTERVAL) {
    // Read the current encoder count
    long current_enc_count = motor_encoder.read();

    // Calculate the encoder count difference
    long enc_count_diff = current_enc_count - last_enc_count;

    // Calculate the motor speed in RPM
    float motor_speed = (float)enc_count_diff / (float)ENC_RESOLUTION;
    motor_speed *= (float)(60000 / ENC_UPDATE_INTERVAL);
    motor_speed /= (float)(MOTOR_POLES * MOTOR_GEAR_RATIO);

    // Save the current encoder count and time for the next update
    last_enc_count = current_enc_count;
    last_enc_time = current_time;

    // Return the motor speed in RPM
    return motor_speed;
  }

  // If not enough time has passed, return the previous motor speed
  return current_speed;
}