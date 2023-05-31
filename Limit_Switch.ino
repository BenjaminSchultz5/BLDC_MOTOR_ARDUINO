// The following script reads the current state of the limit
// switch using digitalRead() function and stores it in the currentState variable. Then, it checks if the current state
// is different from the previous state. If it is, it means the state of the limit switch has changed.

// If the current state is LOW, it means the limit switch is 
// untouched or not triggered. In this case, it sets the PWM output to a value of 255 (maximum).

// If the current state is not LOW, it means the limit switch is touched or triggered. In this case, it sets the PWM output to
// 0,  "turning it off".

// After this, the previousState variable is updated to store the current state, and the loop continues to read the state of the limit switch.

// Pin definitions
const int limitSwitchPin = 12;  // Connect the limit switch to pin 12
const int pwmPin = 13;          // Connect the PWM output to pin 13

// Variables
int previousState = HIGH;      // Previous state of the limit switch

void setup() {
  pinMode(limitSwitchPin, INPUT_PULLUP);  // Set the limit switch pin as input with internal pull-up resistor
  pinMode(pwmPin, OUTPUT);                 // Set the PWM pin as output
}

void loop() {
  int currentState = digitalRead(limitSwitchPin);  // Read the current state of the limit switch

  if (currentState != previousState) {
    // State of the limit switch has changed

    if (currentState == LOW) {
      // Limit switch is untouched

      // Send message to Pixhawk via PWM
      analogWrite(pwmPin, 255);  // Change the PWM value based on your requirements
    } else {
      // Limit switch is touched

      // Reset PWM output to default value (0)
      analogWrite(pwmPin, 0);
    }

    previousState = currentState;  // Update previous state
  }
}
