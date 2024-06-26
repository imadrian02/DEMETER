#include <AccelStepper.h>
#include <Servo.h>

// Define the steps per millimeter for your stepper motors
#define STEPS_PER_MM 5

// Define the maximum speed and acceleration for your stepper motors
#define MAX_SPEED_X 1500
#define MAX_SPEED_Y 2000
#define ACCELERATION_X 2000
#define ACCELERATION_Y 2000

// Define the pins connected to the stepper motor drivers on the CNC shield
#define X_STEP_PIN 2
#define X_DIR_PIN 5
#define Y1_STEP_PIN 3
#define Y1_DIR_PIN 6
#define Y2_STEP_PIN 4
#define Y2_DIR_PIN 7

// Define the pin connected to the servo motor and limit switches
#define Z_SERVO_PIN A3 // Changed servo pin to A3
#define X_LIMIT_SWITCH_PIN 9
#define Y1_LIMIT_SWITCH_PIN 10
#define Y2_LIMIT_SWITCH_PIN 11

#define X_MAX 900
#define Y_MAX 450


// Define the coordinates for the home position
float homeX = 0;
float homeY = 0;

// Create instances of AccelStepper for X and Y axes
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY1(AccelStepper::DRIVER, Y1_STEP_PIN, Y1_DIR_PIN);
AccelStepper stepperY2(AccelStepper::DRIVER, Y2_STEP_PIN, Y2_DIR_PIN);

// Create instance of Servo for Z-axis
Servo servoZ;

void setup() {
  // Set the maximum speed and acceleration for X and Y axes
  stepperX.setMaxSpeed(MAX_SPEED_X);
  stepperX.setAcceleration(ACCELERATION_X);
  stepperY1.setMaxSpeed(MAX_SPEED_Y);
  stepperY1.setAcceleration(ACCELERATION_Y);
  stepperY2.setMaxSpeed(MAX_SPEED_Y);
  stepperY2.setAcceleration(ACCELERATION_Y);

  // Attach the servo to the pin
  servoZ.attach(Z_SERVO_PIN);

  // Initialize serial communication
  Serial.begin(9600);

  // Set the limit switch pins as inputs with pullup resistors
  pinMode(X_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(Y1_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(Y2_LIMIT_SWITCH_PIN, INPUT_PULLUP);

  // Move to home position
  moveGantryToHome();
}

void loop() {
  // Check if there is any data available on the serial port
  if (Serial.available() > 0) {
    // Read the input string until a newline character is received
    String input = Serial.readStringUntil('\n');

    // Check if the input is "RESET" and execute moveGantryToHome() function
    if (input == "R") {
      moveGantryToHome();
      Serial.println("Gantry moved to home position.");
    }
    else {
      // Parse the input string to extract X, Y coordinates, and the third parameter
      int commaIndex1 = input.indexOf(',');
      int commaIndex2 = input.indexOf(',', commaIndex1 + 1);
      if (commaIndex1 != -1 && commaIndex2 != -1) {
        // Extract X, Y coordinates, and the third parameter
        float xTarget = input.substring(0, commaIndex1).toFloat();
        float yTarget = input.substring(commaIndex1 + 1, commaIndex2).toFloat();
        int interval = input.substring(commaIndex2 + 1).toInt();

        if (xTarget <= X_MAX && yTarget <= Y_MAX){
          // Move the X and Y gantry to the target coordinates
          moveGantry(xTarget, yTarget);
          // Toggle the servo for the Z-axis
          if (interval != 0){
            toggleServo(interval);
          }

          Serial.println("OK");
        }else{
          Serial.println("Out of Range");
        }



      }
    }
  }
}

void moveGantry(float targetX, float targetY) {
  // Convert target coordinates from mm to steps
  long int targetPositionX = targetX * STEPS_PER_MM;
  long int targetPositionY1 = targetY * STEPS_PER_MM;
  long int targetPositionY2 = targetY * STEPS_PER_MM;

  // Move the X axis to the target position
  stepperX.moveTo(targetPositionX);

  // Move the Y axes to the target positions
  stepperY1.moveTo(targetPositionY1);
  stepperY2.moveTo(targetPositionY2);

  bool x = true;
  bool y1 = true;
  bool y2 = true;

  // Keep moving until all steppers reach their target positions
  while (x || y1 || y2) {
    x = stepperX.run();
    y1 = stepperY1.run();
    y2 = stepperY2.run();
    // You can add additional operations here if needed
  }
}

void toggleServo(int interval) {
  // Stop the servo (set speed to 0)
  servoZ.writeMicroseconds(1500); // Center position for a continuous rotation servo
  delay(100); // Delay to allow the servo to stop

  // Rotate the servo in one direction (clockwise)
  servoZ.writeMicroseconds(2000); // Adjust this value for the desired speed and direction
  delay(500); // Adjust this delay for the desired rotation duration

  // Stop the servo (set speed to 0)
  servoZ.writeMicroseconds(1500);
  delay(interval * 1000); // Delay based on the interval parameter

  servoZ.writeMicroseconds(1000); // Adjust this value for the desired speed and direction
  delay(500); // Adjust this delay for the desired rotation duration

  servoZ.writeMicroseconds(1500);
  delay(100); // Delay to allow the servo to stop
}

void moveGantryToHome() {
  // Move X axis to home position
  stepperX.setMaxSpeed(1000); // Slow speed for homing
  stepperX.setAcceleration(2000); // Slow acceleration for homing
  while (digitalRead(X_LIMIT_SWITCH_PIN) == HIGH) {
    stepperX.moveTo(-5550); // Move in negative direction until limit switch is hit
    stepperX.run();
    delay(1);
  }
  stepperX.setCurrentPosition(0); // Set current position as home position
  stepperX.setMaxSpeed(MAX_SPEED_X); // Restore maximum speed
  stepperX.setAcceleration(ACCELERATION_X); // Restore acceleration

  // Move Y axes to home position
  stepperY1.setMaxSpeed(1000); // Slow speed for homing
  stepperY1.setAcceleration(2000); // Slow acceleration for homing
  stepperY2.setMaxSpeed(1000); // Slow speed for homing
  stepperY2.setAcceleration(2000); // Slow acceleration for homing
  while (digitalRead(Y1_LIMIT_SWITCH_PIN) == HIGH||digitalRead(Y2_LIMIT_SWITCH_PIN) == HIGH) {
    stepperY1.moveTo(-2475); // Move in negative direction until limit switch is hit
    stepperY2.moveTo(-2475); // Move in negative direction until limit switch is hit
    if (digitalRead(Y1_LIMIT_SWITCH_PIN) == HIGH){
      stepperY1.run();
    }
    if (digitalRead(Y2_LIMIT_SWITCH_PIN) == HIGH){
      stepperY2.run();
    }
    delay(1);
  }
  stepperY1.setCurrentPosition(0); // Set current position as home position
  stepperY2.setCurrentPosition(0); // Set current position as home position
  stepperY1.setMaxSpeed(MAX_SPEED_Y); // Restore maximum speed
  stepperY1.setAcceleration(ACCELERATION_Y); // Restore acceleration
  stepperY2.setMaxSpeed(MAX_SPEED_Y); // Restore maximum speed
  stepperY2.setAcceleration(ACCELERATION_Y); // Restore acceleration

  // Set home position coordinates
  homeX = 0;
  homeY = 0;
}
