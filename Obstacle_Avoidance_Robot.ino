#include <Servo.h>          // Include standard library to control servo motors
#include <NewPing.h>        // Include library to simplify ultrasonic sensor usage

// L298N motor driver pin connections
const int LeftMotorForward = 6;    // Pin to drive left motor forward
const int LeftMotorBackward = 7;   // Pin to drive left motor backward
const int RightMotorForward = 5;   // Pin to drive right motor forward
const int RightMotorBackward = 4;  // Pin to drive right motor backward
const int enA = 8;                 // Enable pin for right motor (motor A)
const int enB = 3;                 // Enable pin for left motor (motor B)

// Ultrasonic sensor pins
#define trig_pin 11   // Trigger pin (sends ultrasonic pulse)
#define echo_pin 12   // Echo pin (receives reflected pulse)

// Maximum distance to detect with ultrasonic (in cm)
#define maximum_distance 200   

// Robot state variables
boolean goesForward = false;   // To remember if robot is already moving forward
int distance = 100;            // Initial measured distance (cm)

// Create ultrasonic object with trigger, echo, and max distance
NewPing sonar(trig_pin, echo_pin, maximum_distance);  

// Create servo object
Servo servo_motor; 

void setup(){

  // Set all motor control pins as output
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  
  // Attach servo to pin 9
  servo_motor.attach(9);

  // Position servo at "front" (115° angle, facing forward)
  servo_motor.write(180);
  delay(2000);  // Wait for servo to settle

  // Take a few initial distance readings to stabilize sensor
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
}

void loop(){

  int distanceRight = 0;   // Will store measured distance to right side
  int distanceLeft = 0;    // Will store measured distance to left side
  delay(50);               // Short delay between loops
  
  // If obstacle is detected within 15 cm
  if (distance <= 15){
    moveStop();       // Stop moving
    delay(300);
    moveBackward();   // Move backward slightly
    delay(300);
    moveStop();       // Stop again
    delay(300);

    // Look right and measure distance
    distanceRight = lookRight();
    delay(300);

    // Look left and measure distance
    distanceLeft = lookLeft();
    delay(300);

    // Compare distances and turn toward the side with more space
    if (distanceRight >= distanceLeft){
      turnRight();
      moveStop();
    }
    else{
      turnLeft();
      moveStop();
    }
  }
  else{
    moveForward();   // If path is clear, keep moving forward
  }
  
  // Update distance in front
  distance = readPing();
}

// Function to check distance on the right side
int lookRight(){  
  servo_motor.write(50);     // Rotate servo to the right (50°)
  delay(300);                // Allow servo to reach position
  int distance = readPing(); // Measure distance
  delay(100);
  servo_motor.write(115);    // Return servo to forward position
  return distance;           // Return measured distance
}

// Function to check distance on the left side
int lookLeft(){
  servo_motor.write(170);    // Rotate servo to the left (170°)
  delay(300);                
  int distance = readPing(); // Measure distance
  delay(100);
  servo_motor.write(115);    // Return servo to forward
  return distance;           
  delay(100);                // (This delay after return is useless, never executed)
}

// Function to measure distance using ultrasonic sensor
int readPing(){
  delay(70);                 // Short delay between readings
  int cm = sonar.ping_cm();  // Measure distance in centimeters
  if (cm == 0){              // If no obstacle detected (out of range)
    cm = 250;                // Set large default distance
  }
  return cm;                 // Return measured distance
}

// Stop both motors
void moveStop(){
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(enA, HIGH);  // Enable motor driver
  digitalWrite(enB, HIGH);
}

// Move forward
void moveForward(){
  if(!goesForward){   // Only set pins if not already moving forward
    goesForward = true;
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW); 
    digitalWrite(enA, HIGH);
    digitalWrite(enB, HIGH);
  }
}

// Move backward
void moveBackward(){
  goesForward = false;   // Update state
  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(enA, HIGH);
  digitalWrite(enB, HIGH);
}

// Turn right
void turnRight(){
  digitalWrite(LeftMotorForward, HIGH);    // Left motor forward
  digitalWrite(RightMotorBackward, HIGH);  // Right motor backward (pivot turn)
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(enA, HIGH);
  digitalWrite(enB, HIGH);
  
  delay(300);   // Turn for some time
  
  // Move forward again after turning
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(enA, HIGH);
  digitalWrite(enB, HIGH);
}

// Turn left
void turnLeft(){
  digitalWrite(LeftMotorBackward, HIGH);   // Left motor backward
  digitalWrite(RightMotorForward, HIGH);   // Right motor forward (pivot turn)
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(enA, HIGH);
  digitalWrite(enB, HIGH);

  delay(300);   // Turn for some time
  
  // Move forward again after turning
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(enA, HIGH);
  digitalWrite(enB, HIGH);
}
