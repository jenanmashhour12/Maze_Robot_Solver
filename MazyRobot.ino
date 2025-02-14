#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>

// Motor Pins
#define IN1 13
#define IN2 12
#define IN3 14
#define IN4 27
#define ENA 26
#define ENB 25

//IR Sensor Pins
#define IR_LEFT 34
#define IR_RIGHT 35

//Encoder Pins
#define ENCA_A 4
#define ENCA_B 18
#define ENCB_A 19
#define ENCB_B 23

//LiDAR and MPU6050
VL53L0X lidar;
MPU6050 mpu;

// Maze Parameters
#define MAZE_SIZE 8
struct Cell {
  bool visited;
  bool walls[4];  
};

Cell maze[MAZE_SIZE][MAZE_SIZE];
int currentX = 0, currentY = 0;
int currentDir = 0;
bool mazeSolved = false;

//Encoder Variables
volatile long pulseCountA = 0;
volatile long pulseCountB = 0;

//PID  Correction Variables >> MPU6050
float yaw = 0.0;
float integralYaw = 0;
float previousYawError = 0;
unsigned long lastTime = 0;

const float KpYaw = 0.5, KiYaw = 0.01, KdYaw = 0.05;

//PID Speed Correction Variables (Encoders)
float integralSpeed = 0;
float previousSpeedError = 0;
const float KpSpeed = 0.3, KiSpeed = 0.01, KdSpeed = 0.02;
const int maxCorrection = 20;
const long PULSES_PER_90_DEGREE = 150; // Adjust based on your robot

// Function Prototypes
void exploreMaze();
void stopMotors();
void updateYaw();
void correctYaw();
void correctMotorSpeeds();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void updateWalls();
void moveToNextCell(int dir);
void moveStraight();
void rotateIfNoWall();
void IRAM_ATTR encoderAInterrupt();
void IRAM_ATTR encoderBInterrupt();

// Encoder Interrupt Functions
void IRAM_ATTR encoderAInterrupt() { pulseCountA += digitalRead(ENCA_B) ? 1 : -1; }
void IRAM_ATTR encoderBInterrupt() { pulseCountB += digitalRead(ENCB_B) ? 1 : -1; }

// Setup Function
void setup() {
  Serial.begin(115200);
  Wire.begin();

  //Motor Pins Setup
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  // IR Sensors Setup
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  //Encoder Setup
  pinMode(ENCA_A, INPUT_PULLUP); pinMode(ENCA_B, INPUT_PULLUP);
  pinMode(ENCB_A, INPUT_PULLUP); pinMode(ENCB_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA_A), encoderAInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_A), encoderBInterrupt, CHANGE);

  // Sensor Setup
  lidar.init();
  lidar.setTimeout(500);
  mpu.initialize();

  lastTime = millis(); // Initialize time for yaw correction

  // Initialize Maze Grid
  for (int i = 0; i < MAZE_SIZE; i++) {
    for (int j = 0; j < MAZE_SIZE; j++) {
      maze[i][j].visited = false;
      for (int k = 0; k < 4; k++) maze[i][j].walls[k] = false;
    }
  }

  Serial.println("Setup Complete");
}

// Loop Function
void loop() {
  if (!mazeSolved) {
    exploreMaze();
    mazeSolved = true;
  }

  //Apply PID Correction
  updateYaw();
  correctYaw();
  correctMotorSpeeds();

  stopMotors();
}

// MPU6050 Correction
void updateYaw() {
  int16_t gyroZ = mpu.getRotationZ();
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  yaw += (gyroZ / 131.0) * dt;

  Serial.print("Yaw coorection: "); Serial.println(yaw);
}

void correctYaw() {
  float yawError = yaw;
  integralYaw += yawError;
  float derivativeYaw = yawError - previousYawError;
  previousYawError = yawError;

  float correction = (KpYaw * yawError) + (KiYaw * integralYaw) + (KdYaw * derivativeYaw);
  correction = constrain(correction, -maxCorrection, maxCorrection);

  analogWrite(ENA, 100 - correction); // Adjust motor A speed
  analogWrite(ENB, 100 + correction); // Adjust motor B speed

  Serial.print("Yaw Correction: "); Serial.println(correction);
}

// Speed Correction Using Encoders
void correctMotorSpeeds() {
  float speedError = pulseCountA - pulseCountB;
  integralSpeed += speedError;
  float derivativeSpeed = speedError - previousSpeedError;
  previousSpeedError = speedError;

  float correction = (KpSpeed * speedError) + (KiSpeed * integralSpeed) + (KdSpeed * derivativeSpeed);
  correction = constrain(correction, -maxCorrection, maxCorrection);

  analogWrite(ENA, 100 - correction); // Adjust motor A speed
  analogWrite(ENB, 100 + correction); // Adjust motor B speed

  Serial.print("Speed Correction: "); Serial.println(correction);
}

// Maze Exploration
void exploreMaze() {
  maze[currentX][currentY].visited = true;
  updateWalls();

  for (int i = 0; i < 4; i++) {
    int nextDir = (currentDir + i) % 4;
    if (!maze[currentX][currentY].walls[nextDir]) {
      moveStraight();
      moveToNextCell(nextDir);
      exploreMaze();
      moveToNextCell((nextDir + 2) % 4);
    }
  }
}

// Motor Functions
void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, 100); // Motor A speed
  analogWrite(ENB, 100); // Motor B speed
  Serial.println("Moving Forward");
}

void moveBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 50); // Motor A speed
  analogWrite(ENB, 50); // Motor B speed
  Serial.println("Moving Backward");
}

void turnLeft() {
  // Reset encoder counts
  pulseCountA = 0;
  pulseCountB = 0;

  // Set motors to turn left
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); // Left motor forward
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); // Right motor backward
  analogWrite(ENA, 100); // Left motor speed
  analogWrite(ENB, 100); // Right motor speed

  // Wait until the robot turns 90 degrees
  while (abs(pulseCountA) < PULSES_PER_90_DEGREE || abs(pulseCountB) < PULSES_PER_90_DEGREE) {
    // Proportional control for precise turning
    float error = PULSES_PER_90_DEGREE - abs(pulseCountA);
    float correction = KpYaw * error;

    // Adjust motor speeds based on the error
    analogWrite(ENA, 100 - correction); // Slow down left motor
    analogWrite(ENB, 100 + correction); // Speed up right motor

    Serial.print("PulseCountA: "); Serial.println(pulseCountA);
    Serial.print("PulseCountB: "); Serial.println(pulseCountB);
    delay(10); // Small delay to avoid flooding the serial monitor
  }

  stopMotors(); // Stop motors after turning
  Serial.println("Turned Left");
}

void turnRight() {
  // Reset encoder counts
  pulseCountA = 0;
  pulseCountB = 0;

  // Set motors to turn right
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); // Left motor backward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); // Right motor forward
  analogWrite(ENA, 100); // Left motor speed
  analogWrite(ENB, 100); // Right motor speed

  // Wait until the robot turns 90 degrees
  while (abs(pulseCountA) < PULSES_PER_90_DEGREE || abs(pulseCountB) < PULSES_PER_90_DEGREE) {
    // Proportional control for precise turning
    float error = PULSES_PER_90_DEGREE - abs(pulseCountA);
    float correction = KpYaw * error;

    // Adjust motor speeds based on the error
    analogWrite(ENA, 100 + correction); // Speed up left motor
    analogWrite(ENB, 100 - correction); // Slow down right motor

    Serial.print("PulseCountA: "); Serial.println(pulseCountA);
    Serial.print("PulseCountB: "); Serial.println(pulseCountB);
    delay(10); // Small delay to avoid flooding the serial monitor
  }

  stopMotors(); // Stop motors after turning
  Serial.println("Turned Right");
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); // Motor A speed
  analogWrite(ENB, 0); // Motor B speed
  Serial.println("Motors Stopped");
}

// Wall Detection
void updateWalls() {
  int distance = lidar.readRangeSingleMillimeters() / 10;
  if (distance < 18) maze[currentX][currentY].walls[currentDir] = true;
  maze[currentX][currentY].walls[(currentDir + 3) % 4] = digitalRead(IR_LEFT);
  maze[currentX][currentY].walls[(currentDir + 1) % 4] = digitalRead(IR_RIGHT);
}

// Move to Next Cell
void moveToNextCell(int dir) {
  if (dir == 0) currentY++;
  else if (dir == 1) currentX++;
  else if (dir == 2) currentY--;
  else if (dir == 3) currentX--;
}

// Move Straight Using Encoders
void moveStraight() {
  long initialPulseCountA = pulseCountA;
  long initialPulseCountB = pulseCountB;

  while (abs(pulseCountA - initialPulseCountA) < 1000 && abs(pulseCountB - initialPulseCountB) < 1000) {
    moveForward(); //  Move forward while ensuring equal motor speed
    correctMotorSpeeds(); //  Maintain equal motor speeds using PID correction
  }
  stopMotors(); //  Stop motors after reaching the required distance
  Serial.println("Moved Straight");
}