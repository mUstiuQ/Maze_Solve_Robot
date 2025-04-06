#include <L298NX2.h>
#include <StackArray.h>

// Motor Pins
const unsigned int EN_A = 3;
const unsigned int IN1_A = 12;
const unsigned int IN2_A = 11;
const unsigned int IN1_B = 10;
const unsigned int IN2_B = 9;
const unsigned int EN_B = 4;

// Initialize motors
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

// Sensor settings (cm)
#define FRONT_THRESHOLD 15
#define SIDE_THRESHOLD 5
#define SAFE_DISTANCE 20

// Movement times
const int TURN_TIME = 500;    // 90° turn
const int UTURN_TIME = 1000;  // 180° turn
const int BACK_TIME = 300;
const int THINK_TIME = 1000;

// Ultrasonic Sensor Pins
const int trigPinf = 6, echopinf = A1;
const int trigPinr = 7, echoPinr = A0;
const int trigPinl = 5, echoPinl = A2;

// Motor speeds
int speedA = 120, speedB = 100;

// Mapping system
#define GRID_SIZE 10
int robotX = GRID_SIZE/2, robotY = GRID_SIZE/2; // Start in middle
int robotDir = 0; // 0=up, 1=right, 2=down, 3=left
byte worldMap[GRID_SIZE][GRID_SIZE]; // 0=unknown, 1=empty, 2=obstacle

// Pathfinding
StackArray<int> pathStack;

// Sensor readings buffer
#define NUM_READINGS 5
float frontReadings[NUM_READINGS], rightReadings[NUM_READINGS], leftReadings[NUM_READINGS];
int readingIndex = 0;

// -------------------------
// Utility Functions
// -------------------------

float getMedian(float arr[], int size) {
  for (int i = 0; i < size-1; i++) {
    for (int j = 0; j < size-i-1; j++) {
      if (arr[j] > arr[j+1]) {
        float temp = arr[j];
        arr[j] = arr[j+1];
        arr[j+1] = temp;
      }
    }
  }
  return arr[size/2];
}

float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) * 0.034 / 2;
}

void updateSensorReadings() {
  frontReadings[readingIndex] = measureDistance(trigPinf, echopinf);
  rightReadings[readingIndex] = measureDistance(trigPinr, echoPinr);
  leftReadings[readingIndex] = measureDistance(trigPinl, echoPinl);
  readingIndex = (readingIndex + 1) % NUM_READINGS;
}

void getCurrentDistances(float &front, float &right, float &left) {
  front = getMedian(frontReadings, NUM_READINGS);
  right = getMedian(rightReadings, NUM_READINGS);
  left = getMedian(leftReadings, NUM_READINGS);
}

// -------------------------
// Movement Functions
// -------------------------

void stopAndThink() {
  motors.stopA();
  motors.stopB();
  delay(THINK_TIME);
}

void goForward() {
  motors.setSpeedA(speedA);
  motors.setSpeedB(speedB);
  motors.forwardA();
  motors.forwardB();
  
  // Update position based on direction
  switch(robotDir) {
    case 0: robotY--; break;
    case 1: robotX++; break;
    case 2: robotY++; break;
    case 3: robotX--; break;
  }
  
  // Mark current position as empty
  if(robotX >= 0 && robotX < GRID_SIZE && robotY >= 0 && robotY < GRID_SIZE) {
    worldMap[robotX][robotY] = 1;
  }
}

void turnRight() {
  stopAndThink();
  motors.setSpeedA(speedA);
  motors.setSpeedB(speedB);
  motors.forwardA();
  motors.backwardB();
  delay(TURN_TIME);
  motors.stopA();
  motors.stopB();
  robotDir = (robotDir + 1) % 4;
}

void turnLeft() {
  stopAndThink();
  motors.setSpeedA(speedA);
  motors.setSpeedB(speedB);
  motors.backwardA();
  motors.forwardB();
  delay(TURN_TIME);
  motors.stopA();
  motors.stopB();
  robotDir = (robotDir + 3) % 4; // +3 is same as -1 mod 4
}

// -------------------------
// Mapping & Pathfinding
// -------------------------

void updateMap(float frontDist, float rightDist, float leftDist) {
  // Mark obstacles in front
  if(frontDist < FRONT_THRESHOLD) {
    int obsX = robotX, obsY = robotY;
    switch(robotDir) {
      case 0: obsY--; break;
      case 1: obsX++; break;
      case 2: obsY++; break;
      case 3: obsX--; break;
    }
    if(obsX >= 0 && obsX < GRID_SIZE && obsY >= 0 && obsY < GRID_SIZE) {
      worldMap[obsX][obsY] = 2;
    }
  }
  
  // Mark obstacles to the right
  if(rightDist < SIDE_THRESHOLD) {
    int obsX = robotX, obsY = robotY;
    switch(robotDir) {
      case 0: obsX++; break;
      case 1: obsY++; break;
      case 2: obsX--; break;
      case 3: obsY--; break;
    }
    if(obsX >= 0 && obsX < GRID_SIZE && obsY >= 0 && obsY < GRID_SIZE) {
      worldMap[obsX][obsY] = 2;
    }
  }
  
  // Mark obstacles to the left
  if(leftDist < SIDE_THRESHOLD) {
    int obsX = robotX, obsY = robotY;
    switch(robotDir) {
      case 0: obsX--; break;
      case 1: obsY--; break;
      case 2: obsX++; break;
      case 3: obsY++; break;
    }
    if(obsX >= 0 && obsX < GRID_SIZE && obsY >= 0 && obsY < GRID_SIZE) {
      worldMap[obsX][obsY] = 2;
    }
  }
}

bool findPath(int targetX, int targetY) {
  // Simple BFS pathfinding
  int cameFrom[GRID_SIZE][GRID_SIZE];
  for(int x=0; x<GRID_SIZE; x++) {
    for(int y=0; y<GRID_SIZE; y++) {
      cameFrom[x][y] = -1;
    }
  }
  
  StackArray<int> queue;
  queue.push(robotX);
  queue.push(robotY);
  cameFrom[robotX][robotY] = robotDir;
  
  while(!queue.isEmpty()) {
    int x = queue.pop();
    int y = queue.pop();
    
    if(x == targetX && y == targetY) {
      // Reconstruct path
      pathStack.clear();
      while(x != robotX || y != robotY) {
        int dir = cameFrom[x][y];
        pathStack.push(dir);
        // Move backward
        switch(dir) {
          case 0: y++; break;
          case 1: x--; break;
          case 2: y--; break;
          case 3: x++; break;
        }
      }
      return true;
    }
    
    // Check neighbors
    int dx[] = {0, 1, 0, -1};
    int dy[] = {-1, 0, 1, 0};
    for(int i=0; i<4; i++) {
      int nx = x + dx[i];
      int ny = y + dy[i];
      if(nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE && 
         worldMap[nx][ny] != 2 && cameFrom[nx][ny] == -1) {
        cameFrom[nx][ny] = i;
        queue.push(nx);
        queue.push(ny);
      }
    }
  }
  
  return false; // No path found
}

void followPath() {
  while(!pathStack.isEmpty()) {
    int nextDir = pathStack.pop();
    
    // Turn to face correct direction
    while(robotDir != nextDir) {
      turnRight();
    }
    
    // Move forward
    goForward();
    delay(500); // Small pause between moves
  }
}

// -------------------------
// Setup & Main Loop
// -------------------------

void setup() {
  Serial.begin(9600);
  while(!Serial);
  
  pinMode(trigPinf, OUTPUT);
  pinMode(echopinf, INPUT);
  pinMode(trigPinr, OUTPUT);
  pinMode(echoPinr, INPUT);
  pinMode(trigPinl, OUTPUT);
  pinMode(echoPinl, INPUT);
  
  motors.setSpeedA(speedA);
  motors.setSpeedB(speedB);
  
  // Initialize map with unknown spaces
  for(int x=0; x<GRID_SIZE; x++) {
    for(int y=0; y<GRID_SIZE; y++) {
      worldMap[x][y] = 0;
    }
  }
  
  // Mark starting position as empty
  worldMap[robotX][robotY] = 1;
  
  Serial.println("Robot initialized with mapping system");
}

void loop() {
  updateSensorReadings();
  
  float frontDist, rightDist, leftDist;
  getCurrentDistances(frontDist, rightDist, leftDist);
  
  // Update map with new sensor data
  updateMap(frontDist, rightDist, leftDist);
  
  // Print current status
  Serial.print("Pos: ("); Serial.print(robotX); 
  Serial.print(","); Serial.print(robotY);
  Serial.print(") Facing: "); Serial.println(robotDir);
  Serial.print("Distances - F:"); Serial.print(frontDist);
  Serial.print(" R:"); Serial.print(rightDist);
  Serial.print(" L:"); Serial.println(leftDist);
  
  // Check for obstacles
  if(frontDist <= FRONT_THRESHOLD) {
    Serial.println("Obstacle detected! Calculating path...");
    
    // Find nearest unexplored area
    int targetX = -1, targetY = -1;
    for(int y=0; y<GRID_SIZE && targetX==-1; y++) {
      for(int x=0; x<GRID_SIZE; x++) {
        if(worldMap[x][y] == 0) { // Unknown area
          targetX = x;
          targetY = y;
          break;
        }
      }
    }
    
    if(targetX != -1 && findPath(targetX, targetY)) {
      Serial.println("Path found! Following...");
      followPath();
    } else {
      Serial.println("No path found! Turning...");
      turnRight();
    }
  } else {
    // No obstacle - move forward
    goForward();
    delay(500);
  }
  
  delay(100);
}