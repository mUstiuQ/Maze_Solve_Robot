#include <L298NX2.h>


// Pinii motoarelor
const unsigned int EN_A = 3;
const unsigned int IN1_A = 12;
const unsigned int IN2_A = 11;


const unsigned int IN1_B = 10;
const unsigned int IN2_B = 9;
const unsigned int EN_B = 4;


// Ini?ializare motoare
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);


// Set?ri senzori (�n centimetri)
#define FRONT_THRESHOLD 15  // Prag pentru senzorul frontal
#define SIDE_THRESHOLD 5    // Prag pentru senzorii laterali
#define SAFE_DISTANCE 20    // Distan?? considerat? sigur?


// Timpi pentru manevre
const int TURN_TIME = 500;    // Timp pentru viraj de 90 de grade
const int UTURN_TIME = 1000;  // Timp pentru �ntoarcere de 180 de grade
const int BACK_TIME = 300;    // Timp pentru mers �napoi
const int THINK_TIME = 1000;  // Timp pentru "g�ndire" (1 secund?)


// Pinii senzorilor ultrasonic
const int trigPinf = 6;    // Senzor frontal
const int echopinf = A1;


const int trigPinr = 7;    // Senzor dreapta
const int echoPinr = A0;


const int trigPinl = 5;    // Senzor st�nga
const int echoPinl = A2;


// Viteze motoare
int speedA = 30;  // Vitez? motor A
int speedB = 20;  // Vitez? motor B


// Buffer pentru citiri senzori
#define NUM_READINGS 5
float frontReadings[NUM_READINGS];
float rightReadings[NUM_READINGS];
float leftReadings[NUM_READINGS];
int readingIndex = 0;


// -------------------------
// Func?ii utilitare
// -------------------------


float getMedian(float arr[], int size) {
  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        float temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
  return arr[size / 2];
}


float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
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
// Func?ii de mi?care
// -------------------------


void stopAndThink() {
  motors.stopA();
  motors.stopB();
  Serial.println("Oprit pentru g�ndire...");
  delay(THINK_TIME); // Pauz? de g�ndire
}


void goForward() {
  motors.setSpeedA(speedA);
  motors.setSpeedB(speedB);
  motors.forwardA();
  motors.forwardB();
}


void goBackward() {
  motors.setSpeedA(speedA);
  motors.setSpeedB(speedB);
  motors.backwardA();
  motors.backwardB();
  delay(BACK_TIME);
  motors.stopA();
  motors.stopB();
}


void turnRight() {
  stopAndThink();
  Serial.println("Execut viraj dreapta");
  motors.setSpeedA(speedA);
  motors.setSpeedB(speedB);
  motors.forwardA();
  motors.backwardB();
  delay(TURN_TIME);
  motors.stopA();
  motors.stopB();
}


void turnLeft() {
  stopAndThink();
  Serial.println("Execut viraj st�nga");
  motors.setSpeedA(speedA);
  motors.setSpeedB(speedB);
  motors.backwardA();
  motors.forwardB();
  delay(TURN_TIME);
  motors.stopA();
  motors.stopB();
}


void performUTurn() {
  stopAndThink();
  Serial.println("Execut �ntoarcere complet?");
  motors.setSpeedA(speedA);
  motors.setSpeedB(speedB);
  motors.forwardA();
  motors.backwardB();
  delay(UTURN_TIME);
  motors.stopA();
  motors.stopB();
}


// -------------------------
// Setup
// -------------------------
void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }
  
  pinMode(trigPinf, OUTPUT);
  pinMode(echopinf, INPUT);
  pinMode(trigPinr, OUTPUT);
  pinMode(echoPinr, INPUT);
  pinMode(trigPinl, OUTPUT);
  pinMode(echoPinl, INPUT);
  
  motors.setSpeedA(speedA);
  motors.setSpeedB(speedB);
  
  for (int i = 0; i < NUM_READINGS; i++) {
    updateSensorReadings();
    delay(50);
  }
  
  Serial.println("Robotul este gata!");
  Serial.println("Citiri senzori (cm): Fata | Dreapta | Stanga");
}


// -------------------------
// Bucla principal?
// -------------------------
void loop() {
  updateSensorReadings();
  
  float frontDist, rightDist, leftDist;
  getCurrentDistances(frontDist, rightDist, leftDist);
  
  Serial.print("Distan?e - F: ");
  Serial.print(frontDist);
  Serial.print(" | D: ");
  Serial.print(rightDist);
  Serial.print(" | S: ");
  Serial.print(leftDist);
  Serial.println(" cm");
  
  bool frontObstacle = frontDist <= FRONT_THRESHOLD;
  bool rightObstacle = rightDist <= SIDE_THRESHOLD;
  bool leftObstacle = leftDist <= SIDE_THRESHOLD;
  
  if (frontObstacle) {
    Serial.println("Obstacol detectat �n fa??!");
    
    // Decizie rut?
    if (!rightObstacle && rightDist > leftDist) {
      Serial.println("Am decis s? virez dreapta (mai mult spa?iu)");
      turnRight();
    } 
    else if (!leftObstacle) {
      Serial.println("Am decis s? virez st�nga");
      turnLeft();
    } 
    else if (!rightObstacle) {
      Serial.println("Am decis s? virez dreapta (ultima op?iune)");
      turnRight();
    }
    else {
      Serial.println("Toate c?ile blocate - mers �napoi ?i �ntoarcere");
      goBackward();
      performUTurn();
    }
  } 
  else {
    // Nicio obstruc?ie - mergem �nainte
    Serial.println("Drum liber - mergem �nainte");
    goForward();
  }
  
  delay(100); 
}