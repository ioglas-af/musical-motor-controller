#include "pitches.h"  // Make sure this library is available, if needed

// Pin definitions for motor and electromagnet control
#define VA 18   // Encoder A pin
#define VB 31   // Encoder B pin
#define I1 34   // Motor direction pin 1
#define I2 35   // Motor direction pin 2
#define PWM 12  // Motor speed control (PWM pin)

#define electromagnetPin A0  // Electromagnet control pin
#define LED 13  // Indicator LED for incoming data

// PID control constants
#define KP 0.05
#define KI 0.0005
#define KD 0.00005
#define MAX_PWM 150
#define MAX_INTEGRAL 500
#define DEAD_ZONE 2.0

// Motor and electromagnet variables
volatile long pulseCount = 0;
volatile int direction = 1;
volatile float currentAngle = 0;
const int pulsesPerRevolution = 1024;
const float degreesPerPulse = 360.0 / pulsesPerRevolution;
const int totalPetals = 7;
const float degreesPerPetal = 360.0 / totalPetals;
int selectedPetal = 0;
float targetAngle = 0;

// PID error control variables
float errorIntegral = 0;
float previousError = 0;
float errorDerivative = 0;
unsigned long lastTime = 0;

// Electromagnet control
bool electromagnetActive = true;  // Starts active (HIGH level)
bool electromagnetOnCycle = false;
unsigned long noteSelectionTime = 0;
unsigned long electromagnetDeactivationTime = 0;

// Note mapping
int gamme[] = {NOTE_D4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_F5, NOTE_G5, NOTE_A5, NOTE_B5, NOTE_C6};
int incomingByte;
int note;

// Function prototypes
void encoderHandlerA();
void encoderHandlerB();
void controlElectromagnet();
void moveToTargetAngle();
void MotorControl(float controlSignal);

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);  // Communication with Raspberry Pi
  pinMode(LED, OUTPUT);

  Serial.println("System ready! Waiting for note from Raspberry Pi.");

  // Pin setup for motor and electromagnet
  pinMode(VA, INPUT_PULLUP);
  pinMode(VB, INPUT_PULLUP);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(electromagnetPin, OUTPUT);
  digitalWrite(electromagnetPin, HIGH);  // Start with electromagnet ON

  // Encoder interrupts
  attachInterrupt(digitalPinToInterrupt(VA), encoderHandlerA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(VB), encoderHandlerB, CHANGE);

  lastTime = millis();  // Initialize time tracking for `dt` calculation
}

void loop() {
  // Check if a note was received from Raspberry Pi
  if (Serial2.available() > 0) {
    incomingByte = Serial2.read();
    Serial.print("Note received: ");
    Serial.println(char(incomingByte));

    // Convert the received character (musical note) to an index from 0 to 13
    if (incomingByte >= 'A' && incomingByte <= 'Z') {
      note = incomingByte - 'A';
      if (note < totalPetals) {  // Ensure the index is within valid range
        selectedPetal = note + 1;  // Adjust position for petals (1-7)
        targetAngle = (selectedPetal - 1) * degreesPerPetal * 6.2;

        // Update note selection time and prepare electromagnet cycle
        noteSelectionTime = millis();
        electromagnetOnCycle = false;
        electromagnetActive = true;
        digitalWrite(electromagnetPin, HIGH);  // Activate electromagnet
        digitalWrite(LED, HIGH);  // Indicate data received
        delay(100);
        digitalWrite(LED, LOW);
      } else {
        Serial.println("Note out of range.");
      }
    }
  }

  // Control motor to reach the target angle
  moveToTargetAngle();

  // Check if it's time to deactivate or reactivate the electromagnet
  controlElectromagnet();
}

// Time-based control for the electromagnet
void controlElectromagnet() {
  // Deactivate electromagnet 2 seconds after note selection
  if (!electromagnetOnCycle && millis() - noteSelectionTime >= 2000) {
    digitalWrite(electromagnetPin, LOW);  // Turn OFF
    electromagnetOnCycle = true;
    electromagnetDeactivationTime = millis();
    Serial.println("Electromagnet deactivated.");
  }
  // Reactivate electromagnet 1 second after deactivation
  else if (electromagnetOnCycle && millis() - electromagnetDeactivationTime >= 1000) {
    digitalWrite(electromagnetPin, HIGH);  // Turn ON
    electromagnetOnCycle = false;
    Serial.println("Electromagnet reactivated.");
  }
}

// Encoder interrupt handlers
void encoderHandlerA() {
  int aState = digitalRead(VA);
  int bState = digitalRead(VB);
  direction = (aState == bState) ? 1 : -1;
  pulseCount += direction;
  currentAngle = pulseCount * degreesPerPulse;
}

void encoderHandlerB() {
  int aState = digitalRead(VA);
  int bState = digitalRead(VB);
  direction = (aState == bState) ? 1 : -1;
  pulseCount += direction;
  currentAngle = pulseCount * degreesPerPulse;
}

// Move motor to the desired angle
void moveToTargetAngle() {
  float error = targetAngle - currentAngle;
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  if (dt <= 0.0) return;

  // Check if error is within dead zone
  if (abs(error) <= DEAD_ZONE) {
    MotorControl(0);
    return;
  }

  // Update integral term with clamping
  errorIntegral += error * dt;
  errorIntegral = constrain(errorIntegral, -MAX_INTEGRAL, MAX_INTEGRAL);

  // Compute smoothed derivative term
  float rawDerivative = (error - previousError) / dt;
  errorDerivative = 0.8 * errorDerivative + 0.2 * rawDerivative;
  previousError = error;

  // Compute PID control signal
  float controlSignal = KP * error + KI * errorIntegral + KD * errorDerivative;
  MotorControl(controlSignal);
}

// Motor speed and direction control with PID
void MotorControl(float controlSignal) {
  controlSignal = constrain(controlSignal, -9.0, 9.0);

  if (controlSignal > 0) {
    digitalWrite(I2, LOW);
    digitalWrite(I1, HIGH);
  } else {
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
  }

  int pwmValue = map(abs(controlSignal), 0, 9, 0, MAX_PWM);
  analogWrite(PWM, pwmValue);
}
