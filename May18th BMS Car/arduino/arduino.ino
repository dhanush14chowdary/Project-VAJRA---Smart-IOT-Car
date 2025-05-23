#include <SoftwareSerial.h>

// Create a software serial port on pins 8 (RX) and 12 (TX)
SoftwareSerial BoltSerial(8, 12); 

// Ultrasonic sensor pins
#define TRIG_PIN 11       // Ultrasonic Trigger Pin
#define ECHO_PIN 10       // Ultrasonic Echo Pin

// Define motor driver pins
#define ENA 9
#define IN1 7
#define IN2 6
#define ENB 3
#define IN3 5
#define IN4 4

// LDR, LED, and buzzer pins
#define LDR_PIN A0        // LDR on analog pin A0
#define RED_LED_1 A1      // Red LED 1 (back) on analog pin A1
#define RED_LED_2 A2      // Red LED 2 (back) on analog pin A2
#define WHITE_LED A5      // White LED (front) on analog pin A5
#define BUZZER A4         // Buzzer on analog pin A4

// Distance threshold for auto-braking and buzzer (in cm)
#define OBSTACLE_DISTANCE 15

// LDR threshold for lights
#define DARK_THRESHOLD 100

// Driving mode speeds (PWM values)
#define STRADA_SPEED 150
#define SPORT_SPEED 200
#define CORSA_SPEED 255

// PWM values for red LEDs
#define RED_DIM 100       // Dim brightness for dark/manual/movement
#define RED_FULL 255      // Full brightness for stop/backward

// Variables to store the last command, current mode, and states
String lastCommand = "STOP";
String currentMode = "STRADA";
int currentSpeed = STRADA_SPEED; // Default to Strada
bool lightsManual = false;        // Manual light control
bool lightsOn = false;           // White LED state
bool redLightsOn = false;        // Red LED state
int redBrightness = 0;           // Current red LED PWM
bool buzzerActive = false;       // Auto-braking buzzer state
bool hornActive = false;         // Horn state
bool wasMovingForward = false;   // Track forward movement
bool buzzerTriggered = false;    // Track buzzer trigger state

// Timing for status updates, buzzer, and red LEDs
unsigned long lastStatusTime = 0;
const unsigned long statusInterval = 2000; // 2 seconds
unsigned long buzzerStartTime = 0;
const unsigned long buzzerDuration = 1000; // 1 second
unsigned long redLightStartTime = 0;
const unsigned long redLightStopDuration = 1000; // 1 second for stop

void setup() {
  Serial.begin(9600);         // Serial monitor
  BoltSerial.begin(9600);     // RX/TX with ESP32

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // LDR, LED, and buzzer pins
  pinMode(LDR_PIN, INPUT);
  pinMode(RED_LED_1, OUTPUT);
  pinMode(RED_LED_2, OUTPUT);
  pinMode(WHITE_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  // Set initial speed (Strada)
  analogWrite(ENA, currentSpeed);
  analogWrite(ENB, currentSpeed);

  // Initialize LEDs and buzzer off
  analogWrite(RED_LED_1, 0);
  analogWrite(RED_LED_2, 0);
  digitalWrite(WHITE_LED, LOW);
  digitalWrite(BUZZER, LOW);
}

void setSpeed(int targetSpeed, String mode) {
  if (mode == "STRADA") {
    for (int speed = 0; speed <= targetSpeed; speed += 3) {
      analogWrite(ENA, speed);
      analogWrite(ENB, speed);
      delay(100); // 5 seconds
    }
  } else if (mode == "SPORT") {
    for (int speed = 0; speed <= targetSpeed; speed += 7) {
      analogWrite(ENA, speed);
      analogWrite(ENB, speed);
      delay(100); // ~3 seconds
    }
  } else {
    analogWrite(ENA, targetSpeed);
    analogWrite(ENB, targetSpeed);
  }
  currentSpeed = targetSpeed;
  Serial.println("Set mode: " + mode + ", speed: " + String(currentSpeed));
}

void setLights(bool state) {
  if (state) {
    digitalWrite(WHITE_LED, HIGH);
    lightsOn = true;
    // Red LEDs at dim brightness unless in BACKWARD or stop
    if (lastCommand != "BACKWARD" && (!redLightsOn || redBrightness != RED_DIM)) {
      analogWrite(RED_LED_1, RED_DIM);
      analogWrite(RED_LED_2, RED_DIM);
      redLightsOn = true;
      redBrightness = RED_DIM;
      Serial.println("Lights: ON (White FULL, Red DIM at 100)");
    }
  } else {
    digitalWrite(WHITE_LED, LOW);
    lightsOn = false;
    // Red LEDs off unless needed for stop/backward/movement
    if (!redLightsOn || (lastCommand != "BACKWARD" && lastCommand != "FORWARD" && 
         lastCommand != "LEFT" && lastCommand != "RIGHT" && redBrightness != RED_FULL)) {
      analogWrite(RED_LED_1, 0);
      analogWrite(RED_LED_2, 0);
      redLightsOn = false;
      redBrightness = 0;
      Serial.println("Lights: OFF (White and Red)");
    }
  }
}

void setRedLights(bool state, int brightness) {
  if (state && (!redLightsOn || redBrightness != brightness)) {
    analogWrite(RED_LED_1, brightness);
    analogWrite(RED_LED_2, brightness);
    redLightsOn = true;
    redBrightness = brightness;
    redLightStartTime = millis();
    Serial.println("Red Lights: ON, Brightness: " + String(brightness));
  } else if (!state && redLightsOn && !lightsOn && 
             lastCommand != "BACKWARD" && lastCommand != "FORWARD" && 
             lastCommand != "LEFT" && lastCommand != "RIGHT") {
    analogWrite(RED_LED_1, 0);
    analogWrite(RED_LED_2, 0);
    redLightsOn = false;
    redBrightness = 0;
    Serial.println("Red Lights: OFF");
  }
}

void moveForward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  wasMovingForward = true;
  setRedLights(true, RED_DIM); // Red LEDs at PWM 100
}

void moveBackward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  wasMovingForward = false;
  setRedLights(true, RED_FULL); // Red LEDs at full brightness
}

void turnLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  wasMovingForward = false;
  setRedLights(true, RED_DIM); // Red LEDs at PWM 100
}

void turnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  wasMovingForward = false;
  setRedLights(true, RED_DIM); // Red LEDs at PWM 100
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  if (wasMovingForward) {
    setRedLights(true, RED_FULL); // Red LEDs at full brightness for 1 second
    wasMovingForward = false;
  }
}

// Function to measure distance using ultrasonic sensor
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  
  if (duration == 0) {
    return -1;
  }
  
  float distance = (duration * 0.0343) / 2;
  
  if (distance > 400 || distance < 2) {
    return -1;
  }
  
  return distance;
}

// Function to print status every 2 seconds
void printStatus() {
  unsigned long currentTime = millis();
  if (currentTime - lastStatusTime >= statusInterval) {
    Serial.println("=== Status Update ===");
    Serial.println("All Modes:");
    Serial.println("- Strada: " + String(STRADA_SPEED) + " PWM");
    Serial.println("- Sport: " + String(SPORT_SPEED) + " PWM");
    Serial.println("- Corsa: " + String(CORSA_SPEED) + " PWM");
    Serial.println("Current Mode: " + currentMode + ", Speed: " + String(currentSpeed) + " PWM");
    
    float distance = getDistance();
    if (distance == -1) {
      Serial.println("Distance: Error (No echo or invalid)");
    } else {
      Serial.println("Distance: " + String(distance, 1) + " cm");
    }
    
    int ldrValue = analogRead(LDR_PIN);
    Serial.println("LDR Value: " + String(ldrValue) + " (Dark < " + String(DARK_THRESHOLD) + ")");
    Serial.println("Lights: " + String(lightsOn ? "ON" : "OFF") + (lightsManual ? " (Manual)" : " (Auto)"));
    Serial.println("Red Lights: " + String(redLightsOn ? "ON" : "OFF") + ", Brightness: " + String(redBrightness));
    Serial.println("Buzzer/Horn: " + String(buzzerActive || hornActive ? "ON" : "OFF") + (hornActive ? " (Horn)" : ""));
    
    Serial.println("====================");
    lastStatusTime = currentTime;
  }
}

void loop() {
  // Print status every 2 seconds
  printStatus();

  // Check distance to obstacle
  float distance = getDistance();
  bool obstacleDetected = (distance <= OBSTACLE_DISTANCE && distance > 0);

  // Auto-light control (if not in manual mode)
  int ldrValue = analogRead(LDR_PIN);
  if (!lightsManual) {
    if (ldrValue < DARK_THRESHOLD && !lightsOn) {
      setLights(true); // Red LEDs at PWM 100
    } else if (ldrValue >= DARK_THRESHOLD && lightsOn) {
      setLights(false);
    }
  }

  // Turn off buzzer after 1 second
  if (buzzerActive && !hornActive && (millis() - buzzerStartTime >= buzzerDuration)) {
    digitalWrite(BUZZER, LOW);
    buzzerActive = false;
    buzzerTriggered = false;
    Serial.println("Buzzer: OFF (1 second elapsed)");
  }

  // Handle auto-braking and buzzer during FORWARD movement
  if (lastCommand == "FORWARD" && wasMovingForward) {
    if (obstacleDetected) {
      if (digitalRead(IN1) == LOW && digitalRead(IN2) == HIGH && 
          digitalRead(IN3) == HIGH && digitalRead(IN4) == LOW) {
        Serial.println("Auto-braking: Stopping due to obstacle");
        stopMotors();
        setRedLights(true, RED_FULL);
        if (!buzzerActive && !hornActive && !buzzerTriggered) {
          digitalWrite(BUZZER, HIGH);
          buzzerActive = true;
          buzzerTriggered = true;
          buzzerStartTime = millis();
          Serial.println("Buzzer: ON (Auto-braking)");
        }
      }
    } else {
      moveForward();
      buzzerTriggered = false; // Reset when obstacle clears
    }
  }

  // Turn off red LEDs after 1 second for sudden stop
  if (redLightsOn && !lightsOn && lastCommand != "BACKWARD" && 
      lastCommand != "FORWARD" && lastCommand != "LEFT" && lastCommand != "RIGHT" && 
      (millis() - redLightStartTime >= redLightStopDuration) && redBrightness == RED_FULL) {
    setRedLights(false, 0);
  }

  // Handle new commands from ESP32
  if (BoltSerial.available()) {
    String command = BoltSerial.readStringUntil('\n');
    command.trim();
    Serial.println("Received command: " + command);
    lastCommand = command;

    if (command == "STRADA") {
      currentMode = "STRADA";
      setSpeed(STRADA_SPEED, currentMode);
    } else if (command == "SPORT") {
      currentMode = "SPORT";
      setSpeed(SPORT_SPEED, currentMode);
    } else if (command == "CORSA") {
      currentMode = "CORSA";
      setSpeed(CORSA_SPEED, currentMode);
    } else if (command == "LIGHTS_ON") {
      lightsManual = true;
      setLights(true);
    } else if (command == "LIGHTS_OFF") {
      lightsManual = true;
      setLights(false);
    } else if (command == "HORN_ON") {
      if (!hornActive) {
        digitalWrite(BUZZER, HIGH);
        hornActive = true;
        buzzerActive = false;
        buzzerTriggered = false;
        Serial.println("Horn: ON");
      }
    } else if (command == "HORN_OFF") {
      if (hornActive) {
        digitalWrite(BUZZER, LOW);
        hornActive = false;
        Serial.println("Buzzer: OFF (Horn stopped)");
      }
    } else if (command == "FORWARD") {
      if (obstacleDetected) {
        Serial.println("Cannot move forward: Obstacle within 15cm");
        stopMotors();
      } else {
        moveForward();
      }
    } else if (command == "BACKWARD") {
      moveBackward();
    } else if (command == "LEFT") {
      turnLeft();
    } else if (command == "RIGHT") {
      turnRight();
    } else if (command == "STOP") {
      stopMotors();
    }
  }

  delay(100);
}
