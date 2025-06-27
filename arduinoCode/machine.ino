// Libraries
#include <SPI.h>
#include <MFRC522.h>
#include <MFRC522Extended.h>
#include <deprecated.h>
#include <require_cpp11.h>
#include <Servo.h> 
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

// Arduino pins

// Step 1 --> Customer Decision (LCD and Keypad)

// I2C settinngs (address, columns #, rows #)
LiquidCrystal_I2C lcd(0x27, 16, 2); 
// Keypad settings 
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
// Rows (A7 - A6) --> R1 with A7 --- Columns (A3 - A0) --> C1 with A3
byte rowPins[ROWS] = {A7,A6,A5,A4};
byte colPins[COLS] = {A3,A2,A1,A0};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Variables
String selectedPackage = ""; 

// --------------------------------------------------------------------------------------
// Step 2 --> Payment Process Via RFID

// Blue -> UID:  F3 B5 2A 1B
// White -> UID:  84 76 CE 72

#define SS_PIN 53 
#define RST_PIN 11

MFRC522 mfrc522(SS_PIN, RST_PIN);

// UIDs of allowed cards
byte blueCardUID[] = {0xF3, 0xB5, 0x2A, 0x1B};
byte whiteCardUID[] = {0x84, 0x76, 0xCE, 0x72};

int price = 0;

// --------------------------------------------------------------------------------------
// Step 3 --> Heating Oil Process

const int oilRelayPin = 4;

// ReHeating Duration - 3 min
unsigned long reHeatingDuration = 180000;

bool isOilHeating = false;
unsigned long oilHeatStartTime = 0;
const unsigned long oilHeatDuration = 1800000UL; // 30 دقيقة


// --------------------------------------------------------------------------------------
// Step 4 --> Push and Cut Process

// Pin 2 for sensor
const int sensorPin = 2;
// Pin 6 -> pushing piston relay CH1
// Pin 7 -> cutting piston relay CH2
const int pressPistonPin = 5;
const int cutterPistonPin = 6;

// Variables
int targetCount = 0; // Based on choosen package --> S: 2, M: 4, L: 6 
int pieceCount = 0; // Counter
bool lastSensorState = HIGH;

// --------------------------------------------------------------------------------------
// Step 5 --> Frying Process

// Basket Motor Pins
const int basketPinRPWM = 7;
const int basketPinLPWM = 8;
const int basketPinR_EN = 9;
const int basketPinL_EN = 10;

// Basket Motor Speed (0-255)
int basketMotorSpeed = 60;

// Frying Duration - 3 min
unsigned long fryingDuration = 180000;

// Stages: 
// INITIAL POSITION -> FRYING POSITION -> TILTED POSITION -BOX ARRIVED- -> WAIT FOR THE BASKET TO EMPTY -> BACK TO INITIAL POSITION

// --------------------------------------------------------------------------------------
// Step 6 --> Convayer Belt Movement

// Convayer Belt Motor Pins
const int ConvayerBeltPinRPWM = 22;
const int ConvayerBeltPinLPWM = 23;
const int ConvayerBeltPinR_EN = 24;
const int ConvayerBeltPinL_EN = 25;

// Convayer Belt Motor Speed (0-255)
int ConvayerBeltMotorSpeed = 200;

bool isBeltMovingRight = false;

// --------------------------------------------------------------------------------------
// Laser And LDR Modules Pins

// Laser -Signal- Pins - Digital
const int laserBoxPin = 26;      // Carton Box Dispenser Sensor
const int laserChurrosPin = 27;  // Box Filling Sensor
const int laserSugarPin = 28;    // Sugar And Cinammon Sprinkling Sensor
const int laserSaucePin = 29;    // Sause Dispenser Sensor

// LDR Pins -Analog-
const int ldrBoxPin = A8;       // Carton Box Dispenser Sensor
const int ldrChurrosPin = A9;   // Box Filling Sensor
const int ldrSugarPin = A10;    // Sugar And Cinammon Sprinkling Sensor
const int ldrSaucePin = A11;    // Sause Dispenser Sensor

const int threshold = 130;

// --------------------------------------------------------------------------------------
// Stage 1 --> Carton Box Dispenser

// Carton Box Dispenser Stepper Motor -Nema 23- Driver Pins
const int stepPin = 30;
const int dirPin = 31;
const int enPin = 32;

const float stepsPerRev = 1600.0; 
const float stepsPerDegree = stepsPerRev / 360.0; 

// Rotation Angle = 75; 

bool boxDropped = false;

// --------------------------------------------------------------------------------------
// Stage 2 --> Box Filling -Frying Basket Fliping-

// --------------------------------------------------------------------------------------
// Stage 3 --> Sugar And Cinammon Sprinkling

// Sugar And Cinammon Sprinkling Servo Motor Pins
Servo sprinklingServo;
int sprinklingServoPin = 33;
const int TOTAL_SPRINKLES = 4;

// --------------------------------------------------------------------------------------
// Stage 4 --> Sause Dispenser

// Sauce dispensing Servo Motor Pins
Servo sauseDispenserServo; 
int sauseDispenserServoPin = 34;

// --------------------------------------------------------------------------------------
// Machine States
enum State {
  WAIT_FOR_POWER_ON,
  WELCOME,
  SELECT_PACKAGE,
  PAYMENT,
  HEAT_OIL,
  PUSH_CUT,
  // FROM FRYING
  INIT,
  // Frying process States -Before Box-
  BASKET_INITIAL_POSITION,
  FRYING_POSITION,
  // Convayer Belt States
  WAITING_FOR_BOX,
  // Frying process States -After Box-
  // Moving the belt to the right
  MOVING_RIGHT_TO_FILL,
  // Change basket position and Wait until basket to empty
  FILLING_CHURROS,
  // Move basket to initial position
  BASKET_BACK_TO_INITIAL,
  // Moving the belt to the left
  MOVING_LEFT_TO_SUGAR,
  ADDING_SUGAR,
  MOVING_LEFT_TO_SAUCE,
  ADDING_SAUCE,
  DONE
};

State currentState = WAIT_FOR_POWER_ON;

void setup() {
  // Step 1 --> Customer decision -- LCD set up
  lcd.init();
  lcd.backlight();

  // Step 2 --> Payment set up
  SPI.begin();
  mfrc522.PCD_Init();

  // Step 3 --> Heating Oil -- relay set up
  pinMode(oilRelayPin, OUTPUT);

  // Step 4 --> Push and Cut -- relay set up
  pinMode(sensorPin, INPUT);
  pinMode(cutterPistonPin, OUTPUT);
  pinMode(pressPistonPin, OUTPUT);
  
  digitalWrite(cutterPistonPin, HIGH);
  digitalWrite(pressPistonPin, HIGH);
  digitalWrite(oilRelayPin, HIGH);

  // Step 5 --> Frying -- Basket motor driver set up
  pinMode(basketPinRPWM, OUTPUT);
  pinMode(basketPinLPWM, OUTPUT);
  pinMode(basketPinR_EN, OUTPUT);
  pinMode(basketPinL_EN, OUTPUT);
  digitalWrite(basketPinR_EN, HIGH);
  digitalWrite(basketPinL_EN, HIGH);

  // Step 6 --> Convayer belt motor driver set up
  pinMode(ConvayerBeltPinRPWM, OUTPUT);
  pinMode(ConvayerBeltPinLPWM, OUTPUT);
  pinMode(ConvayerBeltPinR_EN, OUTPUT);
  pinMode(ConvayerBeltPinL_EN, OUTPUT);
  digitalWrite(ConvayerBeltPinR_EN, HIGH);
  digitalWrite(ConvayerBeltPinL_EN, HIGH);

  // Laser and LDR modules set up
  pinMode(laserBoxPin, OUTPUT);
  digitalWrite(laserBoxPin, HIGH);
  pinMode(laserChurrosPin, OUTPUT);
  digitalWrite(laserChurrosPin, HIGH);
  pinMode(laserSugarPin, OUTPUT);
  digitalWrite(laserSugarPin, HIGH);
  pinMode(laserSaucePin, OUTPUT);
  digitalWrite(laserSaucePin, HIGH);

  // Stage 1 --> Carton box dispenser -Stepper motor nema 23 driver- set up
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(enPin,OUTPUT);
  digitalWrite(enPin,LOW);

  // Stage 2 --> Box filling -frying basket fliping- 

  // Stage 3 --> Sugar and cinammon sprinkling -Servo motor- set up
  sprinklingServo.attach(sprinklingServoPin);
  sprinklingServo.write(0); // initial position
  delay(1000);
  sprinklingServo.detach();

  // Stage 4 --> Chocolate sauce dispenser -Servo motor- set up
  sauseDispenserServo.attach(sauseDispenserServoPin);
  sauseDispenserServo.write(80); // initial position    
  delay(1000);
  sauseDispenserServo.detach();

  // Serial
  Serial.begin(9600);
}

// READING LDR SENSOR TO AVOID DISRUPTION
int readSensor(int pin) {
  int total = 0;
  for (int i = 0; i < 10; i++) {
    total += analogRead(pin);
    delay(5);
  }
  return total / 10;
}

void loop() {
  // From website
  checkSerialCommand();

  int boxSensor = readSensor(ldrBoxPin);
  int churrosSensor = readSensor(ldrChurrosPin);
  int sugarSensor = readSensor(ldrSugarPin);
  int sauceSensor = readSensor(ldrSaucePin);

  switch (currentState) {
    case WAIT_FOR_POWER_ON:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("A: Power ON");
      lcd.setCursor(0, 1);
      lcd.print("B: Heat Oil");

      while (true) {
        char key = keypad.getKey();

        if (key == 'A') {
          Serial.println("✅ Power ON pressed.");
          lcd.clear();
          lcd.print("System Starting...");
          delay(2000);
          currentState = WELCOME;
          break;
        } else if (key == 'B' && !isOilHeating) {
          Serial.println("🔥 Heating oil for 30 minutes...");
          lcd.clear();
          lcd.print("Heating Oil...");
          digitalWrite(oilRelayPin, LOW);
          isOilHeating = true;
          oilHeatStartTime = millis();
        }

        // فحص إذا انتهى وقت التسخين
        if (isOilHeating && millis() - oilHeatStartTime >= oilHeatDuration) {
          digitalWrite(oilRelayPin, HIGH);
          isOilHeating = false;
          Serial.println("✅ Oil heating complete.");
          lcd.clear();
          lcd.print("Oil Heated Done");
          lcd.setCursor(0, 1);
          lcd.print("Press A to start");
        }
      }
      break;

    case WELCOME:
      welcomeMessage();
      currentState = SELECT_PACKAGE;
      break;

    case SELECT_PACKAGE:
      choosePackage();
      currentState = PAYMENT;
      break;

    case PAYMENT:
      processPayment();
      currentState = HEAT_OIL;
      break;

    case HEAT_OIL:
      heatOil();
      currentState = PUSH_CUT;
      break;

    case PUSH_CUT:
      pushCut();
      currentState = INIT;
      break;

    case INIT: 
      // 4 sec before starting
      // Serial.println("🟢 Starting systen in 4 sec...");
      // delay(4000);
      // currentState = BASKET_INITIAL_POSITION;
      // break;
      Serial.println("🟡 Waiting for 'S' to start...");
      while (true) {
        if (Serial.available() > 0) {
          char incomingChar = Serial.read();
          if (incomingChar == 'S' || incomingChar == 's') {
            Serial.println("🟢 Starting system in 4 sec...");
            delay(4000);
            
             // 🚨 اختبار مباشر لحركة السير عكس عقارب الساعة
              // ConvayerBeltMoveCCW(ConvayerBeltMotorSpeed);
              // delay(3000);
              // ConvayerBeltStopMotor();

            //currentState = BASKET_INITIAL_POSITION;
            currentState = FRYING_POSITION;
            break;
          }
        }
      }
      break;

    case BASKET_INITIAL_POSITION:
      Serial.println("🟢BASKET_INITIAL_POSITION Start frying in 1.5 sec...");
      delay(1500);
      currentState = FRYING_POSITION;
      break;

    case FRYING_POSITION:
      //Serial.println("🟢 Start frying in 1.5 sec...");
      fryingLoop(basketMotorSpeed, fryingDuration);
      currentState = WAITING_FOR_BOX;
      break;

    case WAITING_FOR_BOX:
      //Serial.print("box LDR before: ");
      //Serial.println(boxSensor);
      if (!boxDropped) {
        Serial.println("⬇️ Box dispensing...");
        dropBox();
        boxDropped = true;
        //digitalWrite(laserBoxPin, LOW);
      }
      //Serial.print("box LDR after: ");
      //Serial.println(boxSensor);
      if (boxSensor > 130) {
        Serial.println("🟢 New Box Dispensed...");
        currentState = MOVING_RIGHT_TO_FILL;
      }
      break;

    case MOVING_RIGHT_TO_FILL:
      Serial.println("➡️ Moving Belt To Churros Filling...");
      ConvayerBeltMoveCW(ConvayerBeltMotorSpeed);
      Serial.print("🔍 churrosSensor BEFORE: ");
      Serial.println(churrosSensor);
      while (true) {
        churrosSensor = readSensor(ldrChurrosPin);
        if (churrosSensor > 320) {
          Serial.print("🔍 churrosSensor AFTER: ");
          Serial.println(churrosSensor);
          ConvayerBeltStopMotor();
          Serial.println("🍩 Box Arrived Under Frying Basket...");
          currentState = FILLING_CHURROS;
          break;
        }
        delay(200); // انتظار صغير لمنع القراءة السريعة جداً
      }
      break;

    case FILLING_CHURROS:
      // BASKET TO TILTED POSITION -BOX ARRIVED-
      moveBasketCCW(basketMotorSpeed);
      delay(5000);
      // WAIT FOR THE BASKET TO EMPTY
      stopBasketMotor();
      delay(5000);
      Serial.println("✅ Filling Churros Done Successfully...");
      currentState = BASKET_BACK_TO_INITIAL;
      break;

    case BASKET_BACK_TO_INITIAL:
      moveBasketCW(basketMotorSpeed);
      delay(2850);
      stopBasketMotor();
      currentState = MOVING_LEFT_TO_SUGAR;
      break;

    case MOVING_LEFT_TO_SUGAR:
      ConvayerBeltMoveCCW(ConvayerBeltMotorSpeed);
      Serial.print("🔍 sugarSensor BEFORE: ");
      Serial.println(sugarSensor);
      //delay(5000);
      // if (sugarSensor > 100) {
      //   Serial.print("🔍 sugarSensor AFTER: ");
      // Serial.println(sugarSensor);
      //   ConvayerBeltStopMotor();
      //   Serial.println("🍩 Box Arrived To Start Sprinkling...");
      //   //currentState = ADDING_SUGAR;
      //   currentState = DONE;
      // }
      while (true) {
        sugarSensor = readSensor(ldrSugarPin);
        if (sugarSensor > 320) {
          Serial.print("🔍 sugarSensor AFTER: ");
          Serial.println(sugarSensor);
          ConvayerBeltStopMotor();
          Serial.println("🍩 Box Arrived Under Sugar...");
          currentState = ADDING_SUGAR;
          break;
        }
        delay(200); // انتظار صغير لمنع القراءة السريعة جداً
      }
      break;

    case ADDING_SUGAR:
      sprinkling(sprinklingServoPin);
      Serial.println("✅ Sprinkling Done Successfully...");
      currentState = MOVING_LEFT_TO_SAUCE;
      break;

    case MOVING_LEFT_TO_SAUCE:
      ConvayerBeltMoveCCW(ConvayerBeltMotorSpeed);
      if (sauceSensor > threshold) {
        ConvayerBeltStopMotor();
        Serial.println("🍩 Box Arrived To Sause Dispenser...");
        currentState = ADDING_SAUCE;
      }
      break;

    case ADDING_SAUCE:
      sauceDispensing(sauseDispenserServoPin);
      Serial.println("✅ Sauce Dispensing Done Successfully...");
      currentState = DONE;
      break;

    case DONE:
      lcd.setCursor(0, 0);
      lcd.print("Process Done!   ");
      lcd.setCursor(0, 1);
      lcd.print("Enjoy your food!");
      delay(5000);
      //digitalWrite(pressPistonPin, LOW);
      while (true); // إنهاء البرنامج
      break;

    case FINISHED:
      lcd.setCursor(0, 0);
      lcd.print("Process Done!   ");
      lcd.setCursor(0, 1);
      lcd.print("Enjoy your food!");
      delay(5000);
      //digitalWrite(pressPistonPin, LOW);
      while (true); // إنهاء البرنامج
      break;
  }
}

// --------------------------------------------------------------------------------------
// SHOW WELCOME MESSAGE
void welcomeMessage() {
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("WELCOME TO");
  lcd.setCursor(0, 1);
  lcd.print("CHURROS DELIGHT");
  delay(2000);
  lcd.clear();
}

// --------------------------------------------------------------------------------------
// CHOOSE PACKAGE PROCESS

// Using Keypad
void choosePackage() {
  bool chosen = false;
  while (!chosen) {
    lcd.setCursor(0, 0);
    lcd.print("Choose Package:");
    lcd.setCursor(0, 1);
    lcd.print("1:S 2:M 3:L   ");

    char key = keypad.getKey();
    if (key) {
      lcd.clear();
      if (key == '1') {
        selectedPackage = "Small";
        price = 15; // Small package price
        targetCount = 2; // Small package peices #
        chosen = true;
      } else if (key == '2') {
        selectedPackage = "Medium";
        price = 25; // Medium package price
        targetCount = 4; // Medium package peices #
        chosen = true;
      } else if (key == '3') {
        selectedPackage = "Large";
        price = 35; // Large package peices #
        targetCount = 6; // Large package peices #
        chosen = true;
      } else {
        lcd.print("Invalid Input");
        delay(1000);
        lcd.clear();
      }
    }
  }

  lcd.setCursor(0, 0);
  lcd.print("You chose:");
  lcd.setCursor(0, 1);
  lcd.print(selectedPackage);
  delay(2000);
  lcd.clear();
}

// Using Serial
void checkSerialCommand() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    Serial.print("📥 Received via Serial: ");
    Serial.println(cmd);

    if (cmd == 'S') {
      selectedPackage = "Small";
      targetCount = 2;
      price = 15;
      currentState = PAYMENT;
    } else if (cmd == 'M') {
      selectedPackage = "Medium";
      targetCount = 4;
      price = 25;
      currentState = PAYMENT;
    } else if (cmd == 'L') {
      selectedPackage = "Large";
      targetCount = 6;
      price = 35;
      currentState = PAYMENT;
    }
  }
}

// --------------------------------------------------------------------------------------
// PAYMENT PROCESS
void processPayment() {
  lcd.setCursor(0, 0);
  lcd.print("Price: ");
  lcd.print(price);
  lcd.print(" NIS");

  lcd.setCursor(0, 1);
  lcd.print("Scan your card");

  bool paid = false;
  while (!paid) {
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
      if (isCardAuthorized(mfrc522.uid.uidByte)) {
        lcd.clear();
        lcd.print("Payment Done");
        lcd.setCursor(0, 1);
        lcd.print("Thank You!");
        paid = true;
      } else {
        lcd.clear();
        lcd.print("Unauthorized");
        lcd.setCursor(0, 1);
        lcd.print("Try again");
        delay(2000);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Price: ");
        lcd.print(price);
        lcd.print(" NIS");
        lcd.setCursor(0, 1);
        lcd.print("Scan your card");
      }
      mfrc522.PICC_HaltA(); // Stop reading
      mfrc522.PCD_StopCrypto1();
    }
  }

  delay(2000);
  lcd.clear();
}

// CHECK CARD AVAILABILITY
bool isCardAuthorized(byte *uid) {
  // BLUE ACRD UID
  if (memcmp(uid, blueCardUID, 4) == 0) {
    Serial.println("✅ Blue card accepted");
    return true;
  }
  // WHITE CARD UID
  if (memcmp(uid, whiteCardUID, 4) == 0) {
    Serial.println("✅ White card accepted");
    return true;
  }

  Serial.println("❌ Unauthorized card");
  return false;
}

// --------------------------------------------------------------------------------------
// HEATING OIL PROCESS
void heatOil() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Heating oil...");
  lcd.setCursor(0, 1);
  lcd.print("Please wait");

  reHeatingLoop(reHeatingDuration);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Oil is ready!");
  delay(2000);
  lcd.clear();

  // For -- Cutting Process --
  pieceCount = 0;
  digitalWrite(pressPistonPin, HIGH);
  Serial.println("🔵 Pressure piston activated.");
}

void reHeatingLoop(unsigned long duration) {
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    digitalWrite(oilRelayPin, LOW);
  }
  digitalWrite(oilRelayPin, HIGH);
}

// --------------------------------------------------------------------------------------
// PUSH AND CUT PROCESS
void pushCut() {
  bool currentSensorState = digitalRead(sensorPin);

  if (pieceCount >= targetCount) {
    Serial.println("🛑 Target reached.");
    //currentState = FINISHED;
    return;
  }

  if (lastSensorState == HIGH && currentSensorState == LOW) {
    pieceCount++;
    Serial.print("📦 Piece detected. Count = ");
    Serial.println(pieceCount);

    // Stop pushing piston
    digitalWrite(pressPistonPin, LOW);
    Serial.println("🔴 Pressure piston deactivated.");

    // Cutting started
    digitalWrite(cutterPistonPin, HIGH);
    Serial.println("✂️  Cutter piston activated.");
    delay(500);
    digitalWrite(cutterPistonPin, LOW);
    Serial.println("✅ Cutter piston deactivated.");

    if (pieceCount < targetCount) {
      digitalWrite(pressPistonPin, HIGH);
      Serial.println("🔵 Pressure piston re-activated.");
    }
  }
  lastSensorState = currentSensorState;
}

// --------------------------------------------------------------------------------------
// FRYING BASKET -MOTOR- DIRECTIONS FUNCTIONS 
void fryingLoop(int speed, unsigned long duration) {
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    moveBasketCCW(speed);
    delay(2500);
    moveBasketCW(speed);
    delay(2000);
  }

  // BASKET TO HORIZONTAL POSITION
  moveBasketCCW(basketMotorSpeed);
  delay(5000);
  stopBasketMotor();
}

void moveBasketCW(int speed) {
  analogWrite(basketPinRPWM, speed);
  analogWrite(basketPinLPWM, 0);
}

void moveBasketCCW(int speed) {
  analogWrite(basketPinRPWM, 0);
  analogWrite(basketPinLPWM, speed);
}

void stopBasketMotor() {
  analogWrite(basketPinRPWM, 0);
  analogWrite(basketPinLPWM, 0);
}

// --------------------------------------------------------------------------------------
// CONVAYER BELT -MOTOR- DIRECTIONS FUNCTIONS 
void ConvayerBeltMoveCW(int speed) {
  analogWrite(ConvayerBeltPinRPWM, speed);
  analogWrite(ConvayerBeltPinLPWM, 0);
}

void ConvayerBeltMoveCCW(int speed) {
  analogWrite(ConvayerBeltPinRPWM, 0);
  analogWrite(ConvayerBeltPinLPWM, speed);
}

void ConvayerBeltStopMotor() {
  analogWrite(ConvayerBeltPinRPWM, 0);
  analogWrite(ConvayerBeltPinLPWM, 0);
}

// --------------------------------------------------------------------------------------
// CARTON BOX DISPENSER
void dropBox() {
  // rotate 75 cw 
  rotateStepper(75, true);
  delay(1000);
   // rotate 75 ccw 
  rotateStepper(70, false);
  delay(1000);
}

// STEPPER MOTOR NEMA 23 ROTATION
void rotateStepper(int angle, bool direction) {
  int steps = angle * stepsPerDegree;
  digitalWrite(dirPin, direction ? HIGH : LOW);
  for (int x = 0; x < steps; x++) {
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(500); 
    digitalWrite(stepPin, LOW); 
    delayMicroseconds(500); 
  }
}

// --------------------------------------------------------------------------------------
// SPRINKLING SUGAR AND CINAMMON
void sprinkling(int servopin) { 
  Serial.println("🚀 4 Sprinkles...");
  sprinklingServo.attach(servopin);
  for (int i = 0; i < TOTAL_SPRINKLES; i++) {
    sprinkleOnce();
  }
  returnToStart();
  sprinklingServo.detach();
}

void sprinkleOnce() {
  sprinklingServo.write(45);  
  delay(400);
  sprinklingServo.write(75);    
  delay(400);
  sprinklingServo.write(45);    // رجعة
  delay(400);
  Serial.println("✨ One Sprinkle Done.");
}

void returnToStart() {
  sprinklingServo.write(0);
  delay(500);
}

// --------------------------------------------------------------------------------------
// SAUCE DISPENSING
void sauceDispensing(int servopin) { 
  sauseDispenserServo.attach(servopin);
  delay(1);
  sauseDispenserServo.write(0);  
  delay(3000);       
  sauseDispenserServo.write(80);    
  delay(1000);
  sauseDispenserServo.detach();
}
