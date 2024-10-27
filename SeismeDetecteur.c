#include <Wire.h>
#include <MPU6050.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

MPU6050 accel;
LiquidCrystal_I2C lcd(0x27, 16, 2);

const int buttonPin = 10;
const int redPin = 6;
const int greenPin = 7;
const int bluePin = 8;
const int buzzerPin = 9;

bool alert = false;
float threshold = 1.5;

int16_t ax, ay, az;

void setup() {
  initializePins();
  initializeDisplay();
  initializeSensor();
}

void loop() {
  float gForce = getGForce();
  float magnitude_richter = calculateMagnitude(gForce);

  Serial.println(gForce);

  if (gForce > threshold) {
    alert = true;
  }

  if (alert) {
    handleAlert(magnitude_richter);
  } else {
    displayNormalState();
  }

  delay(200);
}

void initializePins() {
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
}

void initializeDisplay() {
  lcd.init();
  lcd.backlight();
  Serial.begin(9600);
}

void initializeSensor() {
  Wire.begin();
  accel.initialize();

  if (accel.testConnection()) {
    displayMessage("MPU6050 OK", "Systeme pret");
  } else {
    displayMessage("MPU6050 ERREUR", "");
    while (1);
  }
  
  delay(2000);
  lcd.clear();
}

float getGForce() {
  accel.getAcceleration(&ax, &ay, &az);
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;
  return sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
}

float calculateMagnitude(float gForce) {
  float acceleration_magnitude = gForce * 9.81;
  return log10(acceleration_magnitude) + 0.3;
}

void handleAlert(float magnitude_richter) {
  triggerAlarm();
  displayAlertMessage(magnitude_richter);
  if (digitalRead(buttonPin) == LOW) {
    resetAlert();
  }
}

void triggerAlarm() {
  setLED(255, 0, 0);
  digitalWrite(buzzerPin, HIGH);
  delay(100);
  digitalWrite(buzzerPin, LOW);
}

void resetAlert() {
  alert = false;
  digitalWrite(buzzerPin, LOW);
  lcd.clear();
}

void setLED(int red, int green, int blue) {
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}

void displayMessage(const char* line1, const char* line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void displayNormalState() {
  setLED(0, 0, 255);
  displayMessage("Etat Normal", "");
}

void displayAlertMessage(float magnitude_richter) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Seisme Detecte");
  lcd.setCursor(0, 1);
  lcd.print("Richter: ");
  lcd.print(magnitude_richter, 2);
}
