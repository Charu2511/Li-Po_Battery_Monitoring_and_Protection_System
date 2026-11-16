#include <Arduino.h>
#include <avr/interrupt.h>
#include <LiquidCrystal.h>
const int voltagePin = A0;
const int currentPin = A1;

const int relayPin = 7;
const int pwmPin   = 4;

const int redPin   = 2;
const int greenPin = 3;
const int bluePin  = 5;

const byte lcdRsPin = 34;
const byte lcdEnPin = 35;
const byte lcdD4Pin = 36;
const byte lcdD5Pin = 37;
const byte lcdD6Pin = 38;
const byte lcdD7Pin = 39;

LiquidCrystal lcd(lcdRsPin, lcdEnPin, lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin);

// Relay states: active LOW means LOW = load connected, HIGH = load disconnected.
const bool RELAY_ACTIVE_LOW = true;

int RELAY_LOAD_CONNECTED;
int RELAY_LOAD_DISCONNECTED;

const float ARDUINO_ADC_REF = 4.96;

const float REF_INPUT_VOLTAGE  = 11.84;
const float REF_OUTPUT_VOLTAGE = 2.33;
const float SENSOR_RATIO = REF_INPUT_VOLTAGE / REF_OUTPUT_VOLTAGE;

const float ACS712_SENSITIVITY = 0.066;

const float ZERO_CURRENT_THRESHOLD_A = 0.25;

const float CURRENT_SMOOTHING_FACTOR = 0.85;

float zeroCurrentVoltage = 2.45;
float smoothedCurrent = 0.0;

// Current ranges: <0.25 A = OFF; 0.25-0.60 A = GREEN; 0.60-0.80 A = YELLOW; 0.80-1.00 A = ORANGE; 1.00-1.30 A = slow RED blink; >=1.30 A = cutoff with fast RED blink.
const float RATED_CURRENT  = 1.0;
const float CUTOFF_CURRENT = 1.3;

const float GREEN_LIMIT  = 0.60 * RATED_CURRENT;
const float YELLOW_LIMIT = 0.80 * RATED_CURRENT;
const float ORANGE_LIMIT = 1.00 * RATED_CURRENT;

bool cutoffLatched = false;

const float CAPACITY_AH = 1.55;

// Battery-capacity state: disconnect the load at 1.364 Ah used (approximately 12% SOC remaining).
const float RELAY_OPEN_USED_AH = 1.364;
const float LOW_CAPACITY_USED_AH = 1.364;

// Reconnection state: with near-zero current, reconnect only when battery voltage is above 11.00 V.
const float ZERO_CURRENT_LOAD_ENABLE_VOLTAGE = 11.00;

// SOC display range for LM3914: 12% SOC = PWM 0; 100% SOC = PWM 255.
const float PWM_SOC_MIN = 12.0;
const float PWM_SOC_MAX = 100.0;

const byte segE  = 23;
const byte segD  = 22;
const byte segDP = 25;
const byte segC  = 24;
const byte segG  = 27;
const byte digit4 = 26;

const byte segB  = 29;
const byte digit3 = 28;
const byte digit2 = 31;
const byte segF  = 30;
const byte segA  = 33;
const byte digit1 = 32;

const byte segmentPins[7] = {
  segA, segB, segC, segD, segE, segF, segG
};

const byte digitPins[4] = {
  digit1, digit2, digit3, digit4
};

const byte DIGIT_PATTERN[10] = {
  0b00111111,
  0b00000110,
  0b01011011,
  0b01001111,
  0b01100110,
  0b01101101,
  0b01111101,
  0b00000111,
  0b01111111,
  0b01101111
};

volatile byte displayDigits[4] = {0, 0, 0, 0};
volatile bool displayDP[4] = {false, false, false, false};
volatile byte activeDigit = 0;

const int C_COLS = 5;
const int AH_ROWS = 11;

// C-rate lookup range: 0.2 C to 5.0 C.
const float C_COLUMNS[C_COLS] = {
  0.2,
  0.5,
  1.0,
  2.0,
  5.0
};

const float USED_AH_ROWS[AH_ROWS] = {
  0.000,
  0.155,
  0.310,
  0.465,
  0.620,
  0.775,
  0.930,
  1.085,
  1.240,
  1.395,
  1.550
};

const float VOLTAGE_TABLE[AH_ROWS][C_COLS] = {
  {12.56, 12.54, 12.50, 12.43, 12.22},
  {12.29, 12.27, 12.23, 12.16, 11.95},
  {11.99, 11.97, 11.93, 11.86, 11.65},
  {11.78, 11.76, 11.72, 11.65, 11.44},
  {11.60, 11.58, 11.54, 11.47, 11.26},
  {11.45, 11.43, 11.39, 11.32, 11.11},
  {11.33, 11.31, 11.27, 11.20, 10.99},
  {11.21, 11.19, 11.15, 11.08, 10.87},
  {11.09, 11.07, 11.03, 10.96, 10.75},
  {10.85, 10.83, 10.79, 10.72, 10.51},
  { 9.89,  9.87,  9.83,  9.76,  9.55}
};

float batteryVoltage = 0.0;
float currentA = 0.0;
float cRate = 0.0;

float usedAh = 0.0;
float soc = 0.0;

int pwmValue = 0;
float pwmApproxVoltage = 0.0;

bool relayClosed = false;
bool lowCapacityWarning = false;
bool chargeBatteryWarning = false;
bool zeroCurrentMode = false;

float powerW = 0.0;
float totalEnergyWh = 0.0;
unsigned long previousEnergyMillis = 0;
unsigned long long elapsedTimeMs = 0;

const unsigned long LCD_PAGE_INTERVAL_MS = 2000;
unsigned long lastLCDPageMillis = 0;
byte lcdPage = 0;

float floatAbs(float value) {
  if (value < 0.0) return -value;
  return value;
}

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

float linearInterpolate(float x, float x0, float x1, float y0, float y1) {
  if (x1 == x0) {
    return y0;
  }

  float ratio = (x - x0) / (x1 - x0);
  return y0 + ratio * (y1 - y0);
}

void setRGB(int red, int green, int blue) {
  red   = constrain(red, 0, 255);
  green = constrain(green, 0, 255);
  blue  = constrain(blue, 0, 255);

  analogWrite(redPin,   255 - red);
  analogWrite(greenPin, 255 - green);
  analogWrite(bluePin,  255 - blue);
}

void greenLED() {
  setRGB(0, 255, 0);
}

void yellowLED() {
  setRGB(255, 180, 0);
}

void orangeLED() {
  setRGB(255, 50, 0);
}

void redLED() {
  setRGB(255, 0, 0);
}

void blueLED() {
  setRGB(0, 0, 255);
}

void ledOff() {
  setRGB(0, 0, 0);
}

void redBlinkSlow() {
  redLED();
  delay(250);
  ledOff();
  delay(250);
}

void redBlinkFast() {
  redLED();
  delay(80);
  ledOff();
  delay(80);
}

void writeSegment(byte pin, bool on) {

  digitalWrite(pin, on ? LOW : HIGH);
}

void writeDigit(byte pin, bool on) {

  digitalWrite(pin, on ? HIGH : LOW);
}

void turnOffAllDigits() {
  for (byte i = 0; i < 4; i++) {
    writeDigit(digitPins[i], false);
  }
}

void clearAllSegments() {
  for (byte i = 0; i < 7; i++) {
    writeSegment(segmentPins[i], false);
  }

  writeSegment(segDP, false);
}

void refreshSevenSegmentDisplay() {
  turnOffAllDigits();
  clearAllSegments();

  byte number = displayDigits[activeDigit];
  byte pattern = DIGIT_PATTERN[number];

  for (byte i = 0; i < 7; i++) {
    bool segmentOn = pattern & (1 << i);
    writeSegment(segmentPins[i], segmentOn);
  }

  writeSegment(segDP, displayDP[activeDigit]);

  writeDigit(digitPins[activeDigit], true);

  activeDigit++;

  if (activeDigit >= 4) {
    activeDigit = 0;
  }
}

ISR(TIMER1_COMPA_vect) {
  refreshSevenSegmentDisplay();
}

void setSOCDisplay(float socPercent) {
  float displaySOC = socPercent;

  if (displaySOC < 0.0) {
    displaySOC = 0.0;
  }

  if (displaySOC > 99.99) {
    displaySOC = 99.99;
  }

  int scaledSOC = (int)((displaySOC * 100.0) + 0.5);

  byte d0 = (scaledSOC / 1000) % 10;
  byte d1 = (scaledSOC / 100)  % 10;
  byte d2 = (scaledSOC / 10)   % 10;
  byte d3 = scaledSOC % 10;

  noInterrupts();

  displayDigits[0] = d0;
  displayDigits[1] = d1;
  displayDigits[2] = d2;
  displayDigits[3] = d3;

  displayDP[0] = false;
  displayDP[1] = true;
  displayDP[2] = false;
  displayDP[3] = false;

  interrupts();
}

void setupSevenSegmentDisplay() {
  pinMode(segA, OUTPUT);
  pinMode(segB, OUTPUT);
  pinMode(segC, OUTPUT);
  pinMode(segD, OUTPUT);
  pinMode(segE, OUTPUT);
  pinMode(segF, OUTPUT);
  pinMode(segG, OUTPUT);
  pinMode(segDP, OUTPUT);

  pinMode(digit1, OUTPUT);
  pinMode(digit2, OUTPUT);
  pinMode(digit3, OUTPUT);
  pinMode(digit4, OUTPUT);

  turnOffAllDigits();
  clearAllSegments();

  setSOCDisplay(0.00);

  noInterrupts();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 499;

  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);

  interrupts();
}

void relayOpen() {
  digitalWrite(relayPin, RELAY_LOAD_DISCONNECTED);
  relayClosed = false;
}

void relayClose() {

  if (cutoffLatched) {
    relayOpen();
    return;
  }

  digitalWrite(relayPin, RELAY_LOAD_CONNECTED);
  relayClosed = true;
}

float readAverageADC(int pin, int samples) {
  long sum = 0;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(500);
  }

  return sum / (float)samples;
}

float readBatteryVoltage() {
  float adcValue = readAverageADC(voltagePin, 100);

  float sensorOutputVoltage = adcValue * ARDUINO_ADC_REF / 1023.0;
  float inputVoltage = sensorOutputVoltage * SENSOR_RATIO;

  return inputVoltage;
}

float calibrateZeroCurrent() {
  float adcValue = readAverageADC(currentPin, 1000);
  float voltage = adcValue * ARDUINO_ADC_REF / 1023.0;
  return voltage;
}

float readCurrentRaw() {
  float adcValue = readAverageADC(currentPin, 300);

  float sensorVoltage = adcValue * ARDUINO_ADC_REF / 1023.0;
  float current = (sensorVoltage - zeroCurrentVoltage) / ACS712_SENSITIVITY;

  return current;
}

float readCurrentFiltered() {
  float current = readCurrentRaw();

  if (current < 0.0) {
    current = -current;
  }

  if (current < ZERO_CURRENT_THRESHOLD_A) {
    smoothedCurrent = 0.0;
    return 0.0;
  }

  smoothedCurrent =
    (CURRENT_SMOOTHING_FACTOR * smoothedCurrent) +
    ((1.0 - CURRENT_SMOOTHING_FACTOR) * current);

  return smoothedCurrent;
}

float calculateCRate(float currentA) {
  float currentAbs = floatAbs(currentA);

  float calculatedCRate = currentAbs / CAPACITY_AH;

  calculatedCRate = clampFloat(calculatedCRate, 0.2, 5.0);

  return calculatedCRate;
}

void findCRateBounds(float cRate, int &lowCol, int &highCol) {
  int lastIndex = C_COLS - 1;

  if (cRate <= C_COLUMNS[0]) {
    lowCol = 0;
    highCol = 0;
    return;
  }

  if (cRate >= C_COLUMNS[lastIndex]) {
    lowCol = lastIndex;
    highCol = lastIndex;
    return;
  }

  for (int i = 0; i < lastIndex; i++) {
    float cLow = C_COLUMNS[i];
    float cHigh = C_COLUMNS[i + 1];

    if (cRate >= cLow && cRate <= cHigh) {
      lowCol = i;
      highCol = i + 1;
      return;
    }
  }

  lowCol = lastIndex;
  highCol = lastIndex;
}

float getVoltageAtUsedAhRow(int rowIndex, float cRate) {
  int lowCol = 0;
  int highCol = 0;

  findCRateBounds(cRate, lowCol, highCol);

  if (lowCol == highCol) {
    return VOLTAGE_TABLE[rowIndex][lowCol];
  }

  float cLow = C_COLUMNS[lowCol];
  float cHigh = C_COLUMNS[highCol];

  float vLowC = VOLTAGE_TABLE[rowIndex][lowCol];
  float vHighC = VOLTAGE_TABLE[rowIndex][highCol];

  float voltageAtCRate = linearInterpolate(
    cRate,
    cLow,
    cHigh,
    vLowC,
    vHighC
  );

  return voltageAtCRate;
}

float estimateUsedAhFromVoltageAndCurrent(float measuredVoltage, float measuredCurrent) {
  float localCRate = calculateCRate(measuredCurrent);

  float tempVoltageCurve[AH_ROWS];

  for (int row = 0; row < AH_ROWS; row++) {
    tempVoltageCurve[row] = getVoltageAtUsedAhRow(row, localCRate);
  }

  if (measuredVoltage >= tempVoltageCurve[0]) {
    return 0.0;
  }

  if (measuredVoltage <= tempVoltageCurve[AH_ROWS - 1]) {
    return CAPACITY_AH;
  }

  for (int row = 0; row < AH_ROWS - 1; row++) {
    float vHigh = tempVoltageCurve[row];
    float vLow  = tempVoltageCurve[row + 1];

    float ahHigh = USED_AH_ROWS[row];
    float ahLow  = USED_AH_ROWS[row + 1];

    if (measuredVoltage <= vHigh && measuredVoltage >= vLow) {
      float estimatedUsedAh = linearInterpolate(
        measuredVoltage,
        vHigh,
        vLow,
        ahHigh,
        ahLow
      );

      estimatedUsedAh = clampFloat(
        estimatedUsedAh,
        0.0,
        CAPACITY_AH
      );

      return estimatedUsedAh;
    }
  }

  return CAPACITY_AH;
}

float calculateSOC(float usedAh) {
  float calculatedSOC = ((CAPACITY_AH - usedAh) / CAPACITY_AH) * 100.0;

  calculatedSOC = clampFloat(calculatedSOC, 0.0, 100.0);

  return calculatedSOC;
}

int mapSOCToPWM(float socPercent) {

  if (socPercent <= PWM_SOC_MIN) return 0;
  if (socPercent >= PWM_SOC_MAX) return 255;

  float rawPWM = ((socPercent - PWM_SOC_MIN) * 255.0) /
                 (PWM_SOC_MAX - PWM_SOC_MIN);

  int value = (int)(rawPWM + 0.5);

  if (value < 0) value = 0;
  if (value > 255) value = 255;

  return value;
}

void updatePWMFromSOC(float socPercent) {
  pwmValue = mapSOCToPWM(socPercent);

  analogWrite(pwmPin, pwmValue);

  pwmApproxVoltage = (pwmValue / 255.0) * ARDUINO_ADC_REF;
}

void updatePowerAndEnergy() {
  unsigned long now = millis();

  if (previousEnergyMillis == 0) {
    previousEnergyMillis = now;
  }

  unsigned long deltaMs = now - previousEnergyMillis;
  previousEnergyMillis = now;

  powerW = batteryVoltage * currentA;

  totalEnergyWh += powerW * (deltaMs / 3600000.0);
  elapsedTimeMs += (unsigned long long)deltaMs;
}

void clearLCDRow(byte row) {
  lcd.setCursor(0, row);
  lcd.print("                ");
}

void updateLCDDisplay() {
  unsigned long now = millis();

  if (now - lastLCDPageMillis >= LCD_PAGE_INTERVAL_MS) {
    lastLCDPageMillis = now;
    lcdPage = (lcdPage + 1) % 2;
  }

  unsigned long totalSeconds = (unsigned long)(elapsedTimeMs / 1000ULL);
  unsigned long hours = totalSeconds / 3600UL;
  byte minutes = (totalSeconds % 3600UL) / 60UL;
  byte seconds = totalSeconds % 60UL;

  clearLCDRow(0);
  clearLCDRow(1);

  if (lcdPage == 0) {
    lcd.setCursor(0, 0);
    lcd.print("V:");
    lcd.print(batteryVoltage, 2);
    lcd.print(" I:");
    lcd.print(currentA, 2);
    lcd.print("A");

    lcd.setCursor(0, 1);
    lcd.print("P:");
    lcd.print(powerW, 1);
    lcd.print("W E:");
    lcd.print(totalEnergyWh, 3);
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Time:");

    if (hours < 10) lcd.print('0');
    lcd.print(hours);
    lcd.print(':');

    if (minutes < 10) lcd.print('0');
    lcd.print(minutes);
    lcd.print(':');

    if (seconds < 10) lcd.print('0');
    lcd.print(seconds);

    lcd.setCursor(0, 1);
    lcd.print("SOC:");
    lcd.print(soc, 2);
    lcd.print("% ");
    lcd.print(relayClosed ? "ON" : "OFF");
  }
}

void setupLCD() {
  lcd.begin(16, 2);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Battery Monitor");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");

  lastLCDPageMillis = millis();
}

bool updateLowCapacityWarning(float usedAh) {
  if (usedAh >= LOW_CAPACITY_USED_AH) {
    return true;
  }

  return false;
}

// States: below 0.25 A = RGB OFF; increasing current = GREEN, YELLOW, ORANGE, then slow RED blink; at >=1.30 A = relay cutoff and fast RED blink.
void updateOvercurrentLogic() {

  if (cutoffLatched) {
    relayOpen();
    redBlinkFast();
    return;
  }

  if (currentA >= CUTOFF_CURRENT) {
    cutoffLatched = true;
    relayOpen();

    Serial.println("ACTION: Overcurrent cutoff triggered. Relay opened and latched.");

    redBlinkFast();
    return;
  }

  if (currentA < 0.25) {
    ledOff();
  }
  else if (currentA < GREEN_LIMIT) {
    greenLED();
  }
  else if (currentA < YELLOW_LIMIT) {
    yellowLED();
  }
  else if (currentA < ORANGE_LIMIT) {
    orangeLED();
  }
  else {
    redBlinkSlow();
  }
}

// Relay states: CLOSED = load connected; OPEN = load disconnected. An overcurrent latch always keeps the relay OPEN.
void updateRelayLogic() {
  if (cutoffLatched) {
    relayOpen();
    return;
  }

  if (relayClosed == true) {
    zeroCurrentMode = false;
    chargeBatteryWarning = false;

    if (usedAh >= RELAY_OPEN_USED_AH) {
      relayOpen();
      lowCapacityWarning = true;

      Serial.println("ACTION: Relay opened because estimated usedAh >= 1.364Ah.");
    }

    return;
  }

  if (relayClosed == false) {
    if (floatAbs(currentA) <= ZERO_CURRENT_THRESHOLD_A) {
      zeroCurrentMode = true;

      if (batteryVoltage > ZERO_CURRENT_LOAD_ENABLE_VOLTAGE) {
        relayClose();
        chargeBatteryWarning = false;

        Serial.println("ACTION: Current is zero and voltage > 11.00V. Relay closed, load connected.");
      } else {
        relayOpen();
        chargeBatteryWarning = true;
        lowCapacityWarning = true;

        Serial.println("WARNING: Current is zero and battery voltage <= 11.00V. Charge the battery.");
      }
    } else {
      zeroCurrentMode = false;
      relayOpen();
    }

    return;
  }
}

void printBatteryData() {
  Serial.print("Voltage: ");
  Serial.print(batteryVoltage, 3);
  Serial.print(" V");

  Serial.print(" | Current: ");
  Serial.print(currentA, 3);
  Serial.print(" A");

  Serial.print(" | Power: ");
  Serial.print(powerW, 3);
  Serial.print(" W");

  Serial.print(" | Energy: ");
  Serial.print(totalEnergyWh, 4);
  Serial.print(" Wh");

  Serial.print(" | Time: ");
  Serial.print((unsigned long)(elapsedTimeMs / 1000ULL));
  Serial.print(" s");

  Serial.print(" | SOC: ");
  Serial.print(soc, 2);
  Serial.print(" %");

  Serial.print(" | C-rate: ");
  Serial.print(cRate, 3);
  Serial.print(" C");

  Serial.print(" | PWM: ");
  Serial.print(pwmValue);

  Serial.print(" | OverCurrentLatch: ");
  Serial.print(cutoffLatched ? "YES" : "NO");

  Serial.print(" | Relay: ");
  Serial.println(relayClosed ? "CLOSED / LOAD CONNECTED" : "OPEN / LOAD DISCONNECTED");
}

void batteryGaugeStartup() {
  relayOpen();

  batteryVoltage = readBatteryVoltage();
  currentA = readCurrentFiltered();

  updatePowerAndEnergy();

  cRate = calculateCRate(currentA);

  usedAh = estimateUsedAhFromVoltageAndCurrent(
    batteryVoltage,
    currentA
  );

  soc = calculateSOC(usedAh);

  updatePWMFromSOC(soc);

  setSOCDisplay(soc);

  lowCapacityWarning = updateLowCapacityWarning(usedAh);

  updateOvercurrentLogic();

  if (!cutoffLatched) {
    updateRelayLogic();
  }

  updateLCDDisplay();
  printBatteryData();
}

void setup() {
  Serial.begin(9600);

  if (RELAY_ACTIVE_LOW) {
    RELAY_LOAD_CONNECTED    = LOW;
    RELAY_LOAD_DISCONNECTED = HIGH;
  } else {
    RELAY_LOAD_CONNECTED    = HIGH;
    RELAY_LOAD_DISCONNECTED = LOW;
  }

  pinMode(voltagePin, INPUT);
  pinMode(currentPin, INPUT);

  pinMode(relayPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  relayOpen();
  analogWrite(pwmPin, 0);
  blueLED();

  setupSevenSegmentDisplay();
  setupLCD();

  Serial.println("=======================================");
  Serial.println("Integrated 3S LiPo Battery Monitor Started");
  Serial.println("Voltage + Current + Power + Energy + SOC + RGB Overcurrent");
  Serial.println("SOC PWM 12%-100% -> 0-255 for LPF + LM3914");
  Serial.println("Relay cutoff enabled");
  Serial.println("Manual reset removed");
  Serial.println("=======================================");

  Serial.print("Relay Active Low: ");
  Serial.println(RELAY_ACTIVE_LOW ? "YES" : "NO");

  Serial.print("Arduino ADC Ref: ");
  Serial.print(ARDUINO_ADC_REF, 3);
  Serial.println(" V");

  Serial.print("Voltage Sensor Ratio: ");
  Serial.println(SENSOR_RATIO, 4);

  Serial.print("Zero Current Threshold: ");
  Serial.print(ZERO_CURRENT_THRESHOLD_A, 3);
  Serial.println(" A");

  Serial.print("Rated Current: ");
  Serial.print(RATED_CURRENT, 3);
  Serial.println(" A");

  Serial.print("Cutoff Current: ");
  Serial.print(CUTOFF_CURRENT, 3);
  Serial.println(" A");

  Serial.println("After overcurrent cutoff, reset/power-cycle Arduino to reconnect load.");

  relayOpen();
  blueLED();

  delay(1000);

  zeroCurrentVoltage = calibrateZeroCurrent();

  Serial.print("Calibrated ACS712 Zero Current Voltage: ");
  Serial.print(zeroCurrentVoltage, 4);
  Serial.println(" V");

  delay(500);

  previousEnergyMillis = millis();
  elapsedTimeMs = 0;
  totalEnergyWh = 0.0;

  batteryGaugeStartup();
}

void loop() {
  batteryVoltage = readBatteryVoltage();
  currentA = readCurrentFiltered();

  updatePowerAndEnergy();

  cRate = calculateCRate(currentA);

  usedAh = estimateUsedAhFromVoltageAndCurrent(
    batteryVoltage,
    currentA
  );

  soc = calculateSOC(usedAh);

  updatePWMFromSOC(soc);

  setSOCDisplay(soc);

  lowCapacityWarning = updateLowCapacityWarning(usedAh);

  updateOvercurrentLogic();

  if (!cutoffLatched) {
    updateRelayLogic();
  }

  updateLCDDisplay();
  printBatteryData();

  delay(1000);
}
