#include <Wire.h>
#include <Adafruit_INA219.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

float cachedCurrent = 0.0;
unsigned long lastCurrentRead = 0;

float cachedVoltage = 0.0;
unsigned long lastVoltageRead = 0;

String lastLine0 = "";
String lastLine1 = "";


// Ekran powitalny
const unsigned long WELCOME_CHAR_DELAY   = 160;  
const unsigned long WELCOME_BY_DELAY     = 800;   
const unsigned long WELCOME_AUTHOR_DELAY = 1200; 

// Sekwencje D6/D8
const unsigned long SEQUENCE_D6_ENABLE_TIME  = 400;  
const unsigned long SEQUENCE_D8_ENABLE_TIME  = 525;  
const unsigned long SEQUENCE_D6_DISABLE_TIME = 600;  

// Timery systemowe
const unsigned long ENCODER_DELAY           = 50;   
const unsigned long DEBOUNCE_DELAY          = 25;   
const unsigned long DISPLAY_UPDATE_INTERVAL = 100; 
const unsigned long VOLTAGE_UPDATE_INTERVAL = 400; 
const unsigned long CONTROL_UPDATE_INTERVAL = 5;  
const unsigned long MAIN_LOOP_DELAY         = 1;    
const unsigned long ERROR_LOOP_DELAY        = 30;   

Adafruit_INA219 ina219;

LiquidCrystal_I2C lcd(0x27, 16, 2);

int encoderPinCLK      = 2;   
int encoderPinDT       = 3;   
int encoderSW          = 4;   
int enablePin          = A2; 
int sequencePin1       = 6;   
int sequencePin2       = 8;  
int relayPin           = 7;   
int voltagePin         = A1;  
int voltageSwitchPin   = A3;  
const int calibrationPin = A0; 
const int pwmPin       = 9;   

float desiredCurrent = 0.0;   
int   pwmValue       = 0;     
float pwmAccumulator = 0.0;   

volatile bool encoderChanged   = false;
volatile int  encoderDirection = 0;
volatile uint8_t lastCLK;


unsigned long lastEncoderChange = 0;

bool pwmEnabled = false;

bool lastEnableButtonState     = HIGH;
bool currentEnableButtonState  = HIGH;
bool lastEncoderButtonState    = HIGH;
bool currentEncoderButtonState = HIGH;
unsigned long lastDebounceTime          = 0;
unsigned long lastEncoderButtonDebounce = 0;

bool lastVoltageSwitchState      = HIGH;
bool currentVoltageSwitchState   = HIGH;
unsigned long lastVoltageSwitchDebounce = 0;
bool voltageDividerEnabled       = false;
bool safetyLockActive            = false;  
bool overTempLockout             = false; 
const float SAFETY_VOLTAGE_THRESHOLD = 10.0; 

unsigned long enableStartTime  = 0;
unsigned long pin1EnableTime   = 0;   
unsigned long disableStartTime = 0;  
bool enableSequenceActive      = false;
bool disableSequenceActive     = false;
bool pin1EnableDone            = false; 
bool pin2EnableDone            = false; 
bool pin1DisableDone           = false; 

const float MAX_VOLTAGE = 4.5; 

const float CURRENT_STEP = 10.0; 
const float MAX_CURRENT  = 400.0;

const float FILTER_R   = 1000.0;
const float FILTER_C   = 2.2e-6;
const float FILTER_TAU = FILTER_R * FILTER_C;

const float AREF_VOLTAGE   = 5.0;
const int   PWM_RESOLUTION = 255; 
const float VOLTAGE_PER_PWM = AREF_VOLTAGE / PWM_RESOLUTION;

const int CURRENT_SAMPLES = 5;
float currentBuffer[CURRENT_SAMPLES] = {0};
int   currentIndex = 0;

const int VOLTAGE_SAMPLES = 8;
float voltageBuffer[VOLTAGE_SAMPLES] = {0};
int   voltageIndex = 0;

unsigned long lastDisplayUpdate = 0;

unsigned long lastVoltageUpdate = 0;
float displayVoltage = 0.0;
float displayPower   = 0.0;

const float DEAD_ZONE    = 1.0;  
const float MAX_PWM_STEP = 0.5;  
const float CONTROL_GAIN = 0.06;
unsigned long lastControlUpdate = 0;

const int NUM_CALIBRATION_POINTS = 9;

struct CalibrationPoint {
    float voltage;
    float current;
} calibration[NUM_CALIBRATION_POINTS] = {
    {0.0,  0},
    {0.1,  10.0},
    {0.2,  20.0},
    {0.3,  30.0},
    {0.4,  40.0},
    {1.0,  100.0},
    {2.0,  200.0},
    {3.0,  300.0},
    {4.0,  400.0},
};

float getFilteredCurrent() {
    float newCurrent = ina219.getCurrent_mA();
    if (newCurrent < 0) newCurrent = 0.0;

    currentBuffer[currentIndex] = newCurrent;
    currentIndex = (currentIndex + 1) % CURRENT_SAMPLES;

    float sum = 0.0;
    for (int i = 0; i < CURRENT_SAMPLES; i++) {
        sum += currentBuffer[i];
    }
    float avg = sum / CURRENT_SAMPLES;

    static bool  initialized = false;
    static float filtered    = 0.0;

    if (!initialized) {
        filtered    = avg;
        initialized = true;
        return filtered;
    }

    const float alpha = 0.25; 
    float candidate = filtered * (1.0 - alpha) + avg * alpha;

    const float MAX_DELTA = 5.0; 
    float delta = candidate - filtered;
    if (delta > MAX_DELTA)      delta = MAX_DELTA;
    else if (delta < -MAX_DELTA) delta = -MAX_DELTA;

    filtered += delta;
    return filtered;
}

float getFilteredVoltageA1() {
    int rawHV = analogRead(voltagePin);
    float hvBase = (rawHV / 1023.0) * AREF_VOLTAGE;
    hvBase *= 100.0;

    int rawCal = analogRead(calibrationPin);
    float vCal = (rawCal / 1023.0) * AREF_VOLTAGE;

    const float MIN_GAIN = 0.9167;
    const float MAX_GAIN = 1.0833;

    float gain = MIN_GAIN + (vCal / AREF_VOLTAGE) * (MAX_GAIN - MIN_GAIN);

    // Zastosuj kalibrację
    float hvCalibrated = hvBase * gain;

    voltageBuffer[voltageIndex] = hvCalibrated;
    voltageIndex = (voltageIndex + 1) % VOLTAGE_SAMPLES;

    float sum = 0.0;
    for (int i = 0; i < VOLTAGE_SAMPLES; i++) {
        sum += voltageBuffer[i];
    }
    return sum / VOLTAGE_SAMPLES;
}

float getDesiredCurrentForVoltage(float voltage) {
    float rawCurrent = 0.0;

    for (int i = 0; i < NUM_CALIBRATION_POINTS - 1; i++) {
        if (voltage >= calibration[i].voltage && voltage < calibration[i+1].voltage) {
            float slope = (calibration[i+1].current - calibration[i].current) /
                          (calibration[i+1].voltage - calibration[i].voltage);
            rawCurrent = calibration[i].current + slope * (voltage - calibration[i].voltage);
            break;
        }
    }

    return round(rawCurrent / 10.0) * 10.0;
}

void encoderISR() {
    int currentCLK = digitalRead(encoderPinCLK);

    if (currentCLK != lastCLK) {
        if (digitalRead(encoderPinDT) != currentCLK) {
            encoderDirection = 1;
        } else {
            encoderDirection = -1;
        }
        encoderChanged = true;
    }
    lastCLK = currentCLK;
}
void updateCurrentMeasurement() {
    if (millis() - lastCurrentRead >= CONTROL_UPDATE_INTERVAL) {
        cachedCurrent = getFilteredCurrent();
        lastCurrentRead = millis();
    }
}

void updateVoltageMeasurement() {
    if (millis() - lastVoltageRead >= VOLTAGE_UPDATE_INTERVAL) {
        cachedVoltage = getFilteredVoltageA1();
        lastVoltageRead = millis();
    }
}

void handleEnableButton() {
    int reading = digitalRead(enablePin);

    if (reading != lastEnableButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
        if (reading != currentEnableButtonState) {
            currentEnableButtonState = reading;

            if (currentEnableButtonState == LOW) {
                if (!pwmEnabled && safetyLockActive) {
                    return;
                }
                if (!pwmEnabled && overTempLockout) {
                    return;
                }
                if (!pwmEnabled && desiredCurrent == 0) {
                    return;
                }

                pwmEnabled = !pwmEnabled;

                if (pwmEnabled) {
                    if (desiredCurrent > 0) {
                        enableStartTime      = millis();
                        enableSequenceActive = true;
                        pin1EnableDone       = false;
                        pin2EnableDone       = false;
                    }
                } else {
                    pwmValue       = 0;
                    pwmAccumulator = 0.0;
                    noInterrupts();
                    OCR1A = 0;
                    interrupts();

                    digitalWrite(sequencePin2, LOW);

                    disableStartTime      = millis();
                    disableSequenceActive = true;
                    pin1DisableDone       = false;

                    enableSequenceActive = false;
                }
            }
        }
    }

    lastEnableButtonState = reading;
}

void handleEncoderButton() {
    int reading = digitalRead(encoderSW);

    if (reading != lastEncoderButtonState) {
        lastEncoderButtonDebounce = millis();
    }

    if ((millis() - lastEncoderButtonDebounce) > DEBOUNCE_DELAY) {
        if (reading != currentEncoderButtonState) {
            currentEncoderButtonState = reading;

            if (currentEncoderButtonState == LOW && !pwmEnabled) {
                desiredCurrent = 0.0;
            }
        }
    }

    lastEncoderButtonState = reading;
}

void handleEncoder() {
    if (pwmEnabled) {
        encoderChanged = false;
        return;
    }

    if (encoderChanged) {
        if ((millis() - lastEncoderChange) >= ENCODER_DELAY) {
            desiredCurrent += encoderDirection * CURRENT_STEP;
            desiredCurrent  = constrain(desiredCurrent, 0.0, MAX_CURRENT);
            lastEncoderChange = millis();
        }
        encoderChanged = false;
    }
}

int calculatePWM(float voltage) {
    float pwmFloat = (voltage / AREF_VOLTAGE) * PWM_RESOLUTION;
    int pwm = round(pwmFloat);
    return constrain(pwm, 0, 255);
}

float calculateVoltage(int pwm) {
    return (pwm * VOLTAGE_PER_PWM);
}

void handleVoltageDividerSwitch() {
    int reading = digitalRead(voltageSwitchPin);

    if (reading != lastVoltageSwitchState) {
        lastVoltageSwitchDebounce = millis();
    }

    if ((millis() - lastVoltageSwitchDebounce) > DEBOUNCE_DELAY) {
        if (reading != currentVoltageSwitchState) {
            currentVoltageSwitchState = reading;
        }
    }

    lastVoltageSwitchState = reading;

    float currentVoltage = cachedVoltage;

    bool dividerShouldBeOn = true;

    if (pwmEnabled) {
        dividerShouldBeOn = (currentVoltageSwitchState == LOW);
        safetyLockActive  = false;
    } else {
        dividerShouldBeOn = true;

        if (currentVoltage >= SAFETY_VOLTAGE_THRESHOLD) {
            safetyLockActive = true;
        } else {
            safetyLockActive = false;
        }
    }

    digitalWrite(relayPin, dividerShouldBeOn ? HIGH : LOW);
    voltageDividerEnabled = dividerShouldBeOn;
}

void precisionCurrentControl() {
    if (!pwmEnabled) return;
    if (!pin2EnableDone) return;

    if ((millis() - lastControlUpdate) < CONTROL_UPDATE_INTERVAL) {
        return;
    }
    lastControlUpdate = millis();

    float currentCurrent = cachedCurrent;
    float error          = desiredCurrent - currentCurrent;

    if (abs(error) <= DEAD_ZONE) {
        return;
    }

    float correction = error * CONTROL_GAIN;
    float maxStep    = (abs(error) > 15.0) ? MAX_PWM_STEP * 1.2 : MAX_PWM_STEP;

    if (correction > maxStep)       correction = maxStep;
    else if (correction < -maxStep) correction = -maxStep;

    pwmAccumulator += correction;
    pwmAccumulator = constrain(pwmAccumulator, 0.0, 255.0);

    pwmValue = (int)round(pwmAccumulator);
    noInterrupts();
    OCR1A = pwmValue;
    interrupts();
}

void handleSequenceTimers() {
    unsigned long currentTime = millis();

    if (enableSequenceActive) {
        if (!pin1EnableDone && (currentTime - enableStartTime >= SEQUENCE_D6_ENABLE_TIME)) {
            digitalWrite(sequencePin1, HIGH);
            pin1EnableDone = true;
            pin1EnableTime = currentTime;
        }

        if (pin1EnableDone && !pin2EnableDone && (currentTime - pin1EnableTime >= SEQUENCE_D8_ENABLE_TIME)) {
            digitalWrite(sequencePin2, HIGH);
            pin2EnableDone       = true;
            enableSequenceActive = false;
        }
    }

    if (disableSequenceActive) {
        if (!pin1DisableDone && (currentTime - disableStartTime >= SEQUENCE_D6_DISABLE_TIME)) {
            digitalWrite(sequencePin1, LOW);
            pin1DisableDone       = true;
            disableSequenceActive = false;
        }
    }
}

const int ntcPin = A6;

const float NTC_SERIES_RESISTOR      = 100000.0;
const float NTC_NOMINAL_RESISTANCE   = 100000.0;
const float NTC_NOMINAL_TEMPERATURE  = 25.0;
const float NTC_BETA_COEFFICIENT     = 3950.0; 
const float NTC_SHUTDOWN_TEMPERATURE = 50.0;  
const float NTC_RESET_TEMPERATURE    = 40.0; 

float readNtcTemperatureC() {
    int adc = analogRead(ntcPin);
    if (adc <= 0 || adc >= 1023) {
        return 150.0;
    }

    float resistance = NTC_SERIES_RESISTOR * ((float)adc / (1023.0 - (float)adc));

    // Steinhart–Hart (wersja uproszczona z B-eta)
    float steinhart = resistance / NTC_NOMINAL_RESISTANCE; 
    steinhart = log(steinhart); 
    steinhart /= NTC_BETA_COEFFICIENT; 
    steinhart += 1.0 / (NTC_NOMINAL_TEMPERATURE + 273.15); 
    steinhart = 1.0 / steinhart;
    steinhart -= 273.15;  

    return steinhart;
}

void thermalShutdownDisableLoad() {
    if (!pwmEnabled) return; 

    pwmEnabled      = false;
    pwmValue        = 0;
    pwmAccumulator  = 0.0;
    noInterrupts();
    OCR1A = 0;
    interrupts();

    digitalWrite(sequencePin2, LOW);

    disableStartTime      = millis();
    disableSequenceActive = true;
    pin1DisableDone       = false;

    enableSequenceActive = false;
}

void handleTemperatureProtection() {
    static unsigned long lastTempRead = 0;
    const unsigned long TEMP_READ_INTERVAL = 500;

    unsigned long now = millis();
    if (now - lastTempRead < TEMP_READ_INTERVAL) {
        return;
    }
    lastTempRead = now;

    float tempC = readNtcTemperatureC();

    if (tempC >= NTC_SHUTDOWN_TEMPERATURE) {
        if (pwmEnabled) {
            thermalShutdownDisableLoad();
        }
        overTempLockout = true;
    }
    else if (overTempLockout && tempC <= NTC_RESET_TEMPERATURE) {
        overTempLockout = false;
    }
}

void updateDisplay();

void setup() {
    Serial.begin(9600);
    Wire.begin();

    lcd.init();
    lcd.backlight();
    lcd.clear();

    String welcomeText = "DUMMYLOAD V2";
    int startPos = 2;

    lcd.setCursor(startPos, 0);
    for (int i = 0; i < welcomeText.length(); i++) {
        lcd.print(welcomeText.charAt(i));
        delay(WELCOME_CHAR_DELAY);
    }

    lcd.setCursor(2, 1);
    lcd.print("by:");
    delay(WELCOME_BY_DELAY);
    lcd.print(" STWuRCA");

    delay(WELCOME_AUTHOR_DELAY);
    lcd.clear();

    analogReference(EXTERNAL);

    if (!ina219.begin()) {
        lcd.clear();
        lcd.print("BLAD INA219!");
        lcd.setCursor(0, 1);
        lcd.print("Sprawdz podlacz");
        while (1) { delay(ERROR_LOOP_DELAY); }
    }

    // Konfiguracja pinów
    pinMode(pwmPin, OUTPUT);
    pinMode(encoderPinCLK, INPUT_PULLUP);
    pinMode(encoderPinDT, INPUT_PULLUP);
    pinMode(encoderSW, INPUT_PULLUP);
    pinMode(enablePin, INPUT_PULLUP);
    pinMode(calibrationPin, INPUT);

    pinMode(sequencePin1, OUTPUT);
    pinMode(sequencePin2, OUTPUT);
    pinMode(relayPin, OUTPUT);
    pinMode(voltagePin, INPUT);
    pinMode(voltageSwitchPin, INPUT_PULLUP);

    digitalWrite(sequencePin1, LOW);
    digitalWrite(sequencePin2, LOW);
    digitalWrite(relayPin, LOW);
    updateCurrentMeasurement();
    updateVoltageMeasurement();

    for (int i = 0; i < CURRENT_SAMPLES; i++) currentBuffer[i]  = 0.0;
    for (int i = 0; i < VOLTAGE_SAMPLES; i++) voltageBuffer[i] = 0.0;

    lastEnableButtonState     = digitalRead(enablePin);
    currentEnableButtonState  = lastEnableButtonState;
    lastEncoderButtonState    = digitalRead(encoderSW);
    currentEncoderButtonState = lastEncoderButtonState;
    lastVoltageSwitchState    = digitalRead(voltageSwitchPin);
    currentVoltageSwitchState = lastVoltageSwitchState;
    lastCLK = PIND & (1 << 2);

    attachInterrupt(digitalPinToInterrupt(encoderPinCLK), encoderISR, CHANGE);
    float initI = ina219.getCurrent_mA();
    for (int i = 0; i < CURRENT_SAMPLES; i++) currentBuffer[i] = initI;

    float initV = cachedVoltage;
    for (int i = 0; i < VOLTAGE_SAMPLES; i++) voltageBuffer[i] = initV;

    TCCR1A = 0;
    TCCR1B = 0;

    TCCR1A |= (1 << WGM10);         
    TCCR1B |= (1 << WGM12);  

    TCCR1A |= (1 << COM1A1); 
    TCCR1B |= (1 << CS11);   

    noInterrupts();
    OCR1A = 0;
    interrupts(); 

    pwmValue           = 0;
    pwmAccumulator     = 0.0;
    voltageDividerEnabled = false;
    safetyLockActive      = false;
    overTempLockout       = false;
}

void loop() {
    updateCurrentMeasurement();
    updateVoltageMeasurement();

    handleEnableButton();
    handleEncoderButton();
    handleVoltageDividerSwitch();
    handleEncoder();
    handleSequenceTimers();
    precisionCurrentControl();
    handleTemperatureProtection();

    if ((millis() - lastDisplayUpdate) >= DISPLAY_UPDATE_INTERVAL) {
        updateDisplay();
        lastDisplayUpdate = millis();
    }

    delay(MAIN_LOOP_DELAY);
}

void updateDisplay() {
    float currentCurrent = cachedCurrent;

    if ((millis() - lastVoltageUpdate) >= VOLTAGE_UPDATE_INTERVAL) {
        displayVoltage = getFilteredVoltageA1();

        if (voltageDividerEnabled && pwmEnabled) {
            displayPower = (displayVoltage * currentCurrent);
        } else {
            displayPower = 0.0;
        }

        lastVoltageUpdate = millis();
    }

    String newLine0 = "Set:";
    newLine0 += String((int)desiredCurrent);
    newLine0 += "mA ";

    int voltageStartPos = newLine0.length();
    newLine0 += String((int)displayVoltage);
    newLine0 += "V";

    while (newLine0.length() < 15) newLine0 += " ";
    newLine0 += pwmEnabled ? "E" : "D";

    String newLine1 = "I:";
    newLine1 += String((int)currentCurrent);
    newLine1 += "mA";

    int powerStartPos = voltageStartPos;
    if (powerStartPos <= newLine1.length()) {
        powerStartPos = newLine1.length() + 1;
    }
    while (newLine1.length() < powerStartPos) newLine1 += " ";

    if (voltageDividerEnabled && pwmEnabled) {
        if (displayPower < 1000.0) {
            newLine1 += String((int)displayPower);
            newLine1 += "mW";
        } else {
            newLine1 += String((int)(displayPower / 1000.0));
            newLine1 += "W";
        }
    }

    if (newLine0.length() > 16) newLine0 = newLine0.substring(0, 16);
    if (newLine1.length() > 16) newLine1 = newLine1.substring(0, 16);
    while (newLine0.length() < 16) newLine0 += " ";
    while (newLine1.length() < 16) newLine1 += " ";

    if (newLine0 != lastLine0) {
        lcd.setCursor(0, 0);
        lcd.print(newLine0);
        lastLine0 = newLine0;
    }

    if (newLine1 != lastLine1) {
        lcd.setCursor(0, 1);
        lcd.print(newLine1);
        lastLine1 = newLine1;
    }
}