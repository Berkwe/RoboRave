#include <Arduino.h>
#include <FastLED.h>



#define PIN_irLeft A2 // sol ir sensörü
#define PIN_irMiddle A1 // orta ir sensörü
#define PIN_irRight A0 //  sağ ir sensörü

#define PIN_motorRightSpeed 5 // sağ motor PWM
#define PIN_motorLeftSpeed 6 // sol motor PWM
#define PIN_motorRight 8 // Sağ motor yön
#define PIN_motorLeft 7 // Sol motor yön
#define PIN_motorEN 3 // Motor sürücüsü enable pini

#define PIN_button 2 // d2 Button pini

#define PIN_led 4 // d4 RGB led pini

// renkler
CRGB leds[1];
CRGB red = CRGB(255,0,0);
CRGB green = CRGB(0,255,0);
CRGB blue = CRGB(0,0,255);
CRGB cyan = CRGB(0,255,255);
CRGB white = CRGB(255,255,255);
CRGB black = CRGB(0,0,0);


#define speedMax 200 // en fazla hız
#define minGroundDistance 950 // robotun havada varsayılması için gereken en düşük değer


bool isReversedLines = false; // çizginin arka plandan daha açık olduğunu belirten flag
int lineDetectionValue = 850; // (|arka_plan_değeri-çizgi_değeri|)/2 böylece dinamik doğru bi dğeer atanır

unsigned long previousTime = 0; // delaySenc için

bool ledState; // ledin durumu

int buttonMode = 1; // mod değeri

bool isCalibrated = false; // kalibre edildi mi
bool isCarOnGround = false; // robot yerde mi

bool sensorStateLeft;
bool sensorStateMiddle;
bool sensorStateRight;


// PID değişkenleri
float g_Kp = 45.0;
float g_Ki = 0.08;
float g_Kd = 12.0;
float g_integral = 0;
float g_previousError = 0;
unsigned long g_lastPIDTime = 0;
int g_baseSpeed = 100;

// Blind detection değişkenleri
unsigned long g_blindStartTime = 0;
bool g_blindActive = false;
float g_lastError = 0;


bool delaySenc(int ms) {
    static unsigned long prevTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - prevTime >= ms) {
        prevTime = currentTime;
        return true;
    }
    else return false;
}


void burnLed(CRGB color, int brightness = 255) {
    if (color == black) FastLED.setBrightness(0);
    else FastLED.setBrightness(255);
    leds[0] = color;
    FastLED.show();
}


void blinkLed(bool var, CRGB color, int interval) {
    if (var) {
        burnLed(black);
        return;
    }

    if (delaySenc(interval)) {
        bool ledState = constrain(FastLED.getBrightness(), 0, 1);
        if (!ledState) {
            burnLed(color);
        } else {
            burnLed(black);
        }
    }
}

void controlCarGround() {
    int sensorL = analogRead(PIN_irLeft);
    int sensorM = analogRead(PIN_irMiddle);
    int sensorR = analogRead(PIN_irRight);
    if (sensorR > minGroundDistance && sensorM > minGroundDistance && sensorL > minGroundDistance) {
        isCarOnGround = false;
        if (buttonMode == 1 && isCalibrated) {
            burnLed(cyan);
        }
    }
    else {
        isCarOnGround = true;
        if (buttonMode == 1 && isCalibrated) {
            burnLed(black);
        }
    }
    
}

void PinInit() {
    // motor initi
    pinMode(PIN_motorLeft, OUTPUT);
    pinMode(PIN_motorRight, OUTPUT);
    pinMode(PIN_motorLeftSpeed, OUTPUT);
    pinMode(PIN_motorRightSpeed, OUTPUT);
    pinMode(PIN_motorEN, OUTPUT);
    // led initi
    FastLED.addLeds<NEOPIXEL, PIN_led>(leds, 1);
    FastLED.setBrightness(255);
    // button initi
    pinMode(PIN_button, INPUT_PULLUP);
}

enum directions {
    forward,       //(1)
    backward,      //(2)
    left,          //(3)
    right,         //(4)
    leftForward,   //(5)
    leftBackward,  //(6)
    rightForward,  //(7)
    rightBackward, //(8)
    stop_it
};

enum mods {
    lineFollowing = 1,
    calibrating = 2
};





void calibrateSensors() { // İr sensörleri arka planla kalibre eder ivmeölçeri falan da aynı şekilde
    burnLed(white);
    delay(2000);
    Serial.println("Kalibrasyon başladı");
    int sensorL = analogRead(PIN_irLeft);
    int sensorM = analogRead(PIN_irMiddle);
    int sensorR = analogRead(PIN_irRight);
    int backgroundDetectionValue;
    int localLineDetectionValue;
    // arka plan kalibrasyonu
    delay(300);
    for (int i = 0; i < 3; i++)
        {
            sensorL += analogRead(PIN_irLeft);
            sensorM += analogRead(PIN_irMiddle);
            sensorR += analogRead(PIN_irRight);
        }
    sensorL /= 3.0;
    sensorM /= 3.0;
    sensorR /= 3.0;
    backgroundDetectionValue = (sensorL+sensorM+sensorR)/3; // üçünün ortalaması
    burnLed(green);
    delay(100);
    burnLed(black);
    delay(100);
    burnLed(green);
    delay(100);
    burnLed(black);
    delay(100);

    // çizgi kalibrasyonu
    sensorL = analogRead(PIN_irLeft);
    sensorM = analogRead(PIN_irMiddle);
    sensorR = analogRead(PIN_irRight);
    burnLed(blue);
    delay(2000);
    burnLed(black);
    for (int i = 0; i < 3; i++)
        {
            sensorL += analogRead(PIN_irLeft);
            sensorM += analogRead(PIN_irMiddle);
            sensorR += analogRead(PIN_irRight);
        }
    sensorL /= 3.0;
    sensorM /= 3.0;
    sensorR /= 3.0;
    localLineDetectionValue = (sensorL+sensorM+sensorR)/3; // üçünün ortalaması
    burnLed(green);
    delay(100);
    burnLed(black);
    delay(100);
    burnLed(green);
    delay(100);
    burnLed(black);
    delay(100);
    isCalibrated = true;
    lineDetectionValue = abs((backgroundDetectionValue-localLineDetectionValue))/2;
    if (lineDetectionValue>backgroundDetectionValue) isReversedLines = true; // eğer çizgi açık bir renk ise
    digitalWrite(PIN_motorEN, 1);
    Serial.println("\narka plan : ");
    Serial.print(backgroundDetectionValue);
    Serial.println("\nçizgi : ");
    Serial.print(localLineDetectionValue);
}


void goTo(directions direction, uint8_t speed = 0) { // motoru hareket ettirir
    if (!isCalibrated) return;
    

    speed = constrain(speed, 0, speedMax);
    switch (direction)
    {
    case forward:
        digitalWrite(PIN_motorRight, 1);
        analogWrite(PIN_motorRightSpeed, speed);
        digitalWrite(PIN_motorLeft, 1);
        analogWrite(PIN_motorLeftSpeed, speed);
        break;
    case backward:
        digitalWrite(PIN_motorRight, 0);
        analogWrite(PIN_motorRightSpeed, speed);
        digitalWrite(PIN_motorLeft, 0);
        analogWrite(PIN_motorLeftSpeed, speed);
        break;
    case left:
        digitalWrite(PIN_motorRight, 1);
        analogWrite(PIN_motorRightSpeed, speed);
        digitalWrite(PIN_motorLeft, 0);
        analogWrite(PIN_motorLeftSpeed, speed);
        break;
    case right:
        digitalWrite(PIN_motorRight, 0);
        analogWrite(PIN_motorRightSpeed, speed);
        digitalWrite(PIN_motorLeft, 1);
        analogWrite(PIN_motorLeftSpeed, speed);
        break;
    case leftForward:
        digitalWrite(PIN_motorRight, 1);
        analogWrite(PIN_motorRightSpeed, speed);
        analogWrite(PIN_motorLeftSpeed, 0);
        break;
    case leftBackward:
        digitalWrite(PIN_motorRight, 0);
        analogWrite(PIN_motorRightSpeed, speed);
        analogWrite(PIN_motorLeftSpeed, 0);
        break;
    case rightForward:
        analogWrite(PIN_motorRightSpeed, 0);
        digitalWrite(PIN_motorLeft, 1);
        analogWrite(PIN_motorLeftSpeed, speed);
        break;
    case rightBackward:
        analogWrite(PIN_motorRightSpeed, 0);
        digitalWrite(PIN_motorLeft, 0);
        analogWrite(PIN_motorLeftSpeed, speed);
        break;
    case stop_it:
        analogWrite(PIN_motorRightSpeed, 0);
        analogWrite(PIN_motorLeftSpeed, 0);
        break;
    default:
        break;
    }

}


void setPIDParameters(float kp, float ki, float kd, int baseSpeed) {
    g_Kp = kp;
    g_Ki = ki;
    g_Kd = kd;
    g_baseSpeed = baseSpeed;
}

void blindDetection() { // robot yönünü kaybettiğinde çalışır
    unsigned long elapsed = millis() - g_blindStartTime;
    
    if (elapsed < 200) {
        goTo(right, 100);
    }
    else if (elapsed < 1600) {
        goTo(left, 120);
    }
    else if (elapsed < 2000) {
        goTo(right, 100);
    }
    else if (elapsed < 3000) {
        goTo(forward, 80);
    }
    else {
        goTo(stop_it, 0);
        g_integral = 0;
        g_previousError = 0;
        g_lastPIDTime = 0;
    }
}

void followLine() { // çizgi takibi
    if (!isCalibrated || !isCarOnGround) {
        goTo(stop_it, 0);
        g_integral = 0;
        g_previousError = 0;
        g_lastPIDTime = 0;
        g_blindActive = false;
        return;
    }

    int sensorL = analogRead(PIN_irLeft);
    int sensorM = analogRead(PIN_irMiddle);
    int sensorR = analogRead(PIN_irRight);

    bool left = isReversedLines ? (sensorL > lineDetectionValue) : (sensorL < lineDetectionValue);
    bool middle = isReversedLines ? (sensorM > lineDetectionValue) : (sensorM < lineDetectionValue);
    bool right = isReversedLines ? (sensorR > lineDetectionValue) : (sensorR < lineDetectionValue);

    if (!left && !middle && !right) {
        if (!g_blindActive) {
            g_blindActive = true;
            g_blindStartTime = millis();
            g_integral = 0;
            g_previousError = 0;
            g_lastPIDTime = 0;
        }
        blindDetection();
        return;
    }

    g_blindActive = false;

    float totalWeight = left + middle + right;
    float error = (-1.0 * left + 0.0 * middle + 1.0 * right) / totalWeight;
    g_lastError = error;

    unsigned long currentTime = millis();
    if (g_lastPIDTime == 0) {
        g_lastPIDTime = currentTime;
        g_previousError = error;
        goTo(forward, g_baseSpeed);
        return;
    }

    float dt = (currentTime - g_lastPIDTime) / 1000.0;
    if (dt <= 0 || dt > 1.0) dt = 0.02;
    g_lastPIDTime = currentTime;

    float P = g_Kp * error;

    if (abs(error) < 0.01) g_integral = 0;
    else {
        g_integral += error * dt;
        g_integral = constrain(g_integral, -50.0, 50.0);
    }
    float I = g_Ki * g_integral;

    float derivative = (error - g_previousError) / dt;
    float D = g_Kd * derivative;
    g_previousError = error;

    int correction = P + I + D;
    int leftSpeed = g_baseSpeed - correction;
    int rightSpeed = g_baseSpeed + correction;

    leftSpeed = constrain(leftSpeed, 0, speedMax);
    rightSpeed = constrain(rightSpeed, 0, speedMax);

    digitalWrite(PIN_motorRight, 1);
    analogWrite(PIN_motorRightSpeed, rightSpeed);
    digitalWrite(PIN_motorLeft, 1);
    analogWrite(PIN_motorLeftSpeed, leftSpeed);
}


void controlLine() { // çizgi kontrolü
    if (!isCalibrated) return;
    if (!isCarOnGround) return;

    int sensorStateLeft = analogRead(PIN_irLeft);
    int sensorStateMiddle = analogRead(PIN_irMiddle);
    int sensorStateRight = analogRead(PIN_irRight);

    if (!isReversedLines) {
        if (sensorStateLeft < lineDetectionValue) sensorStateLeft = 1;
        else sensorStateLeft = 0;
        if (sensorStateMiddle < lineDetectionValue) sensorStateMiddle = 1;
        else sensorStateMiddle = 0;
        if (sensorStateRight < lineDetectionValue) sensorStateRight = 1;
        else sensorStateRight = 0;
    }
    else {
        if (sensorStateLeft > lineDetectionValue) sensorStateLeft = 1;
        else sensorStateLeft = 0;
        if (sensorStateMiddle > lineDetectionValue) sensorStateMiddle = 1;
        else sensorStateMiddle = 0;
        if (sensorStateRight > lineDetectionValue) sensorStateRight = 1;
        else sensorStateRight = 0;
    }
}

void setup() {
    Serial.begin(9600);
    buttonMode = 1;
    PinInit();

}



void loop() {

    blinkLed(isCalibrated, red, 500);
    controlCarGround();

    // Mod seçimi
    if (!digitalRead(PIN_button) && delaySenc(70)) { 
        if (buttonMode < 2) buttonMode += 1; 
        else buttonMode = 1; 
    }

    // Mod kontrolü
    switch (buttonMode) {
        case 1:
            followLine();
            break;
        case 2:
            calibrateSensors();
            buttonMode = 1;
            break;
    }
}

