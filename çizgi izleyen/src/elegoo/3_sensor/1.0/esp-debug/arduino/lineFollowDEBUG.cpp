#include <Arduino.h>
#include <FastLED.h>
#include <Servo.h>

Servo myServo;


String receivedCommand = "";
bool isESPCommand = false;  // YENİ EKLEME

#define PIN_trig 13
#define PIN_echo 12
#define PIN_servo 10
#define PIN_irLeft A2
#define PIN_irMiddle A1
#define PIN_irRight A0

#define PIN_motorRightSpeed 5
#define PIN_motorLeftSpeed 6
#define PIN_motorRight 7
#define PIN_motorLeft 8
#define PIN_motorEN 3

#define PIN_button 2
#define PIN_led 4
#define PIN_irLedPWM 11

CRGB leds[1];
CRGB red = CRGB(255,0,0);
CRGB blue = CRGB(0,0,255);
CRGB white = CRGB(255,255,255);
CRGB green = CRGB(0,255,0);
CRGB cyan = CRGB(0,255,255);
CRGB yellow = CRGB(255,255,0);
CRGB black = CRGB(0,0,0);

#define speedMax 130
#define minGroundDistance 1000



bool isReversedLines = false;
int lineDetectionValue = 700;
bool isStop = false;

int buttonMode = 1;
bool isCalibrated = false;
bool isCarOnGround = false;

float g_Kp = 50.0;
float g_Ki = 0.8;
float g_Kd = 12.0;
float g_integral = 0;
float g_previousError = 0;
unsigned long g_lastPIDTime = 0;
int g_baseSpeed = 70;

unsigned long g_blindStartTime = 0;
bool g_blindActive = false;

unsigned long g_triggerTime = 0;
bool g_waitingEcho = false;
long g_lastDistance = 100;

bool debug = true;

enum TimerID {
    TIMER_LED,
    TIMER_BUTTON,
    TIMER_LINE_DETECT,
    TIMER_TURN,
    TIMER_SERVO,
    TIMER_ULTRASONIC,
    TIMER_DEBUG,
    TIMER_COUNT
};
unsigned long g_timers[TIMER_COUNT] = {0};

enum directions {
    forward,
    backward,
    left,
    right,
    leftForward,
    leftBackward,
    rightForward,
    rightBackward,
    stop_it
};

enum mods {
    lineFollowing = 1,
    calibrating = 2
};

float g_lastError = 0;
unsigned long lastRepeatTime = 0;
bool g_handlingIntersection = false;
unsigned long g_intersectionStartTime = 0;
int g_intersectionType = 0;

enum reactionTypes {
    AUTO,
    STOP,
    LEFT,
    MID,
    RIGHT
};

struct lineReactionsStruct {
    int state;
    reactionTypes reaction;
};

lineReactionsStruct lineReactions[] = {
    {011, AUTO},
    {110, AUTO},
    {111, AUTO}
};

enum stages {
    DEPARTURE,
    DISCHARGE,
    RETURN
};
stages currentStage = DEPARTURE;
bool isNewReturn = true;
bool g_slowDownForIntersection = false;

bool delaySenc(TimerID id, int ms) {
    unsigned long currentTime = millis();
    if (currentTime - g_timers[id] >= ms) {
        g_timers[id] = currentTime;
        return true;
    }
    return false;
}

void burnLed(CRGB color, int brightness = 255) {
    if (color == black) FastLED.setBrightness(0);
    else FastLED.setBrightness(255);
    leds[0] = color;
    FastLED.show();
}

void blinkLed(bool var, CRGB color, int interval = 500) {
    if (var) {
        burnLed(black);
        return;
    }

    if (delaySenc(TIMER_LED, interval)) {
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
            isStop = false;
            digitalWrite(PIN_motorEN, 0);
            currentStage = DEPARTURE;
            burnLed(cyan);
        }
    }
    else {
        isCarOnGround = true;
        if (buttonMode == 1 && isCalibrated) {
            digitalWrite(PIN_motorEN, 1);
            burnLed(black);
        }
    }
}

void PinInit() {
    pinMode(PIN_motorLeft, OUTPUT);
    pinMode(PIN_motorRight, OUTPUT);
    pinMode(PIN_motorLeftSpeed, OUTPUT);
    pinMode(PIN_motorRightSpeed, OUTPUT);
    pinMode(PIN_motorEN, OUTPUT);
    FastLED.addLeds<NEOPIXEL, PIN_led>(leds, 1);
    FastLED.setBrightness(255);
    pinMode(PIN_irLedPWM, OUTPUT);
    pinMode(PIN_button, INPUT_PULLUP);
    myServo.attach(10);
    pinMode(PIN_trig, OUTPUT);
    pinMode(PIN_echo, INPUT);
}

void goTo(directions direction, uint8_t speed = 0, uint8_t speedRight = 0, uint8_t speedLeft = 0) {
    if (!isCalibrated || !isCarOnGround) return;
    speedLeft = constrain((speed == 0 ? speedLeft : speed), 0, speedMax);
    speedRight = constrain((speed == 0 ? speedRight : speed), 0, speedMax);
    switch (direction)
    {
    case forward:
        digitalWrite(PIN_motorRight, 1);
        analogWrite(PIN_motorRightSpeed, speedRight);
        digitalWrite(PIN_motorLeft, 1);
        analogWrite(PIN_motorLeftSpeed, speedLeft);
        break;
    case backward:
        digitalWrite(PIN_motorRight, 0);
        analogWrite(PIN_motorRightSpeed, speedRight);
        digitalWrite(PIN_motorLeft, 0);
        analogWrite(PIN_motorLeftSpeed, speedLeft);
        break;
    case left:
        digitalWrite(PIN_motorRight, 1);
        analogWrite(PIN_motorRightSpeed, speedRight);
        digitalWrite(PIN_motorLeft, 0);
        analogWrite(PIN_motorLeftSpeed, speedLeft);
        break;
    case right:
        digitalWrite(PIN_motorRight, 0);
        analogWrite(PIN_motorRightSpeed, speedRight);
        digitalWrite(PIN_motorLeft, 1);
        analogWrite(PIN_motorLeftSpeed, speedLeft);
        break;
    case leftForward:
        digitalWrite(PIN_motorRight, 1);
        analogWrite(PIN_motorRightSpeed, speedRight);
        analogWrite(PIN_motorLeftSpeed, 0);
        break;
    case leftBackward:
        digitalWrite(PIN_motorRight, 0);
        analogWrite(PIN_motorRightSpeed, speedRight);
        analogWrite(PIN_motorLeftSpeed, 0);
        break;
    case rightForward:
        analogWrite(PIN_motorRightSpeed, 0);
        digitalWrite(PIN_motorLeft, 1);
        analogWrite(PIN_motorLeftSpeed, speedLeft);
        break;
    case rightBackward:
        analogWrite(PIN_motorRightSpeed, 0);
        digitalWrite(PIN_motorLeft, 0);
        analogWrite(PIN_motorLeftSpeed, speedLeft);
        break;
    case stop_it:
        analogWrite(PIN_motorRightSpeed, 0);
        analogWrite(PIN_motorLeftSpeed, 0);
        break;
    default:
        break;
    }
}

long readUltrasonicDistance() {
    digitalWrite(PIN_trig, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_trig, LOW);
    long duration = pulseIn(PIN_echo, HIGH);
    long distanceCm = duration * 0.034 / 2;
    return distanceCm;
}

void calibrateSensors() {
    if (debug) Serial.println(F("\n>>> calibrateSensors()"));
    
    digitalWrite(PIN_motorEN, 0);
    burnLed(white);
    delay(2000);
    
    unsigned long sensorL = analogRead(PIN_irLeft);
    unsigned long sensorM = analogRead(PIN_irMiddle);
    unsigned long sensorR = analogRead(PIN_irRight);
    int backgroundDetectionValue;
    int localLineDetectionValue;
    
    delay(300);
    for (int i = 0; i < 3; i++) {
        sensorL += analogRead(PIN_irLeft);
        sensorM += analogRead(PIN_irMiddle);
        sensorR += analogRead(PIN_irRight);
    }
    sensorL /= 3.0;
    sensorM /= 3.0;
    sensorR /= 3.0;
    backgroundDetectionValue = (sensorL+sensorM+sensorR)/3;
    
    if (debug) {
        Serial.print(F("Arka Plan -> Sol:"));
        Serial.print(sensorL);
        Serial.print(F(" Orta:"));
        Serial.print(sensorM);
        Serial.print(F(" Sag:"));
        Serial.print(sensorR);
        Serial.print(F(" Ort:"));
        Serial.println(backgroundDetectionValue);
    }
    
    burnLed(green);
    delay(100);
    burnLed(black);
    delay(100);
    burnLed(green);
    delay(100);
    burnLed(black);
    delay(100);

    sensorL = 0;
    sensorM = 0;
    sensorR = 0;
    int sensorLV = 1;
    int sensorMV = 1;
    int sensorRV = 1;
    burnLed(blue);
    delay(2000);
    burnLed(green);
    
    for (int i = 0; i < 30; i++) {
        int L_sensorL = analogRead(PIN_irLeft);
        int L_sensorM = analogRead(PIN_irMiddle);
        int L_sensorR = analogRead(PIN_irRight);

        if (backgroundDetectionValue <= 512) {
            if (L_sensorL > backgroundDetectionValue) {
                sensorL += L_sensorL;
                sensorLV++;
            }
            if (L_sensorM > backgroundDetectionValue) {
                sensorM += L_sensorM;
                sensorMV++;
            }
            if (L_sensorR > backgroundDetectionValue) {
                sensorR += L_sensorR;
                sensorRV++;
            }
        }
        else {
            if (L_sensorL < backgroundDetectionValue) {
                sensorL += L_sensorL;
                sensorLV++;
            }
            if (L_sensorM < backgroundDetectionValue) {
                sensorM += L_sensorM;
                sensorMV++;
            }
            if (L_sensorR < backgroundDetectionValue) {
                sensorR += L_sensorR;
                sensorRV++;
            }
        }
        delay(30);
    }
    
    sensorL /= sensorLV;
    sensorM /= sensorMV;
    sensorR /= sensorRV;
    localLineDetectionValue = (sensorL+sensorM+sensorR)/3;
    
    if (debug) {
        Serial.print(F("Cizgi -> Sol:"));
        Serial.print(sensorL);
        Serial.print(F(" Orta:"));
        Serial.print(sensorM);
        Serial.print(F(" Sag:"));
        Serial.print(sensorR);
        Serial.print(F(" Ort:"));
        Serial.println(localLineDetectionValue);
    }
    
    burnLed(green);
    delay(100);
    burnLed(black);
    delay(100);
    burnLed(green);
    delay(100);
    burnLed(black);
    delay(100);
    
    isCalibrated = true;
    lineDetectionValue = abs((backgroundDetectionValue+localLineDetectionValue))/2;
    if (lineDetectionValue>backgroundDetectionValue) isReversedLines = true;
    digitalWrite(PIN_motorEN, 1);
    
    if (debug) {
        Serial.print(F("Esik Deger:"));
        Serial.print(lineDetectionValue);
        Serial.print(F(" | Ters Mod:"));
        Serial.println(isReversedLines ? "EVET" : "HAYIR");
    }
}

void setPIDParameters(float kp, float ki, float kd, int baseSpeed) {
    g_Kp = kp;
    g_Ki = ki;
    g_Kd = kd;
    g_baseSpeed = baseSpeed;
}

void blindDetection() {
    static bool logged = false;
    if (!logged && debug) {
        Serial.println(F("\n>>> blindDetection()"));
        logged = true;
    }
    
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
        logged = false;
    }
}

void controlStages() {
    switch (currentStage) {
        case DEPARTURE:
            followLine();
            break;
        case DISCHARGE:
            if (debug) Serial.println(F("\n>>> DISCHARGE"));
            isNewReturn = true;
            isStop = true;
            goTo(stop_it, 0);
            burnLed(yellow);
            delay(2000);
            burnLed(black);
            currentStage = RETURN;
            isStop = false;
            break;
        case RETURN:
            if (isNewReturn) {
                if (debug) Serial.println(F("\n>>> RETURN Manevrasi"));
                goTo(backward, 100);
                delay(300);
                goTo(right, g_baseSpeed+50);
                delay(1200);
                isNewReturn = false;
            }
            followLine();
        default:
            break;
    }
}

int getSensorPattern(bool Sleft, bool Smiddle, bool Sright) {
    if (!Sleft && Smiddle && Sright) return 1;
    if (Sleft && Smiddle && !Sright) return 2;
    if (Sleft && Smiddle && Sright) return 3;
    return 0;
}

bool isIntersection(bool Sleft, bool Smiddle, bool Sright) {
    int pattern = getSensorPattern(Sleft, Smiddle, Sright);
    
    if (pattern == 0) {
        lastRepeatTime = 0;
        g_lastError = 0;
        return false;
    }
    
    unsigned long currentTime = millis();
    
    if (lastRepeatTime == 0) {
        lastRepeatTime = currentTime;
        g_lastError = pattern;
        if (debug) {
            Serial.print(F("Kavsak Suphesi: Pattern="));
            Serial.println(pattern);
        }
        return false;
    }

    unsigned long repeatTime = currentTime - lastRepeatTime;

    if (repeatTime < 50) {
        return false;
    }

    if (g_lastError != pattern) {
        if (debug) Serial.println(F("Pattern degisti, yanlis alarm"));
        g_lastError = pattern;
        lastRepeatTime = currentTime;
        return false;
    }

    g_intersectionType = pattern;
    lastRepeatTime = 0;
    g_lastError = 0;
    
    if (debug) {
        Serial.print(F("KAVSAK ONAYLANDI! Tur:"));
        Serial.print(pattern);
        Serial.print(F(" Sure:"));
        Serial.println(repeatTime);
    }
    
    return true;
}

void controlIntersection(bool Sleft, bool Smiddle, bool Sright) {
    if (!g_handlingIntersection) {
        g_handlingIntersection = true;
        g_intersectionStartTime = millis();
        g_integral = 0;
        g_previousError = 0;
        g_lastPIDTime = 0;
        g_blindActive = false;
        
        if (debug) Serial.println(F("\n>>> controlIntersection() BASLADI"));
    }
    
    unsigned long elapsed = millis() - g_intersectionStartTime;
    int pattern = getSensorPattern(Sleft, Smiddle, Sright);
    
    if (debug && delaySenc(TIMER_DEBUG, 300)) {
        Serial.print(F("Kavsak Tur:"));
        Serial.print(g_intersectionType);
        Serial.print(F(" Gecen:"));
        Serial.print(elapsed);
        Serial.print(F("ms Pattern:"));
        Serial.println(pattern);
    }
    
    switch (g_intersectionType) {
        case 1:
            handleRightIntersection(elapsed, pattern);
            break;
        case 2:
            handleLeftIntersection(elapsed, pattern);
            break;
        case 3:
            handleTIntersection(elapsed, pattern);
            break;
    }
}

void handleRightIntersection(unsigned long elapsed, int currentPattern) {
    reactionTypes reaction = lineReactions[0].reaction;
    if (reaction == AUTO) reaction = RIGHT;
    
    switch (reaction) {
        case RIGHT:
            if (elapsed < 150) {
                goTo(forward, g_baseSpeed);
            }
            else if (elapsed < 600) {
                goTo(right, g_baseSpeed + 20);
            }
            else {
                checkTurnComplete(currentPattern);
            }
            break;
            
        case LEFT:
            if (elapsed < 150) {
                goTo(forward, g_baseSpeed);
            }
            else if (elapsed < 600) {
                goTo(left, g_baseSpeed + 20);
            }
            else {
                checkTurnComplete(currentPattern);
            }
            break;
            
        case MID:
            if (elapsed < 300) {
                goTo(forward, g_baseSpeed);
            }
            else {
                checkTurnComplete(currentPattern);
            }
            break;
            
        case STOP:
            goTo(stop_it, 0);
            if (elapsed > 2000) {
                g_handlingIntersection = false;
                g_intersectionType = 0;
            }
            break;
            
        default:
            break;
    }
}

void handleLeftIntersection(unsigned long elapsed, int currentPattern) {
    reactionTypes reaction = lineReactions[1].reaction;
    if (reaction == AUTO) reaction = LEFT;
    
    switch (reaction) {
        case LEFT:
            if (elapsed < 150) {
                goTo(forward, g_baseSpeed);
            }
            else if (elapsed < 600) {
                goTo(left, g_baseSpeed + 20);
            }
            else {
                checkTurnComplete(currentPattern);
            }
            break;
            
        case RIGHT:
            if (elapsed < 150) {
                goTo(forward, g_baseSpeed);
            }
            else if (elapsed < 600) {
                goTo(right, g_baseSpeed + 20);
            }
            else {
                checkTurnComplete(currentPattern);
            }
            break;
            
        case MID:
            if (elapsed < 300) {
                goTo(forward, g_baseSpeed);
            }
            else {
                checkTurnComplete(currentPattern);
            }
            break;
            
        case STOP:
            goTo(stop_it, 0);
            if (elapsed > 2000) {
                g_handlingIntersection = false;
                g_intersectionType = 0;
            }
            break;
            
        default:
            break;
    }
}

void handleTIntersection(unsigned long elapsed, int currentPattern) {
    reactionTypes reaction = lineReactions[2].reaction;
    if (reaction == AUTO) reaction = MID;
    
    switch (reaction) {
        case LEFT:
            if (elapsed < 200) {
                goTo(forward, g_baseSpeed);
            }
            else if (elapsed < 700) {
                goTo(left, g_baseSpeed + 30);
            }
            else {
                checkTurnComplete(currentPattern);
            }
            break;
            
        case RIGHT:
            if (elapsed < 200) {
                goTo(forward, g_baseSpeed);
            }
            else if (elapsed < 700) {
                goTo(right, g_baseSpeed + 30);
            }
            else {
                checkTurnComplete(currentPattern);
            }
            break;
            
        case MID:
            if (elapsed < 400) {
                goTo(forward, g_baseSpeed);
            }
            else {
                checkTurnComplete(currentPattern);
            }
            break;
            
        case STOP:
            goTo(stop_it, 0);
            if (elapsed > 2000) {
                g_handlingIntersection = false;
                g_intersectionType = 0;
            }
            break;
            
        default:
            break;
    }
}

void checkTurnComplete(int currentPattern) {
    if (currentPattern == 0) {
        g_handlingIntersection = false;
        g_intersectionType = 0;
        if (debug) Serial.println(F("Kavsak TAMAMLANDI"));
    }
    else if (millis() - g_intersectionStartTime > 1500) {
        g_handlingIntersection = false;
        g_intersectionType = 0;
        if (debug) Serial.println(F("Kavsak TIMEOUT"));
    }
}

void followLine() {
    static bool pidStarted = false;
    
    if (!isCalibrated || !isCarOnGround || isStop || currentStage == DISCHARGE) {
        goTo(stop_it, 0);
        g_integral = 0;
        g_previousError = 0;
        g_lastPIDTime = 0;
        g_blindActive = false;
        pidStarted = false;
        return;
    }
    
    if (!pidStarted && debug) {
        Serial.println(F("\n>>> followLine()"));
        pidStarted = true;
    }
    
    int sensorL = analogRead(PIN_irLeft);
    int sensorM = analogRead(PIN_irMiddle);
    int sensorR = analogRead(PIN_irRight);
    
    int Sleft = !isReversedLines ? (sensorL < lineDetectionValue) : (sensorL >= lineDetectionValue);
    int Smiddle = !isReversedLines ? (sensorM < lineDetectionValue) : (sensorM >= lineDetectionValue);
    int Sright = !isReversedLines ? (sensorR < lineDetectionValue) : (sensorR >= lineDetectionValue);
    
    if (!Sleft && !Smiddle && !Sright) {
        g_integral = 0;
        g_previousError = 0;
        g_lastPIDTime = 0;
        g_slowDownForIntersection = false;
        
        if (!g_blindActive) {
            g_blindActive = true;
            g_blindStartTime = millis();
        }
        blindDetection();
        return;
    }
    
    g_blindActive = false;
    
    if (g_handlingIntersection) {
        controlIntersection(Sleft, Smiddle, Sright);
        return;
    }
    
    int pattern = getSensorPattern(Sleft, Smiddle, Sright);
    
    if (pattern != 0 && !g_slowDownForIntersection) {
        g_slowDownForIntersection = true;
    }
    
    if (g_slowDownForIntersection) {
        goTo(forward, g_baseSpeed * 0.4);
        
        if (isIntersection(Sleft, Smiddle, Sright)) {
            g_slowDownForIntersection = false;
            controlIntersection(Sleft, Smiddle, Sright);
            return;
        }
        
        if (pattern == 0) {
            g_slowDownForIntersection = false;
        }
        
        return;
    }

    float totalWeight = Sleft + Smiddle + Sright;
    float error = (-1.0 * Sleft + 0 * Smiddle + 1.0 * Sright) / totalWeight;
    
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

    if (debug && delaySenc(TIMER_DEBUG, 400)) {
        Serial.print(F("PID-> Sol:"));
        Serial.print(sensorL);
        Serial.print(F(" Orta:"));
        Serial.print(sensorM);
        Serial.print(F(" Sag:"));
        Serial.print(sensorR);
        Serial.print(F(" | MotorL:"));
        Serial.print(leftSpeed);
        Serial.print(F(" MotorR:"));
        Serial.println(rightSpeed);
    }

    goTo(forward, 0, leftSpeed, rightSpeed);
}


void checkESPCommands() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            // Prefix kontrolü
            if (receivedCommand.startsWith("ESP:")) {
                // ESP'den gelen komut
                receivedCommand.remove(0, 4); // "ESP:" kısmını çıkar
                processCommand(receivedCommand);
            }
            // Prefix yoksa Arduino'nun kendi verisi, işleme gerek yok
            receivedCommand = "";
        } else {
            receivedCommand += c;
        }
    }
}

void processCommand(String cmd) {
    cmd.trim();
    
    if (cmd == "START") {
        isStop = false;
        Serial.println("[ARD] Robot BASLATILDI");  // ESP'ye geri bildirim
        if (debug) Serial.println(F("\n[KOMUT] Robot BASLATILDI"));
    }
    else if (cmd == "STOP") {
        isStop = true;
        goTo(stop_it, 0);
        Serial.println("[ARD] Robot DURDURULDU");  // ESP'ye geri bildirim
        if (debug) Serial.println(F("\n[KOMUT] Robot DURDURULDU"));
    }
    else if (cmd == "CALIBRATE") {
        buttonMode = 2;
        Serial.println("[ARD] Kalibrasyon basladi");  // ESP'ye geri bildirim
        if (debug) Serial.println(F("\n[KOMUT] Kalibrasyon baslatiyor"));
    }
    else if (cmd == "RESET") {
        g_integral = 0;
        g_previousError = 0;
        g_lastPIDTime = 0;
        Serial.println("[ARD] PID sifirlandi");  // ESP'ye geri bildirim
        if (debug) Serial.println(F("\n[KOMUT] PID sifirlandi"));
    }
    else if (cmd == "STATUS") {
        Serial.println("[ARD] === DURUM RAPORU ===");
        Serial.print("[ARD] Kalibre: "); Serial.println(isCalibrated ? "EVET" : "HAYIR");
        Serial.print("[ARD] Zemin: "); Serial.println(isCarOnGround ? "YERDE" : "HAVADA");
        Serial.print("[ARD] Stop: "); Serial.println(isStop ? "DURDURULDU" : "CALISIYOR");
        Serial.print("[ARD] PID -> Kp:"); Serial.print(g_Kp);
        Serial.print(" Ki:"); Serial.print(g_Ki);
        Serial.print(" Kd:"); Serial.println(g_Kd);
        
        if (debug) {
            Serial.println(F("\n[DURUM RAPORU]"));
            Serial.print(F("Kalibre: ")); Serial.println(isCalibrated ? "EVET" : "HAYIR");
            Serial.print(F("Zemin: ")); Serial.println(isCarOnGround ? "YERDE" : "HAVADA");
            Serial.print(F("Stop: ")); Serial.println(isStop ? "DURDURULDU" : "CALISIYOR");
            Serial.print(F("PID Kp:")); Serial.print(g_Kp);
            Serial.print(F(" Ki:")); Serial.print(g_Ki);
            Serial.print(F(" Kd:")); Serial.println(g_Kd);
        }
    }
    else if (cmd.startsWith("PID:")) {
        cmd.remove(0, 4);
        
        int firstComma = cmd.indexOf(',');
        int secondComma = cmd.indexOf(',', firstComma + 1);
        int thirdComma = cmd.indexOf(',', secondComma + 1);
        
        if (firstComma > 0 && secondComma > 0 && thirdComma > 0) {
            float newKp = cmd.substring(0, firstComma).toFloat();
            float newKi = cmd.substring(firstComma + 1, secondComma).toFloat();
            float newKd = cmd.substring(secondComma + 1, thirdComma).toFloat();
            int newBase = cmd.substring(thirdComma + 1).toInt();
            
            setPIDParameters(newKp, newKi, newKd, newBase);
            
            Serial.print("[ARD] PID guncellendi -> Kp:"); Serial.print(g_Kp);
            Serial.print(" Ki:"); Serial.print(g_Ki);
            Serial.print(" Kd:"); Serial.print(g_Kd);
            Serial.print(" Base:"); Serial.println(g_baseSpeed);
            
            if (debug) {
                Serial.println(F("\n[KOMUT] PID guncellendi"));
                Serial.print(F("Kp:")); Serial.print(g_Kp);
                Serial.print(F(" Ki:")); Serial.print(g_Ki);
                Serial.print(F(" Kd:")); Serial.print(g_Kd);
                Serial.print(F(" Base:")); Serial.println(g_baseSpeed);
            }
        }
    }
}


void setup() {
    Serial.begin(115200);
    buttonMode = 1;
    PinInit();
    myServo.write(90);
    
    if (debug) Serial.println(F("Sistem basladi"));
}

void loop() {
    checkESPCommands();

    blinkLed(isCalibrated, red, 500);
    
    if (delaySenc(TIMER_ULTRASONIC, 70) && !isStop && isCarOnGround && isCalibrated && currentStage == DEPARTURE) {
        long distance = readUltrasonicDistance();
        g_lastDistance = distance;
        
        if (distance < 5) {
            if (debug) Serial.println(F("\nHEDEF ALGILANDI!"));
            myServo.write(0);
            currentStage = DISCHARGE;
            isStop = true;
        }
    }
    
    controlCarGround();
    
    if (!isStop && isCarOnGround && currentStage == RETURN) {
        myServo.write(90);
    }
    
    if (!digitalRead(PIN_button) && delaySenc(TIMER_BUTTON, 70)) {
        if (buttonMode < 2) buttonMode += 1;
        else buttonMode = 1;
    }

    switch (buttonMode) {
        case 1:
            controlStages();
            break;
        case 2:
            calibrateSensors();
            buttonMode = 1;
            break;
    }
}