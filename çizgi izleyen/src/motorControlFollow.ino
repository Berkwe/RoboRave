#include <Arduino.h>
#include <FastLED.h>
#include <Servo.h>


Servo myServo;

#define PIN_trig 13 // trig mesafe sensoru
#define PIN_echo 12 // echo mesafe sensoru
#define PIN_servo 10 // servo pini
#define PIN_irLeft A2 // sol ir sensörü
#define PIN_irMiddle A1 // orta ir sensörü
#define PIN_irRight A0 //  sağ ir sensörü

#define PIN_motorRightSpeed 5 // sağ motor PWM
#define PIN_motorLeftSpeed 6 // sol motor PWM
#define PIN_motorRight 7 // Sağ motor yön
#define PIN_motorLeft 8 // Sol motor yön
#define PIN_motorEN 3 // Motor sürücüsü enable pini

#define PIN_button 2 // d2 Button pini

#define PIN_led 4 // d4 RGB led pini

// renkler
CRGB leds[1];
// kalibrasyon
CRGB red = CRGB(255,0,0); // kalibrasyon uyarısı rengi
CRGB blue = CRGB(0,0,255); // kalibrasyon çizgi algılama rengi
CRGB white = CRGB(255,255,255); // kalbirasyon arka plan algılama rengi
CRGB green = CRGB(0,255,0); // tamamlanma rengi
// uyarılar
CRGB cyan = CRGB(0,255,255); // robot havada rengi
CRGB yellow = CRGB(255,255,0); // çizgiye yerleştirin rengi
// diğerleri
CRGB black = CRGB(0,0,0); // led kapatma rengi


#define speedMax 130 // en fazla hız
#define minGroundDistance 1000 // robotun havada varsayılması için gereken en düşük değer

bool isReversedLines = false; // çizginin arka plandan daha açık olduğunu belirten flag
int lineDetectionValue = 700; // (|arka_plan_değeri+çizgi_değeri|)/2 böylece dinamik doğru bi dğeer atanır
bool isStop = false;


int buttonMode = 1; // mod değeri

bool isCalibrated = false; // kalibre edildi mi
bool isCarOnGround = false; // robot yerde mi


// PID değişkenleri
float g_Kp = 65.0;  
float g_Ki = 0.12; 
float g_Kd = 18.0; 
float g_integral = 0;
float g_previousError = 0;
unsigned long g_lastPIDTime = 0;
int g_baseSpeed = 50;

// Blind detection değişkenleri
unsigned long g_blindStartTime = 0;
bool g_blindActive = false;
float g_lastError = 0;

// ultrasonik sensor için 
unsigned long g_triggerTime = 0;
bool g_waitingEcho = false;
long g_lastDistance = 100;


bool debug = false; 

// Otomatik timer yönetimi için
enum TimerID {
    TIMER_LED,
    TIMER_BUTTON,
    TIMER_LINE_DETECT,
    TIMER_TURN,

    TIMER_SERVO,
    TIMER_ULTRASONIC,
    TIMER_DEBUG,
    TIMER_COUNT  // Toplam timer sayısı
};
unsigned long g_timers[TIMER_COUNT] = {0};

// yönler
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

// modlar
enum mods {
    lineFollowing = 1,
    calibrating = 2
};


enum followModes {
    AUTO,
    MANUEL // manuel dönüş
};

// map vars
// int mapFollow[] = {0, -1}; // -1 sol, 0 orta, 1 sağ
//followModes followMode = MANUEL;
//int currentMapSection = 0;

// Stages vars
enum stages { // 3 bölüm
    DEPARTURE,
    DISCHARGE,
    RETURN
};
stages currentStage = DEPARTURE;
bool isNewReturn = true;



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
        if (buttonMode == 1 && isCalibrated) { // havada
            isStop = false;
            digitalWrite(PIN_motorEN, 0); // motru kapat
            currentStage = DEPARTURE;
            //currentMapSection = 0;
            burnLed(cyan);
        }
    }
    else { // yerde
        isCarOnGround = true;
        if (buttonMode == 1 && isCalibrated) {
            digitalWrite(PIN_motorEN, 1);
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
    // servo initi
    myServo.attach(10);
    // mesafe sensörü initi
    pinMode(PIN_trig, OUTPUT);
    pinMode(PIN_echo, INPUT);
}


void goTo(directions direction, uint8_t speed = 0, uint8_t speedRight = 0, uint8_t speedLeft = 0) { // motoru hareket ettirir
    if (!isCalibrated || !isCarOnGround) return;
    
    speed = constrain(speed, 0, speedMax);
    switch (direction)
    {
    case forward:
        digitalWrite(PIN_motorRight, 1);
        analogWrite(PIN_motorRightSpeed, (speed == 0 ? speedRight : speed));
        digitalWrite(PIN_motorLeft, 1);
        analogWrite(PIN_motorLeftSpeed, speed == 0 ? speedLeft : speed);
        break;
    case backward:
        digitalWrite(PIN_motorRight, 0);
        analogWrite(PIN_motorRightSpeed, (speed == 0 ? speedRight : speed));
        digitalWrite(PIN_motorLeft, 0);
        analogWrite(PIN_motorLeftSpeed, speed == 0 ? speedLeft : speed);
        break;
    case left:
        digitalWrite(PIN_motorRight, 1);
        analogWrite(PIN_motorRightSpeed, (speed == 0 ? speedRight : speed));
        digitalWrite(PIN_motorLeft, 0);
        analogWrite(PIN_motorLeftSpeed, speed == 0 ? speedLeft : speed);
        break;
    case right:
        digitalWrite(PIN_motorRight, 0);
        analogWrite(PIN_motorRightSpeed, (speed == 0 ? speedRight : speed));
        digitalWrite(PIN_motorLeft, 1);
        analogWrite(PIN_motorLeftSpeed, speed == 0 ? speedLeft : speed);
        break;
    case leftForward:
        digitalWrite(PIN_motorRight, 1);
        analogWrite(PIN_motorRightSpeed, (speed == 0 ? speedRight : speed));
        analogWrite(PIN_motorLeftSpeed, 0);
        break;
    case leftBackward:
        digitalWrite(PIN_motorRight, 0);
        analogWrite(PIN_motorRightSpeed, (speed == 0 ? speedRight : speed));
        analogWrite(PIN_motorLeftSpeed, 0);
        break;
    case rightForward:
        analogWrite(PIN_motorRightSpeed, 0);
        digitalWrite(PIN_motorLeft, 1);
        analogWrite(PIN_motorLeftSpeed, speed == 0 ? speedLeft : speed);
        break;
    case rightBackward:
        analogWrite(PIN_motorRightSpeed, 0);
        digitalWrite(PIN_motorLeft, 0);
        analogWrite(PIN_motorLeftSpeed, speed == 0 ? speedLeft : speed);
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


void calibrateSensors() { // İr sensörleri arka planla kalibre eder ivmeölçeri falan da aynı şekilde
    digitalWrite(PIN_motorEN, 0);
    burnLed(white);
    delay(2000);
    if (debug) Serial.println("Kalibrasyon başladı");
    unsigned long sensorL = analogRead(PIN_irLeft);
    unsigned long sensorM = analogRead(PIN_irMiddle);
    unsigned long sensorR = analogRead(PIN_irRight);
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

    // çizgi kalibrasyonu 50
    sensorL = 0;
    sensorM = 0;
    sensorR = 0;
    int sensorLV = 1;
    int sensorMV = 1;
    int sensorRV = 1;
    burnLed(blue);
    delay(2000);
    burnLed(green);
    for (int i = 0; i < 30; i++)
        {
            int L_sensorL = analogRead(PIN_irLeft);
            int L_sensorM = analogRead(PIN_irMiddle);
            int L_sensorR = analogRead(PIN_irRight);

            // önce arka planın koyu mu açık mı olduğu tespit ediliyor sonrasında okunan değerin arka plana göre çizgi olup olmadığını algılayıp kaydediyor

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
    lineDetectionValue = abs((backgroundDetectionValue+localLineDetectionValue))/2;
    if (lineDetectionValue>backgroundDetectionValue) isReversedLines = true; // eğer çizgi açık bir renk ise
    digitalWrite(PIN_motorEN, 1);
    if (debug) {
        Serial.println("\narka plan : ");
        Serial.print(backgroundDetectionValue);
        Serial.println("\nçizgi : ");
        Serial.print(localLineDetectionValue);
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
/*
unsigned long g_turnStartTime = 0;
bool g_turningActive = false;
int g_turnDirection = 0; // -1: sol, 0: düz, 1: sağ

void followMap() {
    int currentMap = mapFollow[currentMapSection];
    burnLed(yellow);
    if (!g_turningActive) {
        // Dönüş başlangıcı
        g_turningActive = true;
        g_turnStartTime = millis();
        g_turnDirection = currentMap;
        currentMapSection++;
        
        // PID sıfırlama
        g_integral = 0;
        g_previousError = 0;
        g_lastPIDTime = 0;
    }
    
    // Sensör okumaları
    int sensorL = analogRead(PIN_irLeft);
    int sensorM = analogRead(PIN_irMiddle);
    int sensorR = analogRead(PIN_irRight);
    
    int Sleft = !isReversedLines ? (sensorL < lineDetectionValue) : (sensorL >= lineDetectionValue);
    int Smiddle = !isReversedLines ? (sensorM < lineDetectionValue) : (sensorM >= lineDetectionValue);
    int Sright = !isReversedLines ? (sensorR < lineDetectionValue) : (sensorR >= lineDetectionValue);
    
    // Minimum dönüş süresi kontrolü (kavşaktan çıkmak için)
    unsigned long elapsed = millis() - g_turnStartTime;
    if (elapsed < 200) {
        // Kavşaktan çıkana kadar belirli yöne dön
        if (g_turnDirection == -1) {
            goTo(left, g_baseSpeed);
        } else if (g_turnDirection == 1) {
            goTo(right, g_baseSpeed);
        } else {
            goTo(forward, g_baseSpeed);
        }
        return;
    }
    
    // Çizgi tespit edildi mi kontrol et
    bool lineDetected = false;
    
    if (g_turnDirection == -1) { // Sol dönüş
        // Orta veya sol sensörde çizgi varsa
        if (Smiddle || Sleft) {
            lineDetected = true;
        } else {
            goTo(left, g_baseSpeed);
        }
    } 
    else if (g_turnDirection == 1) { // Sağ dönüş
        // Orta veya sağ sensörde çizgi varsa
        if (Smiddle || Sright) {
            lineDetected = true;
        } else {
            goTo(right, g_baseSpeed);
        }
    } 
    else { // Düz git (g_turnDirection == 0)
        // Orta sensörde çizgi varsa
        if (Smiddle) {
            lineDetected = true;
        } else {
            goTo(forward, g_baseSpeed);
        }
    }
    
    // Çizgi bulundu, normal takibe dön
    if (lineDetected) {
        g_turningActive = false;
        g_turnDirection = 0;
        // PID'yi sıfırla
        g_integral = 0;
        g_previousError = 0;
        g_lastPIDTime = 0;
    }
    burnLed(black);
}
*/

void controlStages() {

        if (debug) {
        Serial.print("[controlStages] Aktif aşama: ");
        Serial.println(currentStage == DEPARTURE ? "DEPARTURE" : 
                      currentStage == DISCHARGE ? "DISCHARGE" : "RETURN");
    }
    switch (currentStage)
    {
    case DEPARTURE:
        followLine();
        break;
    case DISCHARGE:
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



void followLine() { // çizgi takibi
    if (!isCalibrated || !isCarOnGround || isStop || currentStage == DISCHARGE) {
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

    
    int Sleft = !isReversedLines ? (sensorL < lineDetectionValue) : (sensorL >= lineDetectionValue);
    int Smiddle = !isReversedLines ? (sensorM < lineDetectionValue) : (sensorM >= lineDetectionValue);
    int Sright = !isReversedLines ? (sensorR < lineDetectionValue) : (sensorR >= lineDetectionValue);
    
    /*if (debug) {
        Serial.print("SensorL: "); Serial.print(sensorL);
        Serial.print("  Left: "); Serial.print(Sleft);
        Serial.print(" | SensorM: "); Serial.print(sensorM);
        Serial.print("  Middle: "); Serial.print(Smiddle);
        Serial.print(" | SensorR: "); Serial.print(sensorR);
        Serial.print("  Right: "); Serial.println(Sright);
    }
 */
    if (!Sleft && !Smiddle && !Sright) {
        if (debug) Serial.println("[followLine] Çizgi kaybedildi, blindDetection aktif");
        g_integral = 0;
        g_previousError = 0;
        g_lastPIDTime = 0;
        
        if (!g_blindActive) {
            g_blindActive = true;
            g_blindStartTime = millis();
        }
        blindDetection();
        return;
    }
    
    g_blindActive = false;
/*
    if (Sleft && Smiddle && Sright && followMode == MANUEL) {
        g_integral = 0;
        g_previousError = 0;
        g_lastPIDTime = 0;
        followMap();
        return;
    }
*/
    // -1 sol 1 sağ
    float totalWeight = Sleft + Smiddle + Sright;
    float error = (-1.0 * Sleft + 0 * Smiddle + 1.0 * Sright) / totalWeight;


    unsigned long currentTime = millis();
    if (g_lastPIDTime == 0) {
        g_lastPIDTime = currentTime;
        g_previousError = error;
        goTo(forward, g_baseSpeed);
        return;
    }

    // P terimi orantısal hata
    float dt = (currentTime - g_lastPIDTime) / 1000.0;
    if (dt <= 0 || dt > 1.0) dt = 0.02; // dt önceki PID dögüsünden bi sonraki döngüye kadar ne kadar zaman geçtiğini hesaplıyor
    g_lastPIDTime = currentTime;

    float P = g_Kp * error; // o anki hatayı orantılı düzeltiyor. mesela -0.5 (çizginin hafif solunda) gibi bir hata var kp = 45.0 P = -22.5 oluyor robot hesaplanan hızda düzelene kadar sola gider
    
    
    // i terimi INTEGRAL
    if (abs(error) < 0.01) g_integral = 0; // o ana kadarki yapılan tüm hatalara göre düzeltiyor
    else {
        g_integral += error * dt;
        g_integral = constrain(g_integral, -50.0, 50.0);
    }
    float I = g_Ki * g_integral;
    // mesela Ki = 0.08 diyelim error 3 döngüde sırayla 0.5 1 -0.5 gibi değerler alsın dt ye de 0.05 diyelim (50 ms)
    // g_integral = (0.05 * 0.5) + (0.05 * 1) + (0.05 * -0.5) = 0.0725 bu sayıyı -50 ve 50 yi geçmiyecek şekilde sınırlandııryoruz
    // I = 0.08 * 0.0725 = 0,0058


    // D terimi TÜREV
    float derivative = (error - g_previousError) / dt; // hatanın ne kadar hızlı düzeldiğini hesaplayıp ona göre düzeltme yapar
    float D = g_Kd * derivative;
    g_previousError = error;
    // mesela dt = 0.05 (50 ms) Kd = 12.0, error = 1 (tam sağda), prevError = 0 (tam ortada) olsun 
    // derivative = (1 - 0) / 0.05 = 20 olur
    // D = 12 * 20 = 240 olur
    // Sonraki döngüde dt = 0.05, error = 0.5, prevError = 1 diyelim
    // derivative = (0.5 - 1) / 0.05 = -10 
    // D = 12 * -10 = -120 olur
    
    // PID
    int correction = P+I+D;

    // motor hızları PID ye göre hesaolanıyor
    int  leftSpeed = g_baseSpeed - correction;
    int  rightSpeed = g_baseSpeed + correction;

    leftSpeed = constrain(leftSpeed, 0, speedMax);
    rightSpeed = constrain(rightSpeed, 0, speedMax);


    if (debug) {
        Serial.print("[followLine] PID -> Error: ");
        Serial.print(error);
        Serial.print(" | P: ");
        Serial.print(P);
        Serial.print(" | I: ");
        Serial.print(I);
        Serial.print(" | D: ");
        Serial.print(D);
        Serial.print(" | Düzeltme: ");
        Serial.print(correction);
        Serial.print(" | Sol hız: ");
        Serial.print(leftSpeed);
        Serial.print(" | Sağ hız: ");
        Serial.println(rightSpeed);
    }

    goTo(forward, 0, leftSpeed, rightSpeed);
}   


void setup() {
    Serial.begin(115200);
    buttonMode = 1;
    PinInit();
    myServo.write(90);
    if (debug) {
        burnLed(blue);
        delay(1500);
        burnLed(black);
        isCalibrated = true;
        isCarOnGround = true;
    }
}


void loop() {
    blinkLed(isCalibrated, red, 500); // kalibre edilmediyse kalır
    
    if (delaySenc(TIMER_ULTRASONIC, 70) && !isStop && isCarOnGround && isCalibrated && currentStage == DEPARTURE) { // Kule kontrolü 
        if (readUltrasonicDistance() < 5) {
                myServo.write(0);
                currentStage = DISCHARGE;
                isStop = true;
            }
    }
    controlCarGround(); // yerde mi kontrolü

    if (!isStop && isCarOnGround && currentStage == RETURN) { // servo 1 düzeltme
            myServo.write(90);
    }
    
    if (!digitalRead(PIN_button) && delaySenc(TIMER_BUTTON, 70)) { // mod atama
        if (buttonMode < 2) buttonMode += 1;
        else buttonMode = 1;
    }

    // Mod kontrolü
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