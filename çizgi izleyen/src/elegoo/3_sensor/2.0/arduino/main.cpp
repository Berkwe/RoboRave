#include <Arduino.h>
#include <FastLED.h>
#include <Servo.h>


Servo myServo;

#define PIN_trig 13 // ? trig mesafe sensoru
#define PIN_echo 12 // ? echo mesafe sensoru
#define PIN_servo 10 // ? servo pini
#define PIN_irLeft A2 // ? sol ir sensörü
#define PIN_irMiddle A1 // ? orta ir sensörü
#define PIN_irRight A0 // ?  sağ ir sensörü

#define PIN_motorRightSpeed 5 // ? sağ motor PWM
#define PIN_motorLeftSpeed 6 // ? sol motor PWM
#define PIN_motorRight 7 // ? Sağ motor yön
#define PIN_motorLeft 8 // ? Sol motor yön
#define PIN_motorEN 3 // ? Motor sürücüsü enable pini

#define PIN_button 2 // ? d2 Button pini

#define PIN_led 4 // ? d4 RGB led pini
#define PIN_irLedPWM 11 // ? alttaki led pini

// * renkler
CRGB leds[1];
// * kalibrasyon
CRGB red = CRGB(255,0,0); // ? kalibrasyon uyarısı rengi
CRGB blue = CRGB(0,0,255); // ? kalibrasyon çizgi algılama rengi
CRGB white = CRGB(255,255,255); // ? kalbirasyon arka plan algılama rengi
CRGB green = CRGB(0,255,0); // ? tamamlanma rengi
// * uyarılar
CRGB cyan = CRGB(0,255,255); // ? robot havada rengi
CRGB yellow = CRGB(255,255,0); // ? çizgiye yerleştirin rengi
// * diğerleri
CRGB black = CRGB(0,0,0); // ? led kapatma rengi


#define speedMax 130 // ? en fazla hız
#define minGroundDistance 1000 // ? robotun havada varsayılması için gereken en düşük değer

bool isReversedLines = false; // ? çizginin arka plandan daha açık olduğunu belirten flag
int lineDetectionValue = 700; // ? (|arka_plan_değeri+çizgi_değeri|)/2 böylece dinamik doğru bi dğeer atanır
bool isStop = false;


int buttonMode = 1; // ? mod değeri

bool isCalibrated = false; // ? kalibre edildi mi
bool isCarOnGround = false; // ? robot yerde mi


// * PID değişkenleri
float g_Kp = 90.0;
float g_Ki = 0.16;
float g_Kd = 24.0;
float g_integral = 0;
float g_previousError = 0;
unsigned long g_lastPIDTime = 0;
int g_baseSpeed = 50;

// * Blind detection değişkenleri
unsigned long g_blindStartTime = 0;
unsigned long g_blindFilterStartTime = 0;
bool g_blindActive = false;
bool controlBlind = true;

// * ultrasonik sensor için 
unsigned long g_triggerTime = 0;
bool g_waitingEcho = false;
long g_lastDistance = 100;


bool debug = false; 

// * Otomatik timer yönetimi için
enum TimerID {
    TIMER_LED,
    TIMER_BUTTON,
    TIMER_LINE_DETECT,
    TIMER_TURN,

    TIMER_SERVO,
    TIMER_ULTRASONIC,
    TIMER_DEBUG,
    TIMER_COUNT  // ? Toplam timer sayısı
};
unsigned long g_timers[TIMER_COUNT] = {0};

// * yönler ve goto değişkenleri
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

unsigned long goDirectionStartTime = 0;
float goDirectionElapsedTime = 0;
bool ifFirstDirection = true;


// * modlar
enum mods {
    lineFollowing = 1,
    calibrating = 2
};

// * kavşak karar mekanizması değişkenler (YENİ)
enum reactionTypes { //  ? kavşak algılandığındaki tepki türleri, AUTO çizgiye göre belirler. Left, Mid veya Right ise statik karar saglar
    AUTO,
    STOP,
    LEFT,
    MID,
    RIGHT
};

struct lineReactionsStruct {
    String state;
    reactionTypes reaction;
};
unsigned long lastInterSectionTime = 0;

// * kavşak ayarları (YENİ)
lineReactionsStruct lineReactions[] = {
    {"RightInterSection", AUTO}, // ? sağda kavşak
    {"LeftInterSection", AUTO}, // ? solda kavşak
    {"TInterSection", RIGHT} // ? T kavşak
};
bool manuelInterSectionMode = true; // ? kavşakları ellemi belirleyeceğimiz yoksa robotu kendi haline mi bırakacağımız etkileyen flag
int interSectionCoolDown = 3; // ? bir kavşak algılandıkdan sonraki kavşağın algılanmsı için geçmesi gereken minimum süre sn cinsinden
int minActiveSensorToInterSections = 2; // ? bir dönüşün kavşak sayılması için en az kaç tane sensorun aktive olması gerektiğini belirler.
int interSectionsnumSensorReadsFilter = 3; // ? sensorlerin kavşak algılamada kaç kere okunacağı
float interSectionsFilterSleepTime = 0.003; // ? filtreleme için okumalar arası bekleme süresi


// * stage(bölümler) değişkenleri
enum stages { // ? Yarışma 3 bölümden oluşuyor
    DEPARTURE,
    DISCHARGE,
    RETURN
};
stages currentStage = DEPARTURE;
bool isNewReturn = true;

// * stublar
void followLine();

bool getSensorBinary(int analogValue) {
    return !isReversedLines ? (analogValue < lineDetectionValue) : (analogValue >= lineDetectionValue);
}


bool delaySenc(TimerID id, int ms) { // ? senkron biçimde geçikme eklemek için
    unsigned long currentTime = millis();
    if (currentTime - g_timers[id] >= ms) {
        g_timers[id] = currentTime;
        return true;
    }
    return false;
}


void burnLed(CRGB color, int brightness = 255) { // ? led yakmak içn
    if (color == black) FastLED.setBrightness(0);
    else FastLED.setBrightness(255);
    leds[0] = color;
    FastLED.show();
}


void blinkLed(bool var, CRGB color, int interval = 500) { // ? bir değişkene bağlı olarak ledi yakıp söndürür
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
        if (buttonMode == 1 && isCalibrated) { // ? havada
            isStop = false;
            digitalWrite(PIN_motorEN, 0); // ? motru kapat
            currentStage = DEPARTURE;
            // ? currentMapSection = 0;
            burnLed(cyan);
        }
    }
    else { // ? yerde
        isCarOnGround = true;
        if (buttonMode == 1 && isCalibrated) {
            digitalWrite(PIN_motorEN, 1);
            burnLed(black);
        }
    }
    
}


void PinInit() {
    // * motor initi
    pinMode(PIN_motorLeft, OUTPUT);
    pinMode(PIN_motorRight, OUTPUT);
    pinMode(PIN_motorLeftSpeed, OUTPUT);
    pinMode(PIN_motorRightSpeed, OUTPUT);
    pinMode(PIN_motorEN, OUTPUT);
    // * led initi
    FastLED.addLeds<NEOPIXEL, PIN_led>(leds, 1);
    FastLED.setBrightness(255);
    pinMode(PIN_irLedPWM, OUTPUT);
    // * button initi
    pinMode(PIN_button, INPUT_PULLUP);
    // * servo initi
    myServo.attach(10);
    // * mesafe sensörü initi
    pinMode(PIN_trig, OUTPUT);
    pinMode(PIN_echo, INPUT);
}

void driveMotors(int speedLeft, int speedRight) {
    if (speedLeft < 0) digitalWrite(PIN_motorLeft, 0);
    else digitalWrite(PIN_motorLeft, 1);
    if (speedRight < 0) digitalWrite(PIN_motorRight, 0);
    else digitalWrite(PIN_motorRight, 1);
    speedRight = constrain(abs(speedRight), 0, speedMax);
    speedLeft = constrain(abs(speedLeft), 0, speedMax);
    analogWrite(PIN_motorRightSpeed, speedRight);
    analogWrite(PIN_motorLeftSpeed, speedLeft);
}


void goTo(directions direction, uint8_t speed = 0, uint8_t speedRight = 0, uint8_t speedLeft = 0) {
    if (!isCalibrated || !isCarOnGround) {
        return;
    }

    speed = constrain(speed, 0, speedMax);

    if (speedRight != 0 && speedLeft != 0) {
        driveMotors(speedLeft, speedRight);
        return;
    }

    if (direction == stop_it) {
        driveMotors(0, 0);
        g_integral = 0;
        g_previousError = 0;
        g_lastPIDTime = 0;
        g_blindActive = false;
        controlBlind = true;
        goDirectionStartTime = 0;
        goDirectionElapsedTime = 0;
        return;
    }


    goDirectionStartTime = millis();
    ifFirstDirection = true;
    goDirectionElapsedTime = 0;
    controlBlind = false;

    while (goDirectionElapsedTime < 3000) {
        if (isStop) {
            driveMotors(0, 0);
            g_integral = 0;
            g_previousError = 0;
            g_lastPIDTime = 0;
            g_blindActive = false;
            controlBlind = true;
            goDirectionStartTime = 0;
            goDirectionElapsedTime = 0;
            return;
        }

        unsigned long currentTime = millis();
        goDirectionElapsedTime = currentTime - goDirectionStartTime;

        switch (direction) {
            case forward: {
                int sp = (speed == 0 ? g_baseSpeed : speed);
                driveMotors(sp, sp);
                delay(300); // ? yaklaşık 2.5 santim ileri gidiyor hız 130 iken
                driveMotors(0, 0);
                controlBlind = true;
                goDirectionStartTime = 0;
                goDirectionElapsedTime = 0;
                return;
            }

            case left:
                if (ifFirstDirection) {
                    driveMotors((speed == 0 ? g_baseSpeed : speed), (speed == 0 ? g_baseSpeed : speed));
                    delay(150);  // ? biraz ileri
                    
                    driveMotors(-(speed == 0 ? (g_baseSpeed + 20) : speed), (speed == 0 ? g_baseSpeed + 20 : speed));
                    delay(150); // ? bu değerler hisse dayalı veriliyor bu arada base speede ters orantılı olması gerekli
                    ifFirstDirection = false;
                } 
                else {
                    // ? Dönmeye devam
                    driveMotors(-(speed == 0 ? (g_baseSpeed + 10) : speed), (speed == 0 ? g_baseSpeed + 10 : speed));
                }
                break;

            case right:
                if (ifFirstDirection) {
                    driveMotors((speed == 0 ? g_baseSpeed : speed), (speed == 0 ? g_baseSpeed : speed));
                    delay(150);  // ? biraz ileri

                    driveMotors((speed == 0 ? g_baseSpeed + 20 : speed), -(speed == 0 ? (g_baseSpeed + 20) : speed));
                    delay(200);
                    ifFirstDirection = false;
                } 
                else {
                    driveMotors((speed == 0 ? g_baseSpeed + 10 : speed), -(speed == 0 ? (g_baseSpeed + 10) : speed));
                }
                break;

            case backward:
                if (ifFirstDirection) {
                    driveMotors(-(speed == 0 ? g_baseSpeed + 20: speed), -(speed == 0 ? g_baseSpeed + 20: speed));
                    delay(300);

                    driveMotors(-(speed == 0 ? g_baseSpeed + 10 : speed), (speed == 0 ? g_baseSpeed + 10 : speed));
                    delay(250);
                    ifFirstDirection = false;
                } 
                else {
                    // ? geri dönerkenn hafif sağa
                    driveMotors(-(speed == 0 ? g_baseSpeed + 10 : speed), (speed == 0 ? g_baseSpeed + 10 : speed));
                }
                break;

            default:
                break;
        }

        bool anyOnLine = getSensorBinary(analogRead(PIN_irMiddle)); // ? orta sensör çizgiyi görene kadar (ortalanana kadar)

        if (anyOnLine) {
            driveMotors(0, 0);
            controlBlind = true;
            goDirectionStartTime = 0;
            goDirectionElapsedTime = 0;
            return;
        }

        // ? Çok hızlı döngü olmasın diye küçük bir gecikme
        delay(10);
    }

    controlBlind = true; // ? üç saniye geçerde çizgiyi bulamazsa
    goDirectionStartTime = 0;
    goDirectionElapsedTime = 0;
    driveMotors(0, 0);
    g_integral = 0;
    g_previousError = 0;
    g_lastPIDTime = 0;

}


long readUltrasonicDistance() { // ? mesafe sensörünü okur (bloklayıcı non-blok yazmaya üşendiminyo)
    digitalWrite(PIN_trig, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_trig, LOW);
    long duration = pulseIn(PIN_echo, HIGH);
    long distanceCm = duration * 0.034 / 2;
    return distanceCm;
}


void calibrateSensors() { // ? Ir sensörleri arka planla kalibre eder ivmeölçeri falan da aynı şekilde
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
    backgroundDetectionValue = (sensorL+sensorM+sensorR)/3; // ? üçünün ortalaması
    burnLed(green);
    delay(100);
    burnLed(black);
    delay(100);
    burnLed(green);
    delay(100);
    burnLed(black);
    delay(100);

    // * çizgi kalibrasyonu 
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

            // ? önce arka planın koyu mu açık mı olduğu tespit ediliyor sonrasında okunan değerin arka plana göre çizgi olup olmadığını algılayıp kaydediyor

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
    localLineDetectionValue = (sensorL+sensorM+sensorR)/3; // ? üçünün ortalaması
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
    if (localLineDetectionValue > backgroundDetectionValue) isReversedLines = false;
    else isReversedLines = true;
    digitalWrite(PIN_motorEN, 1);
    if (debug) {
        Serial.println("\narka plan : ");
        Serial.print(backgroundDetectionValue);
        Serial.println("\nçizgi : ");
        Serial.print(localLineDetectionValue);
        Serial.println("\nthreshold : ");
        Serial.print(lineDetectionValue);
        Serial.println("\nreversed : ");
        Serial.print(isReversedLines);
    }

}



void setPIDParameters(float kp, float ki, float kd, int baseSpeed) {
    g_Kp = kp;
    g_Ki = ki;
    g_Kd = kd;
    g_baseSpeed = baseSpeed;
}

void blindDetection() { // ? robot yönünü kaybettiğinde çalışır
    unsigned long elapsed = millis() - g_blindStartTime;
    
    if (elapsed < 200) {
        driveMotors(g_baseSpeed + 30, -(g_baseSpeed + 30));
    }
    else if (elapsed < 1600) {
        driveMotors(-(g_baseSpeed + 20), g_baseSpeed + 20);
    }
    else if (elapsed < 2000) {
        goTo(backward, 0);
    }
}

void controlStages() { // ? bölümleri kontrol eder
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
        goTo(backward, 0);
        break;
    case RETURN:
        followLine();
    default:
        break;
    }

}

// YENİ KAVŞAK FONKSİYONLARI
void reactionInterSections(String interSectionType, int interSectionPattern) {
    if (isStop) return;

    reactionTypes reaction = AUTO;

    for (int i = 0; i < 3; i++) { // ? patterne verilecek reactionu çeker
        if (interSectionType == lineReactions[i].state) {
            reaction = lineReactions[i].reaction;
            break;
        }
    }

    if (reaction == AUTO) { 

        if (interSectionPattern == -1) {  // ? sol
            goTo(left, g_baseSpeed);
        } 
        else if (interSectionPattern == 0) { //  ? T
            goTo(stop_it, 0);
        } 
        else if (interSectionPattern == 1) { // ? sağ
            goTo(right, g_baseSpeed);
        }
    } 
    else {

        if (reaction == STOP) {
            goTo(stop_it, 0);
        } 
        else if (reaction == LEFT) {
            goTo(left, g_baseSpeed);
        } 
        else if (reaction == MID) {
            goTo(forward, g_baseSpeed);
        } 
        else if (reaction == RIGHT) {
            goTo(right, g_baseSpeed);
        }
    }

    lastInterSectionTime = millis();
}


void controlInterSection() {
    unsigned long currentTime = millis();
    if (lastInterSectionTime != 0 && (currentTime - lastInterSectionTime) < (unsigned long)interSectionCoolDown * 1000) {
        return;
    }

    if (!isCarOnGround || !isCalibrated || isStop) return;

    bool sensorReadings[interSectionsnumSensorReadsFilter][3];

    for (int i = 0; i < interSectionsnumSensorReadsFilter; i++) {
        sensorReadings[i][0] = getSensorBinary(analogRead(PIN_irLeft));
        sensorReadings[i][1] = getSensorBinary(analogRead(PIN_irMiddle));
        sensorReadings[i][2] = getSensorBinary(analogRead(PIN_irRight));
        delay((int)(interSectionsFilterSleepTime * 1000));
    }

    bool finalSensors[3];
    for (int sensorIndex = 0; sensorIndex < 3; sensorIndex++) {
        int trueCount = 0;
        for (int k = 0; k < interSectionsnumSensorReadsFilter; k++) {
            if (sensorReadings[k][sensorIndex]) trueCount++;
        }
        finalSensors[sensorIndex] = (trueCount > interSectionsnumSensorReadsFilter / 2);
    }

    bool isSensorL  = finalSensors[0];
    bool isSensorM  = finalSensors[1];
    bool isSensorR  = finalSensors[2];

    int activeCount = isSensorL + isSensorM + isSensorR;

    if (activeCount < minActiveSensorToInterSections) return;

    bool isT = (isSensorL && isSensorM && isSensorR);
    bool isRight = (!isSensorL && isSensorM && isSensorR);
    bool isLeft  = (isSensorL && isSensorM && !isSensorR); 

    if (isT) {
        if(debug) Serial.println("TInterSection");
        reactionInterSections("TInterSection", 0);
    } else if (isRight) {
        if(debug) Serial.println("RightInterSection");
        reactionInterSections("RightInterSection", 1);
    } else if (isLeft) {
        if(debug) Serial.println("LeftInterSection");
        reactionInterSections("LeftInterSection", -1);
    }
}


void followLine() { // ? çizgi takibi
    if (!isCalibrated || !isCarOnGround || isStop || currentStage == DISCHARGE) {
        goTo(stop_it, 0);
        g_integral = 0;
        g_previousError = 0;
        g_lastPIDTime = 0;
        g_blindActive = false;
        return;
    }
    
    bool Sleft = getSensorBinary(analogRead(PIN_irLeft));
    bool Smiddle = getSensorBinary(analogRead(PIN_irMiddle));
    bool Sright = getSensorBinary(analogRead(PIN_irRight));

    if (controlBlind) {
        bool anyOnLine = Sleft || Smiddle || Sright;

        if (!anyOnLine) { // ? çizgi yoksa
            if (!g_blindActive) { 
                if (g_blindFilterStartTime == 0) {
                    g_blindFilterStartTime = millis();
                }
                if (millis() - g_blindFilterStartTime > 1000) { // ? 1 saniyedir kayıpsa blind detectionu başlat
                    g_integral = 0;
                    g_previousError = 0;
                    g_lastPIDTime = 0;
                    g_blindActive = true;
                    g_blindStartTime = millis();
                    if(debug) Serial.println("blind detectiona girildi");
                }
            }

            if (g_blindActive) {
                blindDetection();
                return;
            }
        } 
        else {
            if (g_blindActive) {
                if(debug) Serial.println("çizgi bulundu blind çıktı");
            }
            g_blindFilterStartTime = 0;
            g_blindActive = false;
        }
    }

    if (isStop) return;

    if (manuelInterSectionMode) {
        controlInterSection();
        if (isStop) return;
    }

    // ? -1 sol 1 sağ
    float totalWeight = Sleft + Smiddle + Sright;
    float error;

    if (totalWeight == 0) error = g_previousError;
    else error = (-1.0 * Sleft + 0 * Smiddle + 1.0 * Sright) / totalWeight;
    

    unsigned long currentTime = millis();

    if (g_lastPIDTime == 0) {
        g_lastPIDTime = currentTime;
        g_previousError = error;
        driveMotors(g_baseSpeed, g_baseSpeed);
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
    
    int correction = P+I+D;

    int  leftSpeed = g_baseSpeed - correction;
    int  rightSpeed = g_baseSpeed + correction;

    leftSpeed = constrain(leftSpeed, 0, speedMax);
    rightSpeed = constrain(rightSpeed, 0, speedMax);

    driveMotors(leftSpeed, rightSpeed);
}   


void setup() {
    Serial.begin(9600);      
    buttonMode = 1;
    PinInit();
    myServo.write(90);

}


void loop() {
    blinkLed(isCalibrated, red, 500); // ? kalibre edilmediyse kalır
    if (delaySenc(TIMER_ULTRASONIC, 70) && !isStop && isCarOnGround && isCalibrated && currentStage == DEPARTURE) { // ? Kule kontrolü 
        if (readUltrasonicDistance() < 5) {
                myServo.write(0);
                currentStage = DISCHARGE;
                isStop = true;
            }
    }
    controlCarGround(); // ? yerde mi kontrolü

    if (!isStop && isCarOnGround && currentStage != DISCHARGE) { // ? servo 1 düzeltme
            myServo.write(90);
    }
    
    if (!digitalRead(PIN_button) && delaySenc(TIMER_BUTTON, 70)) { // ? mod atama
        if (buttonMode < 2) buttonMode += 1;
        else buttonMode = 1;
    }

    // * Mod kontrolü
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