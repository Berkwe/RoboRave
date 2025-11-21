#include <Arduino.h>
#include <FastLED.h>
#include <Servo.h>


Servo myServo;


// * pin tanımlamaları
#define PIN_trig 13 // ? trig mesafe sensoru
#define PIN_echo 12 // ? echo mesafe sensoru
#define PIN_servo 10 // ? servo pini
// * ir sensörler ters aslında fakat bunu bilmeden kodu buna göre optimize ettim şimdi de diğer türlü düzeltmeye üşeniyorum, controlinterseciton fonksyionunda bu yüzden tersten okunuyor
#define PIN_irLeft A0 // ? sol ir sensörü
#define PIN_irMidLeft A1 // ? orta sol ir sensörü
#define PIN_irMiddle A2 // ? orta ir sensörü
#define PIN_irMidRight A4 // ? orta sağ ir sensörü
#define PIN_irRight A5 // ?  sağ ir sensörü

#define PIN_motorRightSpeed 5 // ? sağ motor PWM
#define PIN_motorLeftSpeed 6 // ? sol motor PWM
#define PIN_motorRight 7 // ? Sağ motor yön
#define PIN_motorLeft 8 // ? Sol motor yön
#define PIN_motorEN 3 // ? Motor sürücüsü enable pini

#define PIN_button 2 // ? d2 Button pini

#define PIN_led 4 // ? d4 RGB led pini


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
CRGB pink = CRGB(217, 25, 255); // ? daha atnamdı
// * diğerleri
CRGB black = CRGB(0,0,0); // ? led kapatma rengi


// * kalibrasyon
struct sensorCalibration {
    int lineValue;
    int backgroundValue;
    int threshold;
};

sensorCalibration cal_Left;
sensorCalibration cal_MidLeft;
sensorCalibration cal_Middle;
sensorCalibration cal_MidRight;
sensorCalibration cal_Right;
bool isReversedLines = false; // ? çizginin arka plandan daha açık olduğunu belirten flag (Bizim robotta kullanılan pull-up direncinden kaynaklı açık renkler daha düşük değerlerde okunuyor)
bool isCalibrated = false; // ? kalibre edildi mi


// * sensör okumaları için
struct SensorReadings {
    int analog[5];
    bool binary[5];
};

// * kavşak karar mekanizması değişkenler (değiştirme)
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



// * düzenlenebilecek değişkenler
// * genel
#define speedMax 250 // ? en fazla hız
#define minGroundDistance 780 // ? robotun havada varsayılması için gereken en düşük değer
bool debug = false;

// * PID değişkenleri
float g_Kp = 200; // ? pcb sayesinde ne verirsen gidiyo sıfır göcünme
float g_Ki = 0; // ? arttırınc bi faydasını göremedim robot genelde sağa kayıyor fakat düzletmekte çok osilasyona sebep oldu
float g_Kd = 0; // ? 145 kp ve 130 hızında ideal olanlar 10 ve 10'un katları. 21 verdim farketmiyor sanırsam 10'un katı olduğu sürece
int g_baseSpeed = 75; // ? 200 hızında bile çalışıyor pcby sayesinde

// * ultrasonik değişkenleri
unsigned long minMillisTimeForUltrasonic = 4; // ? saniye cinsinden başlangıç zamanından itibaren ultrasoniğin taramaya başlama zamanı 3-5 yapsan yeter hıza göre değişir

// * kavşak ayarları
lineReactionsStruct lineReactions[] = {
    {"RightInterSection", AUTO}, // ? sağda kavşak
    {"LeftInterSection", AUTO}, // ? solda kavşak
    {"TInterSection", RIGHT} // ? T kavşak
};
bool manuelInterSectionMode = true; // ? kavşakları ellemi belirleyeceğimiz yoksa robotu kendi haline mi bırakacağımız etkileyen flag
int interSectionCoolDown = 3; // ? bir kavşak algılandıkdan sonraki kavşağın algılanmsı için geçmesi gereken minimum süre sn cinsinden
int minActiveSensorToInterSections = 2; // ? bir dönüşün kavşak sayılması için en az kaç tane sensorun aktive olması gerektiğini belirler. bırak kalsın bu ayarlarda.
int interSectionsnumSensorReadsFilter = 3; // ? sensorlerin kavşak algılamada kaç kere okunacağı yükseltilirse PID yavaşlayabilir ve hız arttığında kavşak algılamayabilir, düşürülürse kalibrasyonla çözülmüş nadir olan sorun ortaya çıkabilir. 
float interSectionsFilterSleepTime = 0.003; // ? dikkatli ayarla uzun tutarsan numSensorReads*sleepTime dan çok fazla geçikme yaşanabilir. Kısa tutarsan zaten ortalama almanın anlamı kalmaz

// * kalibrasyon 
int backgroundReadTimes = 5;
int lineReadTimes = 30;


// * PID r değişkenleri
float g_integral = 0;
float g_previousError = 0;
unsigned long g_lastPIDTime = 0;


// * Blind detection değişkenleri
unsigned long g_blindStartTime = 0;
unsigned long g_blindFilterStartTime = 0;
bool g_blindActive = false;


// * ultrasonik sensor değişkenleri
unsigned long g_triggerTime = 0;
bool g_waitingEcho = false;
long g_lastDistance = 100;


// * Otomatik timer yönetimi değişkenleri
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
bool controlBlind = true;


// * modlar
enum mods {
    lineFollowing = 1,
    calibrating = 2
};

int buttonMode = 1; // ? mod değeri


// * stage değişkenleri 
enum stages { // ? Yarışma 3 bölümden oluşuyor
    DEPARTURE,
    DISCHARGE,
    RETURN
};

stages currentStage = DEPARTURE;


// * genel değişkenler
bool isCarOnGround = false; // ? robot yerde mi
bool isStop = false; // ? ismi yeterince açıklayıcı



// * stublar
void followLine();
SensorReadings getSensorValues();


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
    SensorReadings sensorReads = getSensorValues();

    int sensorL = sensorReads.analog[0];
    int sensorML = sensorReads.analog[1];
    int sensorM = sensorReads.analog[2];
    int sensorMR = sensorReads.analog[3];
    int sensorR = sensorReads.analog[4];
    if (sensorR > minGroundDistance && sensorM > minGroundDistance && sensorL > minGroundDistance && sensorML > minGroundDistance && sensorMR > minGroundDistance) {
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
    // * button initi
    pinMode(PIN_button, INPUT_PULLUP);
    // * servo initi
    myServo.attach(10);
    // * mesafe sensörü initi
    pinMode(PIN_trig, OUTPUT);
    pinMode(PIN_echo, INPUT);
}


void driveMotors(int speedLeft, int speedRight, bool backward = false) {
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
        Serial.println("goto ilk satir return");
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

        SensorReadings sensorReads = getSensorValues();
        bool anyOnLine = sensorReads.binary[2]; // ? orta sensör çizgiyi görene kadar (ortalanana kadar)

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


void calibrateSensors() { // ? Ir sensörleri arka planla kalibre eder
    if (debug) Serial.println("Kalibrasyon başladı");
    digitalWrite(PIN_motorEN, 0);
    burnLed(white);
    delay(2000);

    // * değişken atamaları
    // * Genel okuma değişkenleri
    unsigned long sensorL = 0;
    unsigned long sensorML = 0;
    unsigned long sensorM = 0;
    unsigned long sensorMR = 0;
    unsigned long sensorR = 0;
    // * sensöre göre çizgi değeri değişkenleri
    int sensorLV = 0;
    int sensorMLV = 0;
    int sensorMV = 0;
    int sensorMRV = 0;
    int sensorRV = 0;
    // * ortalama değerler değişkenleri
    int backgroundDetectionValueAvarge;
    int localLineDetectionValueAvarge;

    // * arka plan kalibrasyonu
    delay(300);
    for (int i = 0; i < backgroundReadTimes; i++)
        {
            SensorReadings sensorReads = getSensorValues();

            sensorL += sensorReads.analog[0];
            sensorML += sensorReads.analog[1];
            sensorM += sensorReads.analog[2];
            sensorMR += sensorReads.analog[3];
            sensorR += sensorReads.analog[4];
        }
    // * structa yazdırma
    cal_Left.backgroundValue = (sensorL / backgroundReadTimes); // ? sensör okumalarının ortalaması alınıp yazılıyor
    cal_MidLeft.backgroundValue = (sensorML / backgroundReadTimes);
    cal_Middle.backgroundValue = (sensorM / backgroundReadTimes);
    cal_MidRight.backgroundValue = (sensorMR / backgroundReadTimes);
    cal_Right.backgroundValue = (sensorR / backgroundReadTimes);
    backgroundDetectionValueAvarge = (sensorL + sensorML + sensorM + sensorMR + sensorR) / 5; // ? beşinin ortalaması
    // * ledli geri bildirim
    burnLed(green);
    delay(100);
    burnLed(black);
    delay(100);
    burnLed(green);
    delay(100);
    burnLed(black);
    delay(100);

    // * çizgi kalibrasyonu

    // * değerler sıfırlanıyor
    sensorL = 0;
    sensorML = 0;
    sensorM = 0;
    sensorMR = 0;
    sensorR = 0;

    // * ledli geri bildirm
    burnLed(blue);
    delay(2000);
    burnLed(green);

    for (int i = 0; i < lineReadTimes; i++)
        {
            SensorReadings sensorReads = getSensorValues();

            int L_sensorL = sensorReads.analog[0];
            int L_sensorML = sensorReads.analog[1];
            int L_sensorM = sensorReads.analog[2];
            int L_sensorMR = sensorReads.analog[3];
            int L_sensorR = sensorReads.analog[4];


            // ? önce arka planın koyu mu açık mı olduğu tespit ediliyor sonrasında okunan değerin arka plana göre çizgi olup olmadığını algılayıp kaydediyor
            // ? düşük değer koyu yüksek değer açık renk (siyah beyaz işte)
            if (backgroundDetectionValueAvarge <= 512) {  // ? eğer arka plan koyuysa
                if (L_sensorL > backgroundDetectionValueAvarge) {
                    sensorL += L_sensorL;
                    sensorLV++;
                }
                if (L_sensorML > backgroundDetectionValueAvarge) {
                    sensorML += L_sensorML;
                    sensorMLV++;
                }
                if (L_sensorM > backgroundDetectionValueAvarge) {
                    sensorM += L_sensorM;
                    sensorMV++;
                }
                if (L_sensorMR > backgroundDetectionValueAvarge) {
                    sensorMR += L_sensorMR;
                    sensorMRV++;
                }
                if (L_sensorR > backgroundDetectionValueAvarge) {
                    sensorR += L_sensorR;
                    sensorRV++;
                }
            }
            else {
                if (L_sensorL < backgroundDetectionValueAvarge) {
                    sensorL += L_sensorL;
                    sensorLV++;
                }
                if (L_sensorML < backgroundDetectionValueAvarge) {
                    sensorML += L_sensorML;
                    sensorMLV++;
                }
                if (L_sensorM < backgroundDetectionValueAvarge) {
                    sensorM += L_sensorM;
                    sensorMV++;
                }
                if (L_sensorMR < backgroundDetectionValueAvarge) {
                    sensorMR += L_sensorMR;
                    sensorMRV++;
                }
                if (L_sensorR < backgroundDetectionValueAvarge) {
                    sensorR += L_sensorR;
                    sensorRV++;
                }
            }
            
        delay(30); // * küçük bi delay okuma birkaç saniye sürsün diye (readTimese bağlı mesela read times = 30, 30*30=900ms = 1 saniye~)
        }
    // ? ortalaması alınıyor fakat kaç kere değer okunduysa ona göre
    
    sensorL /= sensorLV != 0 ? sensorLV : 1;
    sensorML /= sensorMLV != 0 ? sensorMLV : 1;
    sensorM /= sensorMV != 0 ? sensorMV : 1;
    sensorMR /= sensorMRV != 0 ? sensorMRV : 1;
    sensorR /= sensorRV != 0 ? sensorRV : 1;
    // * structa değer ataması
    cal_Left.lineValue = sensorL;
    cal_MidLeft.lineValue = sensorML;
    cal_Middle.lineValue = sensorM;
    cal_MidRight.lineValue = sensorMR;
    cal_Right.lineValue = sensorR;
    localLineDetectionValueAvarge = (sensorL + sensorML + sensorM + sensorMR + sensorR) / 5; // ? beşinin ortalaması
    // * ledli geri bildirim
    burnLed(green);
    delay(100);
    burnLed(black);
    delay(100);
    burnLed(green);
    delay(100);
    burnLed(black);
    delay(100);

    isCalibrated = true;
    // * thresholdar structa atanıyor
    cal_Left.threshold = (cal_Left.backgroundValue+cal_Left.lineValue)/2;
    cal_MidLeft.threshold = (cal_MidLeft.backgroundValue+cal_MidLeft.lineValue)/2;
    cal_Middle.threshold = (cal_Middle.backgroundValue+cal_Middle.lineValue)/2;
    cal_MidRight.threshold = (cal_MidRight.backgroundValue+cal_MidRight.lineValue)/2;
    cal_Right.threshold = (cal_Right.backgroundValue+cal_Right.lineValue)/2;
    // ! ###
    if (localLineDetectionValueAvarge>backgroundDetectionValueAvarge) isReversedLines = false;
    else isReversedLines = true; // ? eğer çizgi açık bir renk ise
    
    digitalWrite(PIN_motorEN, 1);
    if (debug) { // !!!! BUNU DEBUG YAPMAYI UNUTMA
        Serial.print("Left => line: "); Serial.print(cal_Left.lineValue);
        Serial.print(" bg: "); Serial.print(cal_Left.backgroundValue);
        Serial.print(" th: "); Serial.println(cal_Left.threshold);

        Serial.print("MidLeft => line: "); Serial.print(cal_MidLeft.lineValue);
        Serial.print(" bg: "); Serial.print(cal_MidLeft.backgroundValue);
        Serial.print(" th: "); Serial.println(cal_MidLeft.threshold);

        Serial.print("Middle => line: "); Serial.print(cal_Middle.lineValue);
        Serial.print(" bg: "); Serial.print(cal_Middle.backgroundValue);
        Serial.print(" th: "); Serial.println(cal_Middle.threshold);

        Serial.print("MidRight => line: "); Serial.print(cal_MidRight.lineValue);
        Serial.print(" bg: "); Serial.print(cal_MidRight.backgroundValue);
        Serial.print(" th: "); Serial.println(cal_MidRight.threshold);

        Serial.print("Right => line: "); Serial.print(cal_Right.lineValue);
        Serial.print(" bg: "); Serial.print(cal_Right.backgroundValue);
        Serial.print(" th: "); Serial.println(cal_Right.threshold);


        Serial.println("Reversed Lines : ");
        Serial.print(isReversedLines);
    }
}


void setPIDParameters(float kp, float ki, float kd, int baseSpeed) {
    g_Kp = kp;
    g_Ki = ki;
    g_Kd = kd;
    g_baseSpeed = baseSpeed;
}


void blindDetection() {
    unsigned long elapsed = millis() - g_blindStartTime;

    if (elapsed < 200) {
        driveMotors(g_baseSpeed + 30, -(g_baseSpeed + 30));
    }
    else if (elapsed < 1600) {
        driveMotors(-(g_baseSpeed + 20), g_baseSpeed + 20);
    }
    else if (elapsed < 2000) {
        goTo(backward);
    }
}

void dischargeAction() { // ? boşaltım görevi
    isStop = true;
    goTo(stop_it, 0);
    burnLed(yellow);
    delay(2000);
    burnLed(black);
    currentStage = RETURN;
    isStop = false;
    goTo(backward);
}


void controlStages() { // ? bölümleri kontrol eder
    switch (currentStage)
    {
    case DEPARTURE:
        followLine();
        break;
    case DISCHARGE:
        dischargeAction();
        break;
    case RETURN:
        followLine();
        break;
    default:
        break;
    }
}



SensorReadings getSensorValues() {
    SensorReadings s;
    int temp[5] = {
        analogRead(PIN_irLeft),
        analogRead(PIN_irMidLeft),
        analogRead(PIN_irMiddle),
        analogRead(PIN_irMidRight),
        analogRead(PIN_irRight)
    };

    for ( int i = 0; i < 5; i++) s.analog[i] = temp[i];

    for ( int i = 0; i < 5; i++) {
        switch(i) {
            case 0: s.binary[i] = !isReversedLines ? (temp[i] < cal_Left.threshold) : (temp[i] >= cal_Left.threshold); break;
            case 1: s.binary[i] = !isReversedLines ? (temp[i] < cal_MidLeft.threshold) : (temp[i] >= cal_MidLeft.threshold); break;
            case 2: s.binary[i] = !isReversedLines ? (temp[i] < cal_Middle.threshold) : (temp[i] >= cal_Middle.threshold); break;
            case 3: s.binary[i] = !isReversedLines ? (temp[i] < cal_MidRight.threshold) : (temp[i] >= cal_MidRight.threshold); break;
            case 4: s.binary[i] = !isReversedLines ? (temp[i] < cal_Right.threshold) : (temp[i] >= cal_Right.threshold); break;
        }
    }
    return s;
}



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
    if (lastInterSectionTime != 0 && (currentTime - lastInterSectionTime) < (unsigned long)interSectionCoolDown) { // ? küçük bi filtreleme
        return;
    }

    if (!isCarOnGround || !isCalibrated || isStop) return;

    bool sensorReadings[interSectionsnumSensorReadsFilter][5];

    for (int i = 0; i < interSectionsnumSensorReadsFilter; i++) { // ? ortalama için sensör okuma genelde gerekmiyor ama olsa iyi olur
        SensorReadings s = getSensorValues();
        for (int j = 0; j < 5; j++) {
            sensorReadings[i][j] = s.binary[j];
        }
        delay((int)(interSectionsFilterSleepTime * 1000));
    }

    bool finalSensors[5];
    for (int sensorIndex = 0; sensorIndex < 5; sensorIndex++) {
        int trueCount = 0;
        for (int k = 0; k < interSectionsnumSensorReadsFilter; k++) {
            if (sensorReadings[k][sensorIndex]) trueCount++;
        }
        finalSensors[sensorIndex] = (trueCount > interSectionsnumSensorReadsFilter / 2);
    }

    bool isSensorL  = finalSensors[4];
    bool isSensorML = finalSensors[3];
    bool isSensorM  = finalSensors[2];
    bool isSensorMR = finalSensors[1];
    bool isSensorR  = finalSensors[0];

    int activeCount = isSensorL + isSensorML + isSensorM + isSensorMR + isSensorR; // ? aktif sensör sayısı

    if (activeCount < minActiveSensorToInterSections) return; // ? küçük bir filtreleme daha

    bool isT = (activeCount >= 4);

    bool isRight = (isSensorM && (isSensorML || isSensorMR) && isSensorR && !isSensorL);

    bool isLeft  = (isSensorM && (isSensorML || isSensorMR) && isSensorL && !isSensorR); 


    if (isT) {
        Serial.println("TInterSection");
        reactionInterSections("TInterSection", 0);
    } else if (isRight) {
        Serial.println("RightInterSection");
        reactionInterSections("RightInterSection", 1);
    } else if (isLeft) {
        Serial.println("LeftInterSection");
        reactionInterSections("LeftInterSection", -1);
    }
}




void followLine() { // ? çizgi takibi
    if (!isCalibrated || !isCarOnGround || isStop || currentStage == DISCHARGE) { // ? yine filtreleme
        goTo(stop_it, 0);
        g_integral = 0;
        g_previousError = 0;
        g_lastPIDTime = 0;
        g_blindActive = false;
        return;
    }

    if (controlBlind) {
        SensorReadings s = getSensorValues();
        bool anyOnLine = s.binary[0] || s.binary[1] || s.binary[2] || s.binary[3] || s.binary[4];

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
                    Serial.println("blind detectiona girildi");
                }
            }

            if (g_blindActive) {
                blindDetection();
                return;
            }
        } 
        else {
            if (g_blindActive) {
                Serial.println("çizgi bulundu blind çıktı");
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

    SensorReadings sr = getSensorValues();
    bool Sleft     = sr.binary[0];
    bool SmidLeft  = sr.binary[1];
    bool Smiddle   = sr.binary[2];
    bool SmidRight = sr.binary[3];
    bool Sright    = sr.binary[4];

    float totalWeight = Sleft + SmidLeft + Smiddle + SmidRight + Sright;
    float error;

    if (totalWeight == 0) error = g_previousError;
    else error = (-1.0 * Sleft + -0.5 * SmidLeft + 0.0 * Smiddle + 0.5 * SmidRight + 1.0 * Sright) / totalWeight;

    unsigned long now = millis();

    if (g_lastPIDTime == 0) {
        g_lastPIDTime = now;
        g_previousError = error;
        driveMotors(g_baseSpeed, g_baseSpeed);
        Serial.println("ilk frame düz");
        return;
    }

    float dt = (now - g_lastPIDTime) / 1000.0;
    if (dt <= 0 || dt > 1.0) dt = 0.02;
    g_lastPIDTime = now;

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

    int leftSpeed  = g_baseSpeed - correction;
    int rightSpeed = g_baseSpeed + correction;

    leftSpeed  = constrain(leftSpeed,  0, speedMax);
    rightSpeed = constrain(rightSpeed, 0, speedMax);

    driveMotors(leftSpeed, rightSpeed);
}


void setup() {
    Serial.begin(115250);      
    buttonMode = 1;
    PinInit();
    myServo.write(90);

    cal_Left.lineValue = 537; // !! dikat et bunları tanımlamassan getsensorvalueden kaynaklı calibratesensors fonksyionunda hata alırsın
    cal_Left.backgroundValue = 37;
    cal_Left.threshold = 287;


    cal_MidLeft.lineValue = 321;
    cal_MidLeft.backgroundValue = 33;
    cal_MidLeft.threshold = 177;


    cal_Middle.lineValue = 585;
    cal_Middle.backgroundValue = 37;
    cal_Middle.threshold = 311;


    cal_MidRight.lineValue = 726;
    cal_MidRight.backgroundValue = 42;
    cal_MidRight.threshold = 384;

    cal_Right.lineValue = 565;
    cal_Right.backgroundValue = 38;
    cal_Right.threshold = 301;
    
    isCalibrated = true;
    isReversedLines = true;

}

bool isStart = false;
void loop() {
    if (isStart) {
    blinkLed(isCalibrated, red, 500); // ? kalibre edilmediyse kalır
    if (millis() > (minMillisTimeForUltrasonic*1000) && delaySenc(TIMER_ULTRASONIC, 70) && !isStop && isCarOnGround && isCalibrated && currentStage == DEPARTURE) { // ? Kule kontrolü 
        if (readUltrasonicDistance() < 9) {
                myServo.write(0);
                currentStage = DISCHARGE;
                isStop = true;
            }
    }
    controlCarGround(); // ? yerde mi kontrolü


    if (!isStop && isCarOnGround && currentStage != DISCHARGE) { // ? servo 1 düzeltme
            myServo.write(90);
    }
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
            delay(300);
            isStart = !isStart; 
            //calibrateSensors();
            buttonMode = 1;
            break;
    }

}