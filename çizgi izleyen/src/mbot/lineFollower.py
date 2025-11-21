"""
MBOT2 micropython
author : berkwe
version : 1.1

"""
import event, time, cyberpi, mbot2, mbuild, sys


# * Değiştirilebilen parametreler

# * Discharge ayarları
motorPort = "S2" # ? harici dc motor portu
dischargeMoveSpeed = 100 # ? discharge actiondaki motorun titreşme hızı (dahili motor)
dischargeActionTime_THREEHUNDBALL = 15 # ? kaç kere ileri geri yapacğaını söyler (300 topluk görevde)
dischargeActionTime_ONEBALL = 1 # ? aynısının tek topluk hali


# * PID parametreleri
baseSpeed = 25 # ! 15 stabil net, 25 ideal,  30-35 yüksek,  40 ağırlıksız imkansız
Kp = 27.5 # ! baseSpeed = 15 iken 6-8 arası sonrasında base speede doğru orantılı olarak arttırırlır
# ! üstünde kutu varsa robot dengesizleşiyor kp yi arttırınca çöülüyor
Ki = 0 # ! genelde aynı tutulmalı eğer çizgide düzenli olarak çok kayıyorsa azalt veya arttır 
Kd = 4.0 # ! baseSpeed = 15 iken 1.8-2.4 arası ideal baseSpeede doğru orantılı aynı şekilde. kavşağı dönüşleri alamıyorsa arttırmayı dene kpden bağımsız titriyorsa azalt


# * çizgi kaybetme ayarları
controlBlind = True # ? blindDetetcionun aktif olup olmadığını tutan flag
blindDetectionFilterTime = 28 # ? filtreleme için en düşük blind times değeri


# * kavşak algılama ayarları
lineReactionTypes = [ # ? kavşak tepki türleri
    "AUTO",
    "LEFT",
    "MIDDLE",
    "RIGHT",
    "STOP"
]

lineReactions = { # ?  kavşaklara verilecek tepkiler
    "rightInterSection": "AUTO",
    "leftInterSection": "AUTO",
    "TInterSection": "RIGHT"
}

interSectionCoolDown = 4 # ? kavşak sonrası bekleme süresi düşük verirsen pid düzelitrken sağ ve sol çizgiye değebileceği için
manuelIntersectionMode = True # ? kapalı yapmak robotun belirli kavşaklara elle girilen tepkilerle değil kendi tepkileriyle gitmesini sağlar
interSectionsnumSensorReadsFilter = 2 # ? sensorlerin kavşak algılamada kaç kere okunacağı yükseltilirse PID yavaşlayabilir ve hız arttığında kavşak algılamayabilir, düşürülürse kalibrasyonla çözülmüş nadir olan sorun ortaya çıkabilir. 
interSectionsFilterSleepTime = 0.005 # ? dikkatli ayarla uzun tutarsan numSensorReads*sleepTime dan çok fazla geçikme yaşanabilir. Kısa tutarsan zaten ortalama almanın anlamı kalmaz
minActiveSensorToInterSections = 2 if baseSpeed >= 19 else 3 # ? bir dönüşün kavşak sayılması için en az kaç tane sensorun aktive olması gerektiğini belirler. bırak kalsın bu ayarlarda.


# * genel değişkenler
mainLoopDelay = 0.01 # ? her döngüdeki bekleme süresi, koymazsak diğer işlemler için sorun çıkabilir
debug = False # ? hata ayıklama flagi normalde false yap pid hesaplamasını geçiktirir
calibrateMode = 1 # ? kalibrasyona göre sensor modu, sensör ışıkları arka planda yanıyorsa 1 çizgide yanıyorsa 0 yap
sonicSensorThresholdCM = 6.5

# ! değiştirilemeyen ayarlar

# * PID çok elleşme
maxIntegral = 50 
prevError = 0
integral = 0
isBlind = False # ? kayıp olup olmadığını tutan flag
blindTimes = 0 # ? kaç kere çizgiyi kaybettiği (filtreleme için)


# * kavşak
lastInterSectionTime = 0 # ? son kavşağın ne zamna olduğunu millis biçiminde tutar


# * bölümler 
class stages:
    DEPARTURE = 0 # ? gidiş
    DISCHARGE = 1 # ? boşaltım
    RETURN = 2 # ? dönüş
currentStage = stages.DEPARTURE # ? o anki bölümü tutan 
goDirectionStartTime = 0 # 
goDirectionElaspedTime = 0
ifFirstDirection = True

class stageModes:
    ONEBALL = 0 # ? tek top modu
    THREEHUNDBALL = 1 # ? 300 top modu
currentStageMode = stageModes.THREEHUNDBALL


# * blind ayarları
blindStartTime = 0  # ? ilk kaybedilme süresi


# * genel
isStop = True # ? robotun durmasını kotnrol eden flag
shakeList = []
Kmode = 2 # ? 0 kp 1 kd 2 şuanlık blind threshold


cyberpi.mbot2.write_digital(0, motorPort) # ? dc motoru durdur


def Cprint(*message): 
    """Konsole bişiler yazdırmanın kolay yolu"""
    message = list(message)
    message = map(str, message)
    strMessage = " ".join(message)
    cyberpi.console.println(strMessage)


def getSensorValues(isOnlyMid = False, mode = 1):
    """quad rgbden gelen değerleri soldan sağa doğru listeler
        isonlymid true verilirse sadece orta sensorleri döndürür
        modu kalibrasyona göre ayarla sensör ışıkları arka planda yanıyorsa 1 çizgide yanıyorsa 0 yap
        """
    if mode == 1: # ? Sensor 4 en sol 1 en sağdaki RGB
        if isOnlyMid:
            return [
            not mbuild.quad_rgb_sensor.is_background(3, 1),
            not mbuild.quad_rgb_sensor.is_background(2, 1),
            ]
        return [
            not mbuild.quad_rgb_sensor.is_background(4, 1),
            not mbuild.quad_rgb_sensor.is_background(3, 1),
            not mbuild.quad_rgb_sensor.is_background(2, 1),
            not mbuild.quad_rgb_sensor.is_background(1, 1)
        ]
    else:
        if isOnlyMid:
            return [
            mbuild.quad_rgb_sensor.is_background(3, 1),
            mbuild.quad_rgb_sensor.is_background(2, 1),
            ]
        return [
            mbuild.quad_rgb_sensor.is_background(4, 1),
            mbuild.quad_rgb_sensor.is_background(3, 1),
            mbuild.quad_rgb_sensor.is_background(2, 1),
            mbuild.quad_rgb_sensor.is_background(1, 1)
        ]


def limit_power(power): # ? ismi anlatmıyormu yeterince ?  
    """Motor gücünü -100 ile 100 arasında sınırlar"""
    return max(-100, min(100, power))


def calculatePID(error, dt=1): # * Buna da yorum satırı eklemeyeyimhatırla 
    """PID hesaplaması"""
    global prevError, integral

    integral += error
    integral = max(-maxIntegral, min(maxIntegral, integral))

    derivative = (error - prevError) / dt
    pid_output = Kp * error + Ki * integral + Kd * derivative

    prevError = error
    return pid_output


def driveMotors(left, right): # ? bunun da ismi açıklar
    """Motorları sür ve güçleri limitle"""
    mbot2.drive_power(limit_power(left), limit_power(right))


def blindDetection(): # ? çizgi kaybolduğunda yapılacak standartlar
    """Çizgi kaybolduğunda önce 180 derece döner bulamazsa hazır patterni uygular (yani kağıt üstünde öyle)"""
    global blindStartTime, currentStage

    currentTime = time.time()
    elapsed = (currentTime - blindStartTime) * 1000  # ? ms cinsinden

    # ? 1. aşama: 180 derece dönüs
    if elapsed < 2000:
        goDirection("blind")
    elif elapsed < 5000:
        driveMotors(20, 40)  # ? hafif sağa
    elif elapsed < 8000:
        driveMotors(40, 20)  # ? hafif sola
    else:
        driveMotors(0, 0)
        Cprint("Çizgi bulunamadı, robot durdu ve resetlendi.")
        stop_robot()
        return
    time.sleep(0.05)


def reactionInterSections(ınterSectionPattern: str, pattern: int): # ? kavşaklarda verilecek tepkiler
    """Kavşaklara göre verilecek tepkiler"""
    global lastInterSectionTime
    if isStop:
        return
    reaction = lineReactions.get(ınterSectionPattern, "AUTO") 
    if reaction not in lineReactionTypes:
        Cprint("Geçersiz Tepki Girildi Düzeltin : ", reaction)
    Cprint(pattern)
    if reaction == "AUTO":
        if pattern == -1:
            goDirection("left")
        elif pattern == 0:
            goDirection("stop")
        elif pattern == 1:
            goDirection("right")
    else:
        goDirection(reaction.lower())
    lastInterSectionTime = time.time()


def controlInterSection(): # ? kavşak algılama
    """Kavşak kontrolü"""
    global lastInterSectionTime, isStop
    
    if lastInterSectionTime != 0 and (time.time() - lastInterSectionTime) < interSectionCoolDown: # ? kavşakdan dönerken sağ veya sol sensorler yanıp sönebilir onun için bekleme süresi
        return
    
    if isStop:
        return

    sensorReadings = []
    
    for _ in range(interSectionsnumSensorReadsFilter): # ? okumak için döngü 
        reading = getSensorValues(mode=calibrateMode)
        sensorReadings.append(reading)
        time.sleep(interSectionsFilterSleepTime)
    
    finalSensors = []
    for sensorIndex in range(4): # ? her döngüde bir sensorun ortalaması alınır
        values = [sensorReadings[i][sensorIndex] for i in range(interSectionsnumSensorReadsFilter)] # ? okunan sensorleri o anki indexe göre listeden çeker ve yeni bir liste oluştırur
        trueCount = sum(values) # ? değerlerin toplamını alıyor getSensorVlaues sensore göre 1 veya 0 döndürüyor. bu  yüzden toplamak mümkün 
        finalSensors.append(trueCount > interSectionsnumSensorReadsFilter / 2) # ? eğer o anki sensorun toplam değeri okunanın yarısından büyükse true eklenir yani sensor çizgi üstündedir değilse false eklenir yani arka plandadır
    
    active_count = sum(finalSensors) 
    
    if active_count < minActiveSensorToInterSections: # ? aktif sensör hesabı minActiveden küçükse çalışmaz.
        return
    
    isT = active_count == 4 # ? T kavşak kontrolü gerisini de anla artık
    isRight = (finalSensors[1] or finalSensors[2]) and finalSensors[3] and not finalSensors[0]
    isLeft = (finalSensors[1] or finalSensors[2]) and finalSensors[0] and not finalSensors[3]
    
    if debug:
        Cprint("Sensors:", finalSensors, "Active:", active_count)
        Cprint("T:", isT, "R:", isRight, "L:", isLeft)
    
    if isT:
        reactionInterSections("TInterSection", 0)
    elif isRight:
        reactionInterSections("rightInterSection", 1)
    elif isLeft:
        reactionInterSections("leftInterSection", -1)


def goDirection(direction): # ? bir yöne gider
    """İstenilen yöne göner"""
    global goDirectionElaspedTime, ifFirstDirection, goDirectionStartTime, controlBlind

    goDirectionStartTime = time.time()
    ifFirstDirection = True
    goDirectionElaspedTime = 0

    while goDirectionElaspedTime < 8: # ? 3 saniyede dönerken çizgi bulamadıysa blind detectiona geçer
        if isStop:
            driveMotors(0, 0)
            return False
        
        currentTime = time.time()
        goDirectionElaspedTime = currentTime - goDirectionStartTime
        controlBlind = False

        if direction == "middle" or direction == "mıddle":
            mbot2.straight(5) # ? 5 cm ileri 
            return

        elif direction == "stop":
            stop_robot()
            return

        elif direction == "right":
            if ifFirstDirection:
                mbot2.straight(5)
                mbot2.turn_right(30)
                time.sleep(0.5)
                ifFirstDirection = False

            else:
                mbot2.turn_right(25)

        elif direction == "left":
            if ifFirstDirection:
                mbot2.straight(5)
                mbot2.turn_left(30)
                time.sleep(0.5)
                ifFirstDirection = False

            else:
                mbot2.turn_left(25)

        elif direction == "blind":
            mbot2.turn_right(40)

        elif direction == "backward":
            if ifFirstDirection and currentStage == stages.DISCHARGE:
                mbot2.straight(-10)
                mbot2.turn_left(30)
                time.sleep(0.5)
                ifFirstDirection = False
            else:
                mbot2.turn_left(25)
        sensors = getSensorValues(True, mode=calibrateMode)
        isOnLine = any(sensors)
        if not isOnLine: # ? çizgiyi bulamadıysa dönmeye devam
            continue
        
        driveMotors(0, 0)
        controlBlind = True
        goDirectionStartTime = 0
        goDirectionElaspedTime = 0 
        break

    if goDirectionElaspedTime > 8:
        controlBlind = True


def line_follow(): # ? çizgiyi takip eder
    """çizgi takibi"""
    global integral, prevError, blindStartTime, isBlind, blindTimes, lastInterSectionTime
    if isStop:
        return
    
    if controlBlind:
        sensors = getSensorValues(mode=calibrateMode)
        isOnLine = any(sensors) # ? herhangi sensör çizgi üstünde mi?
        if not isOnLine:
            if blindTimes > blindDetectionFilterTime:
                if not isBlind:
                    integral = 0
                    prevError = 0
                    blindStartTime = time.time()
                    isBlind = True

                blindDetection()
                return
            blindTimes += 1
        else:
            blindTimes = 0
            if isBlind:
                lastInterSectionTime = 0.1
                isBlind = False 

    if isStop:  
        return
    
    if manuelIntersectionMode:
        controlInterSection()

        if isStop:
            return
        
    error = mbuild.quad_rgb_sensor.get_offset_track(1)/100 # ? mbotun API'sinden hata oranını çeker
    pid = calculatePID(error) 
    left_power = baseSpeed - pid
    right_power = -1 * (baseSpeed + pid)
    driveMotors(left_power, right_power)
    shakeList.append(cyberpi.get_shakeval()) # ? sallanma testi için listeye ekler

    if debug:
        Cprint("E:", error, "I:", integral, "D:", prevError)


def DischargeAction(): # ? boşaltım fonksyionu
    """Boşaltım görevi için fonksiyon"""
    global currentStage, currentStageMode, dischargeActionTime_THREEHUNDBALL, dischargeActionTime_ONEBALL
    driveMotors(0, 0)
    cyberpi.mbot2.write_digital(1, motorPort)
    if currentStageMode == stageModes.ONEBALL: # ? tek top
        if dischargeActionTime_ONEBALL % 2 == 1 and dischargeActionTime_ONEBALL != 1:
            dischargeActionTime_ONEBALL += 1
        for _ in range(dischargeActionTime_ONEBALL):
            mbot2.straight(-4, dischargeMoveSpeed)
            mbot2.straight(6, dischargeMoveSpeed)
        goDirection("backward")
    elif currentStageMode == stageModes.THREEHUNDBALL: # ? 300 top
        for _ in range(dischargeActionTime_THREEHUNDBALL):
            mbot2.straight(-4, dischargeMoveSpeed)
            mbot2.straight(6, dischargeMoveSpeed)
        goDirection("backward")
    cyberpi.mbot2.write_digital(0, motorPort)
    currentStage = stages.RETURN


def controlStages(): # ? bölümleri kontrol eder
    """bölüm kontrolü"""
    global currentStage

    if currentStage == stages.DEPARTURE:
        line_follow()

    elif currentStage == stages.DISCHARGE:
        DischargeAction()

    elif currentStage == stages.RETURN:
        line_follow()


def main(): # ? ana fonksyion
    global currentStage
    try:
        while True:

            if isStop:
                time.sleep(0.04)
                continue

            if currentStage == stages.DEPARTURE:
                sonicSensorCM = mbuild.ultrasonic2.get()
                if sonicSensorCM < sonicSensorThresholdCM:
                    Cprint("Sensor okudu")
                    currentStage = stages.DISCHARGE
            
            controlStages()
            time.sleep(mainLoopDelay)
    except Exception as e:
        Cprint("Ana döngüde hata : \n")
        sys.print_exception(e)
        stop_robot()


def stop_robot(): # ? robotu durdurur
    global prevError, integral, ifFirstDirection, lastInterSectionTime, isStop, blindTimes, isBlind, blindStartTime, goDirectionStartTime
    isStop = True
    cyberpi.stop_other()
    Cprint("Robot durdu")
    cyberpi.mbot2.write_digital(0, motorPort)
    cyberpi
    if len(shakeList) > 0:
        shakeAvargeVal = sum(shakeList)/len(shakeList)
        cyberpi.console.clear()
        Cprint("shakeVal : ", shakeAvargeVal)
    else:
        Cprint("Shake verisi toplanmadi.")
    shakeList.clear()
    prevError = 0
    integral = 0
    blindTimes = 0
    isBlind = False
    blindStartTime = 0
    goDirectionStartTime = 4
    lastInterSectionTime = 0
    mbot2.EM_stop("ALL")
    ifFirstDirection = True


def start_robot(): # ? robotu başlatır
    global prevError, integral, isStop, currentStage, lastInterSectionTime, blindTimes, isBlind, blindStartTime, goDirectionStartTime, ifFirstDirection, currentStageMode
    cyberpi.stop_other()
    prevError = 0
    integral = 0
    isStop = False
    cyberpi.mbot2.write_digital(0, motorPort)
    currentStage = stages.DEPARTURE
    lastInterSectionTime = 0
    blindTimes = 0
    isBlind = False
    blindStartTime = 0
    goDirectionStartTime = 0
    ifFirstDirection = True
    
    Cprint("Robot basladi")
    main()


@event.is_press('a')
def aEvent():
    stop_robot()


@event.is_press('b')
def bEvent():
    start_robot()


@event.is_press('middle')
def mEvent():
    global Kmode
    if Kmode >= 2:
        Kmode = 0

    else:
        Kmode += 1

    Cprint("mod : ", ("KP modu" if Kmode == 0 else "KD modu"))


@event.is_press('up')
def upEvent():
    global Kp, Kd, blindDetectionFilterTime
    if Kmode == 0:
        Kp += 0.5
        Cprint("Kp : ", Kp)

    elif Kmode == 1:
        Kd += 0.2
        Cprint("Kd : ", Kd)

    else:
        blindDetectionFilterTime += 1
        Cprint("BlindFilter : ", blindDetectionFilterTime)


@event.is_press('down')
def downEvent():
    global Kp, Kd, blindDetectionFilterTime
    if Kmode == 0:
        Kp -= 0.5
        Cprint("Kp : ", Kp)

    elif Kmode == 1:
        Kd -= 0.2
        Cprint("Kd : ", Kd)

    else:
        blindDetectionFilterTime -= 1
        Cprint("BlindFilter : ", blindDetectionFilterTime)


@event.is_press('right')
def rightEvent():
    global currentStageMode
    if currentStageMode == stageModes.ONEBALL:
        currentStageMode = stageModes.THREEHUNDBALL
    else:
        currentStageMode = stageModes.ONEBALL
    Cprint("mod : ", "tek top" if currentStageMode == stageModes.ONEBALL else "çok top")


@event.is_press('left')
def leftEvent():
    pass