import event, time, cyberpi, mbot2, mbuild, sys

# * pid parametreleri
baseSpeed = 15
Kp = 6.5
Ki = 0.5
Kd = 2
maxIntegral = 50
prevError = 0
integral = 0
######

mainLoopDelay = 0.01 # ? her döngüdekşi bekleme süresi koymazsak diğer işlemler için sorun çıkabilir


debug = False

# * çizgi kaybedilme ayarları
blindStartTime = 0 
isBlind = False
blindTimes = 0

# * bölümler 
class stages:
    DEPARTURE = 0 # ? gidiş
    DISCHARGE = 1 # ? boşaltım
    RETURN = 2 # ? dönüş
currentStage = stages.DEPARTURE
goDirectionStartTime = 0
goDirectionElaspedTime = 0
ifFirstDirection = True


# * pattern tepkileri
lineReactionTypes = [
    "AUTO",
    "LEFT",
    "MIDDLE",
    "RIGHT",
    "STOP"
]

lineReactions = {
    "rightInterSection": "AUTO",
    "leftInterSection": "AUTO",
    "TInterSection": "RIGHT"
}
interSectionCoolDown = 4 # ? kavşak sonrası bekleme süresi düşük verirsen pid düzelitrken sağ ve sol çizgiye değebileceği için 
lastInterSectionTime = 0
manuelIntersectionMode = True

# * genel
calibrateMode = 1 # ? kalibrasyona göre sensor modu, sensör ışıkları arka planda yanıyorsa 1 çizgide yanıyorsa 0 yap
isStop = True # ? robotun durmasını kotnrol eden flag
currentCalibrateTreshold = 50 # ? siyah çizgi için eşik değeri  


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
    if mode == 1:
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

def limit_power(power):
    """Motor gücünü -100 ile 100 arasında sınırlar"""
    return max(-100, min(100, power))

def calculatePID(error, dt=1):
    """PID hesaplaması"""
    global prevError, integral

    integral += error
    integral = max(-maxIntegral, min(maxIntegral, integral))

    derivative = (error - prevError) / dt
    pid_output = Kp * error + Ki * integral + Kd * derivative

    prevError = error
    return pid_output




def driveMotors(left, right):
    """Motorları sür ve güçleri limitle"""
    mbot2.drive_power(limit_power(left), limit_power(right))




def blindDetection():
    """Çizgi kaybolduğunda önce 180 derece döner bulamazsa hazır patterni uygular (yani kağıt üstünde öyle)"""
    global blindStartTime, currentStage

    currentTime = time.time()
    elapsed = (currentTime - blindStartTime) * 1000  # ms cinsinden

    # 1. aşama: 180 derece dönüs
    if elapsed < 2000:
        goDirection()
    elif elapsed < 5000:
        driveMotors(20, 40)  # hafif sağa
    elif elapsed < 8000:
        driveMotors(40, 20)  # hafif sola
    else:
        driveMotors(0, 0)
        Cprint("Çizgi bulunamadı, robot durdu ve resetlendi.")
        stop_robot()
        return
    time.sleep(0.05)


def reactionInterSections(ınterSectionPattern: str, pattern: int):
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



def controlInterSection():
    """Kavşak kontrolü"""
    global lastInterSectionTime, isStop
    
    if lastInterSectionTime != 0 and (time.time() - lastInterSectionTime) < interSectionCoolDown: # ? kavşakdan dönerken sağ veya sol sensorler yanıp sönebilir onun için bekleme süresi
        return
    
    if isStop:
        return
    
    numSensorReads = 5 # ? sensörlerin kaç kere okunacağı. yükseltilirse robotun tepkileri yavaşlar
    sensorReadings = []
    sleepTime = 0.005 # ? dikkatli ayarla uzun tutarsan numSensorReads*sleepTime dan çok fazla geçikme yaşanabilir. Kısa tutarsan zaten ortalama almanın anlamı kalmaz
    
    for _ in range(numSensorReads): # ? okumak için döngü 
        reading = getSensorValues(mode=calibrateMode)
        sensorReadings.append(reading)
        time.sleep(sleepTime)
    
    finalSensors = []
    for sensorIndex in range(4): # ? her döngüde bir sensorun ortalaması alınır
        values = [sensorReadings[i][sensorIndex] for i in range(numSensorReads)] # ? okunan sensorleri o anki indexe göre listeden çeker ve yeni bir liste oluştırur
        trueCount = sum(values) # ? değerlerin toplamını alıyor getSensorVlaues sensore göre 1 veya 0 döndürüyor. bu  yüzden toplamak mümkün 
        finalSensors.append(trueCount > numSensorReads / 2) # ? eğer o anki sensorun toplam değeri okunanın yarısından büyükse true eklenir yani sensor çizgi üstündedir değilse false eklenir yani arka plandadır
    
    active_count = sum(finalSensors) 
    
    if active_count < 3: # ? aktif sensör hesabı 3 den küçükse çalışmaz (iki ortanın ve (sağ veya sol) un aktifliğini bekler kısaca)
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



            

def goDirection(direction = "backward"):
    """İstenilen yöne göner"""
    global goDirectionElaspedTime, ifFirstDirection, goDirectionStartTime
    goDirectionStartTime = time.time()
    ifFirstDirection = True
    goDirectionElaspedTime = 0
    while goDirectionElaspedTime < 3:
        if isStop:
            driveMotors(0, 0)
            return False
        currentTime = time.time()
        goDirectionElaspedTime = currentTime - goDirectionStartTime 
        if direction == "middle" or direction == "mıddle":
            mbot2.straight(5)
            return

        elif direction == "stop":
            stop_robot()
            return

        elif direction == "right":
            if ifFirstDirection:
                mbot2.straight(5)
                mbot2.turn_right(50)
                time.sleep(0.5)
                ifFirstDirection = False
            else:
                mbot2.turn_right(40)
        elif direction == "left":
            if ifFirstDirection:
                mbot2.straight(5)
                mbot2.turn_left(50)
                time.sleep(0.5)
                ifFirstDirection = False
            else:
                mbot2.turn_left(40)
        elif direction == "backward":
            if ifFirstDirection and currentStage == stages.DISCHARGE:
                mbot2.straight(-10)
                mbot2.turn_right(50)
                time.sleep(0.5)
                ifFirstDirection = False
            mbot2.turn_right(40)
        sensors = getSensorValues(True, mode=calibrateMode)
        isOnLine = any(sensors)
        if not isOnLine: # ? çizgiyi bulamadıysa dönmeye devam
            continue
        driveMotors(0, 0)
        goDirectionStartTime = 0
        goDirectionElaspedTime = 0 
        break


def line_follow():
    """çizgi takibi"""
    global integral, prevError, blindStartTime, isBlind, blindTimes, lastInterSectionTime
    if isStop:
        return
    sensors = getSensorValues(mode=calibrateMode)
    isOnLine = any(sensors)
    if not isOnLine:
        if blindTimes > 3:
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
            lastInterSectionTime = 0.1;
            isBlind = False

    if isStop:  
        return
    if manuelIntersectionMode:
        controlInterSection()
        if isStop:
            return
    error = mbuild.quad_rgb_sensor.get_offset_track(1)/100
    pid = calculatePID(error)
    left_power = baseSpeed - pid
    right_power = -1 * (baseSpeed + pid)
    driveMotors(left_power, right_power)

    if debug:
        Cprint("E:", error, "I:", integral, "D:", prevError)



def controlStages():
    """bölüm kontrolü"""
    global currentStage
    if currentStage == stages.DEPARTURE:
        line_follow()
    elif currentStage == stages.DISCHARGE:
        driveMotors(0, 0)
        goDirection()
        currentStage = stages.RETURN
    elif currentStage == stages.RETURN:
        line_follow()

def main():
    global currentStage
    try:
        while True:
            if isStop:
                time.sleep(0.04)
                continue
            if currentStage == stages.DEPARTURE:
                sonicSensorCM = mbuild.ultrasonic2.get()
                if sonicSensorCM < 8:
                    currentStage = stages.DISCHARGE
            controlStages()
            
            time.sleep(mainLoopDelay)
    except Exception as e:
        Cprint("Ana döngüde hata : \n")
        sys.print_exception(e)
        stop_robot()
    

def calibrateThreshold(threshold = 50):
    try:
        mbuild.quad_rgb_sensor.set_custom_color(
            r=255,
            g=255,
            b=255,
            tolerance=threshold
        )
    except Exception as e:
        sys.print_exception(e)

def stop_robot():
    global prevError, integral, ifFirstDirection, lastInterSectionTime, isStop, blindTimes, isBlind, blindStartTime, goDirectionStartTime
    isStop = True
    cyberpi.stop_other()
    Cprint("Robot durdu")
    prevError = 0
    integral = 0
    blindTimes = 0
    isBlind = False
    blindStartTime = 0
    goDirectionStartTime = 4
    lastInterSectionTime = 0
    mbot2.EM_stop("ALL")
    ifFirstDirection = True

def debug_robot():
    mbuild.quad_rgb_sensor.set_led(color = "white")

    while True:
        sensors = [mbuild.quad_rgb_sensor.get_color_sta(sensor) for sensor in range(4, 0, -1)]
        Cprint(sensors)
        time.sleep(0.2)


def start_robot():
    global prevError, integral, isStop, currentStage, lastInterSectionTime, blindTimes, isBlind, blindStartTime, goDirectionStartTime, ifFirstDirection
    
    cyberpi.stop_other()
    
    prevError = 0
    integral = 0
    isStop = False
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
    global Kd, kp
    Kp = 6.5
    Kd = 2
    Cprint("Değerler sıfırlandı")

@event.is_press('up')
def upEvent():
    global Kp
    Kp += 0.5
    Cprint("kp : ", Kp)

@event.is_press('down')
def downEvent():
    global Kp
    Kp -= 0.5
    Cprint("kp : ", Kp)

@event.is_press('right')
def rightEvent():
    global Kd
    Kd += 0.2
    Cprint("kd : ", Kd)

@event.is_press('left')
def rightEvent():
    global Kd
    Kd -= 0.2
    Cprint("kd : ", Kd)
