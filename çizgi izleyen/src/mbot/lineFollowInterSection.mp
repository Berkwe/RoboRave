import event, time, cyberpi, mbot2, mbuild

# * pid parametreleri
baseSpeed = 15
Kp = 18
Ki = 0.8
Kd = 6
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

# * genel
calibrateMode = 1 # ? kalibrasyona göre sensor modu, sensör ışıkları arka planda yanıyorsa 1 çizgide yanıyorsa 0 yap
calibrateTime = 1
isStop = True
          


def Cprint(*message): 
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
    """Motor gücünü -100 ile 100 arasında sınırla"""
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
    """Çizgi kaybolduğunda önce 180° dön, bulunamazsa farklı bir arama patterni uygula."""
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
    global lastInterSectionTime, isStop
    
    if lastInterSectionTime != 0 and (time.time() - lastInterSectionTime) < interSectionCoolDown:
        return
    
    if isStop:
        return
    
    SAMPLE_COUNT = 5
    sensorReadings = []
    
    for _ in range(SAMPLE_COUNT):
        reading = getSensorValues(mode=calibrateMode)
        sensorReadings.append(reading)
        time.sleep(0.005)
    
    finalSensors = []
    for sensorIndex in range(4):
        values = [sensorReadings[i][sensorIndex] for i in range(SAMPLE_COUNT)]
        trueCount = sum(values)
        finalSensors.append(trueCount > SAMPLE_COUNT / 2)
    
    active_count = sum(finalSensors)
    
    if active_count < 3:
        return
    
    isT = active_count == 4
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
            mbot2.straight(5)
            mbot2.turn_right(50)
            if ifFirstDirection:
                time.sleep(0.5)
                ifFirstDirection = False
        elif direction == "left":
            mbot2.straight(5)
            mbot2.turn_left(50)
            if ifFirstDirection:
                time.sleep(0.5)
                ifFirstDirection = False

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



def sensor_test_mode():
    """Sensör test modu"""
    Cprint("Sensör Test Modu")
    while True:
        sensors = getSensorValues(mode=calibrateMode)
        Cprint(
            "R1:", sensors[0],
            "R2:", sensors[1],
            "R3:", sensors[2],
            "R4:", sensors[3]
        )
        time.sleep(0.2)

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
        Cprint("Ana döngüde hata : " + str(e))
        stop_robot()
    
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

@event.is_press('up')
def start_sensor_test():
    global calibrateTime
    cyberpi.stop_other()
    if calibrateTime != 8:
        calibrateTime += 1
    else:
        calibrateTime = 1
    mbuild.quad_rgb_sensor.adjust(calibrateTime)
