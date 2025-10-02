// Motor pinlerini siz kendi kitinize göre doldurun
#define ENA 5   // Sol motor hız pin (PWM)
#define IN1 6   // Sol motor ileri
#define IN2 7   // Sol motor geri

#define ENB 9   // Sağ motor hız pin (PWM)
#define IN3 10  // Sağ motor ileri
#define IN4 11  // Sağ motor geri

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();

  Serial.println("Motor kontrol hazir.");
  Serial.println("Komutlar:");
  Serial.println("f <hiz> -> ileri");
  Serial.println("b <hiz> -> geri");
  Serial.println("l <hiz> -> sola don");
  Serial.println("r <hiz> -> saga don");
  Serial.println("s -> dur");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    int speed = 150; // varsayilan hiz

    if (Serial.peek() == ' ') { // hiz parametresi varsa oku
      Serial.read(); // boslugu atla
      speed = Serial.parseInt();
    }

    switch (cmd) {
      case 'f': forward(speed); break;
      case 'b': backward(speed); break;
      case 'l': turnLeft(speed); break;
      case 'r': turnRight(speed); break;
      case 's': stopMotors(); break;
    }
  }
}

void forward(int spd) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

void backward(int spd) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

void turnLeft(int spd) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); // sol geri
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); // sag ileri
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

void turnRight(int spd) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); // sol ileri
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); // sag geri
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
