#define PWM_A D3
#define DIR_A D12
#define BRAKE_A D9
#define PWM_B D11
#define DIR_B D13
#define BRAKE_B D8
#define TRIGGER_F D1
#define ECHO_F D0
#define sen_1 D2
#define sen_2 D7
#define sen_3 D4
#define sen_4 D6
#define sen_5 D5

int pos = 0;
bool FS1, FS2, FS3, FS4, FS5;
long cm;
int error = 0;
int sum_error = 0;
int PID_value = 0;
float Kp = 1.0;
float Ki = 0.0;

int speedA = 180;
int speedB = 255;

void setup() {
  Serial.begin(9600);
  delay(500);
  pinMode(PWM_A, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(BRAKE_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(BRAKE_B, OUTPUT);
  pinMode(ECHO_F, INPUT);
  pinMode(TRIGGER_F, OUTPUT);
  pinMode(sen_1, INPUT);
  pinMode(sen_2, INPUT);
  pinMode(sen_3, INPUT);
  pinMode(sen_4, INPUT);
  pinMode(sen_5, INPUT);
}

void read_ir() {
  FS1 = digitalRead(sen_1);
  FS2 = digitalRead(sen_2);
  FS3 = digitalRead(sen_3);
  FS4 = digitalRead(sen_4);
  FS5 = digitalRead(sen_5);

  Serial.print(FS1);
  Serial.print("\t");
  Serial.print(FS2);
  Serial.print("\t");
  Serial.print(FS3);
  Serial.print("\t");
  Serial.print(FS4);
  Serial.print("\t");
  Serial.print(FS5);
  Serial.print("\n");
}

long read_ultra(int trigger, int echo) {
  long duration, distance;
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigger, LOW);
  duration = pulseIn(echo, HIGH);
  distance = (duration / 2) / 29.1;
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
}

void drive_motorA(int speedA) {
  if (speedA > 0) {
    digitalWrite(DIR_A, LOW);
    digitalWrite(BRAKE_A, LOW);
    analogWrite(PWM_A, speedA);
  } else if (speedA < 0) {
    digitalWrite(DIR_A, HIGH);
    digitalWrite(BRAKE_A, LOW);
    analogWrite(PWM_A, abs(speedA));
  } else {
    digitalWrite(BRAKE_A, HIGH);  // Brake if speedA is 0
  }
}

void drive_motorB(int speedB) {
  if (speedB > 0) {
    digitalWrite(DIR_B, LOW);
    digitalWrite(BRAKE_B, LOW);
    analogWrite(PWM_B, speedB);
  } else if (speedB < 0) {
    digitalWrite(DIR_B, HIGH);
    digitalWrite(BRAKE_B, LOW);
    analogWrite(PWM_B, abs(speedB));
  } else {
    digitalWrite(BRAKE_B, HIGH);  // Brake if speedB is 0
  }
}

void forward_cross() {
  int FS1 = digitalRead(sen_1);
  int FS2 = digitalRead(sen_2);
  int FS3 = digitalRead(sen_3);
  int FS4 = digitalRead(sen_4);
  int FS5 = digitalRead(sen_5);

  if (FS1 == LOW && FS2 == HIGH && FS3 == HIGH && FS4 == HIGH && FS5 == HIGH) {
    stop();
  } else if (FS1 == LOW && FS2 == LOW && FS3 == HIGH && FS4 == HIGH && FS5 == HIGH) {
    stop();
  } else if (FS1 == LOW && FS2 == LOW && FS3 == LOW && FS4 == HIGH && FS5 == HIGH) {
    stop();
  } else if (FS1 == HIGH && FS2 == LOW && FS3 == LOW && FS4 == LOW && FS5 == LOW) {
    // turn left
    drive_motorA(-50);
    drive_motorB(50);
  } else if (FS1 == LOW && FS2 == HIGH && FS3 == LOW && FS4 == LOW && FS5 == LOW) {
    // turn left
    drive_motorA(-50);
    drive_motorB(50);
  } else if (FS1 == LOW && FS2 == LOW && FS3 == HIGH && FS4 == LOW && FS5 == LOW) {
    // forward
    drive_motorA(50);
    drive_motorB(50);
  } else if (FS1 == LOW && FS2 == LOW && FS3 == LOW && FS4 == HIGH && FS5 == LOW) {
    // turn right
    drive_motorA(50);
    drive_motorB(-50);
  } else if (FS1 == LOW && FS2 == LOW && FS3 == LOW && FS4 == LOW && FS5 == HIGH) {
    // turn right
    drive_motorA(50);
    drive_motorB(-50);
  } else {
    stop();
  }
}

void forward(int speedA, int speedB) {
  drive_motorA(speedA);
  drive_motorB(speedB);
}

void stop() {
  drive_motorA(0);
  drive_motorB(0);
}

void turn_right() {
  while (digitalRead(sen_5) == HIGH) {
    drive_motorA(100);
    drive_motorB(-100);
  }
  stop();

  while (digitalRead(sen_3) == HIGH) {
    drive_motorA(100);
    drive_motorB(100);
  }
  stop();
}

void turn_left() {
  while (digitalRead(sen_1) == HIGH) {
    drive_motorA(-100);
    drive_motorB(100);
  }
  stop();

  while (digitalRead(sen_3) == HIGH) {
    drive_motorA(100);
    drive_motorB(100);
  }
  stop();
}

void move_outcross() {
  while (true) {
    int FS1 = digitalRead(sen_1);
    int FS2 = digitalRead(sen_2);
    int FS3 = digitalRead(sen_3);
    int FS4 = digitalRead(sen_4);
    int FS5 = digitalRead(sen_5);

    if (FS1 == LOW && FS2 == HIGH && FS3 == HIGH && FS4 == HIGH && FS5 == HIGH) {
      drive_motorA(speedA);
      drive_motorB(speedB);
    } else if (FS1 == LOW && FS2 == LOW && FS3 == HIGH && FS4 == HIGH && FS5 == HIGH) {
      drive_motorA(speedA);
      drive_motorB(speedB);
    } else if (FS1 == HIGH && FS2 == HIGH && FS3 == HIGH && FS4 == LOW && FS5 == LOW) {
      drive_motorA(speedA);
      drive_motorB(speedB);
    } else {
      stop();
      break;
    }
  }
}

void loop() {
  read_ir();
  cm = read_ultra(TRIGGER_F, ECHO_F);
  stop();
  turn_right();
  turn_left();
  forward_cross();
  move_outcross();
    int setpoint = 20; 
  error = setpoint - cm;
  int P_value = Kp * error;
  sum_error += error;
  int I_value = Ki * sum_error;
  PID_value = P_value + I_value;
  speedA = speedA + PID_value;
  speedB = speedB - PID_value;
  speedA = constrain(speedA, -255, 255);
  speedB = constrain(speedB, -255, 255);
  forward(speedA, speedB);
  delay(50); 
}
