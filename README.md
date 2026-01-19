# Bluetooth-Controlled-Car-with-Obstacle-Avoidance-

#include <Servo.h>
// Pin Definitions
#define BUZZER_PIN 12
#define MOTOR1_PIN1 2
#define MOTOR1_PIN2 3
#define MOTOR2_PIN1 4
#define MOTOR2_PIN2 5
#define ENA 6
#define ENB 11
#define SERVO_PIN 9
#define IR_SENSOR_PIN 7  // IR Sensor digital output

#define MOTOR_SPEED 100

Servo scanServo;

// Variables
int msec_cnt = 0;
int ms_counter = 0;
char B_CNT = 0;
int L_CNT = 0;
int SEED_SPEED = 0;

bool SEED_MOT_ON_FLG = 0;
boolean OBJECT_DECT_FLG = 0;
boolean BUZZ_FLG = 0, BLUET_BUZZ_FLG = 0, UART_FLG = 0;
boolean LCD_CLR_FLG = 0;
boolean START_FLG = 0, STOP_FLAG = 0;
boolean S_flag = 0, L_flag = 0, F_flag = 0, R_flag = 0;
boolean FARWARD_FLAG = 0;
boolean BACKWARD_FLG = 0, RIGHT_TURN_FLAG = 0, LEFT_TURN_FLAG = 0, STOP_FLG = 0, ONCE_FLG = 0;
bool HUMAN_FLAG = 0, OBJ_DECT_FLAG = 0;
bool LEFT_OBJECT_FLG = 0, RIGHT_OBJECT_FLG = 0;
char RX_DATA;
bool UART_FLG1 = 0;
int CNT = 0;
bool OBJ_LEFT_TURN_FLAG = 0, OBJ_RIGHT_TURN_FLAG = 0, OBJ_BACKWARD_FLG = 0;

void MOTOR_CONTROL(void);
void BT_RecieveData(void);
void timerIsr(void);

void setup() {
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  scanServo.attach(SERVO_PIN);
  scanServo.write(90); // Center position

  Serial.begin(9600);
  START_FLG = 0;
}

bool isObjectDetected() {
  return digitalRead(IR_SENSOR_PIN) == LOW; // LOW means object detected
}

void checkAndAvoidObstacle() {
  scanServo.write(90); delay(500); // Face front
  STOP_FLG = 1;
  OBJECT_DECT_FLG = 1;
  MOTOR_CONTROL();
  delay(800);

  bool frontObject = isObjectDetected();

  scanServo.write(150); delay(700); // Look right
  bool rightObject = isObjectDetected();

  scanServo.write(30); delay(700);  // Look left
  bool leftObject = isObjectDetected();

  scanServo.write(90); delay(500);  // Reset to front

  Serial.print("Front: "); Serial.println(frontObject ? "YES" : "NO");
  Serial.print("Right: "); Serial.println(rightObject ? "YES" : "NO");
  Serial.print("Left: "); Serial.println(leftObject ? "YES" : "NO");

  if (!leftObject && rightObject) {
    Serial.println("Decision: TURN LEFT");
    OBJ_LEFT_TURN_FLAG = 1;
    digitalWrite(MOTOR1_PIN1, HIGH);
    digitalWrite(MOTOR1_PIN2, LOW);
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, HIGH);
    analogWrite(ENA, MOTOR_SPEED);
    analogWrite(ENB, MOTOR_SPEED);
    delay(2000);
    OBJ_LEFT_TURN_FLAG = 0;
  } else if (!rightObject) {
    Serial.println("Decision: TURN RIGHT");
    OBJ_RIGHT_TURN_FLAG = 1;
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, HIGH);
    digitalWrite(MOTOR2_PIN1, HIGH);
    digitalWrite(MOTOR2_PIN2, LOW);
    analogWrite(ENA, MOTOR_SPEED);
    analogWrite(ENB, MOTOR_SPEED);
    delay(2000);
    OBJ_RIGHT_TURN_FLAG = 0;
  } else {
    Serial.println("Decision: MOVE BACKWARD");
    OBJ_BACKWARD_FLG = 1;
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, HIGH);
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, HIGH);
    analogWrite(ENA, MOTOR_SPEED);
    analogWrite(ENB, MOTOR_SPEED);
    delay(2000);
    OBJ_BACKWARD_FLG = 0;
  }

  OBJECT_DECT_FLG = 0;
  STOP_FLG = 0;
}

void loop() {
  BT_RecieveData();
  timerIsr();
  delay(2);

  if (UART_FLG) {
    if (STOP_FLG) {
      Serial.println("ROBO IS STOPPED");
    } else if (FARWARD_FLAG) {
      Serial.println("ROBO IS MOVING FORWARD");
    } else if (RIGHT_TURN_FLAG) {
      Serial.println("ROBO IS TURNING RIGHT");
    } else if (LEFT_TURN_FLAG) {
      Serial.println("ROBO IS TURNING LEFT");
    } else if (BACKWARD_FLG) {
      Serial.println("ROBO IS MOVING BACKWARD");
    } else {
      Serial.println("ROBO IS STOPPED");
    }
    UART_FLG = 0;
  }

  if (FARWARD_FLAG || BACKWARD_FLG || LEFT_TURN_FLAG || RIGHT_TURN_FLAG) {
    if (isObjectDetected()) {
      CNT++;
      if (CNT >= 2) {
        OBJECT_DECT_FLG = 1;
        CNT = 0;
      }
    } else {
      CNT = 0;
    }
  }

  if (OBJECT_DECT_FLG) {
    checkAndAvoidObstacle();
  } else {
    MOTOR_CONTROL();
  }
}

void timerIsr(void) {
  msec_cnt++;
  if (msec_cnt >= 150) {
    if (OBJECT_DECT_FLG) {
      digitalWrite(BUZZER_PIN, digitalRead(BUZZER_PIN) ^ 1);
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }
    msec_cnt = 0;
  }

  L_CNT++;
  if (L_CNT > 200) {
    UART_FLG = 1;
    UART_FLG1 = 1;
    LCD_CLR_FLG = 1;
    L_CNT = 0;
  }

  if(BUZZ_FLG) {
    B_CNT++;
    if(B_CNT > 50) {
        B_CNT = 0;
        BUZZ_FLG = 0;
     }
  }

  ms_counter++;
  if (ms_counter >= 100) {
    ms_counter = 0;
  }
}

void MOTOR_CONTROL(void) {
  if (STOP_FLG) {
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, LOW);
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
  } else if (FARWARD_FLAG) {
    digitalWrite(MOTOR1_PIN1, HIGH);
    digitalWrite(MOTOR1_PIN2, LOW);
    digitalWrite(MOTOR2_PIN1, HIGH);
    digitalWrite(MOTOR2_PIN2, LOW);
    analogWrite(ENA, MOTOR_SPEED);
    analogWrite(ENB, MOTOR_SPEED);
  } else if (LEFT_TURN_FLAG) {
    digitalWrite(MOTOR1_PIN1, HIGH);
    digitalWrite(MOTOR1_PIN2, LOW);
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, HIGH);
    analogWrite(ENA, MOTOR_SPEED);
    analogWrite(ENB, MOTOR_SPEED);
  } else if (RIGHT_TURN_FLAG) {
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, HIGH);
    digitalWrite(MOTOR2_PIN1, HIGH);
    digitalWrite(MOTOR2_PIN2, LOW);
    analogWrite(ENA, MOTOR_SPEED);
    analogWrite(ENB, MOTOR_SPEED);
  } else if (BACKWARD_FLG) {
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, HIGH);
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, HIGH);
    analogWrite(ENA, MOTOR_SPEED);
    analogWrite(ENB, MOTOR_SPEED);
  }
}

void BT_RecieveData(void) {
  if (Serial.available() > 0) {
    RX_DATA = Serial.read();
    if (RX_DATA == 'F') {
      FARWARD_FLAG = 1; BACKWARD_FLG = 0; RIGHT_TURN_FLAG = 0; LEFT_TURN_FLAG = 0; STOP_FLG = 0;
    } else if (RX_DATA == 'B') {
      FARWARD_FLAG = 0; BACKWARD_FLG = 1; RIGHT_TURN_FLAG = 0; LEFT_TURN_FLAG = 0; STOP_FLG = 0;
    } else if (RX_DATA == 'L') {
      FARWARD_FLAG = 0; BACKWARD_FLG = 0; RIGHT_TURN_FLAG = 0; LEFT_TURN_FLAG = 1; STOP_FLG = 0;
    } else if (RX_DATA == 'R') {
      FARWARD_FLAG = 0; BACKWARD_FLG = 0; RIGHT_TURN_FLAG = 1; LEFT_TURN_FLAG = 0; STOP_FLG = 0;
    } else if (RX_DATA == 'S') {
      FARWARD_FLAG = 0; BACKWARD_FLG = 0; RIGHT_TURN_FLAG = 0; LEFT_TURN_FLAG = 0; STOP_FLG = 1; SEED_SPEED = 0;
    }
  }
}
