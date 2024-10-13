// PID 제어

#include <Wire.h>
#include <ESP32Servo.h>       // ESP32용 서보 라이브러리 추가
#include "BluetoothSerial.h"  // ESP32의 블루투스 시리얼 라이브러리 추가

const int MPU_ADDR = 0x68;                  // I2C 통신을 위한 MPU6050의 주소
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;  // 가속도와 자이로
double angleAcX, angleAcY, angleAcZ;
double angleGyX, angleGyY, angleGyZ;
double angleFiX, angleFiY, angleFiZ;

const double RADIAN_TO_DEGREE = 180 / 3.14159;
const double DEG_PER_SEC = 32767 / 250;
const double ALPHA = 1 / (1 + 0.04);

unsigned long now = 0;
unsigned long past = 0;
double dt = 0;

// PID 제어 변수
double integralX = 0;
double integralY = 0;
double lastErrorX = 0;
double lastErrorY = 0;

double averAcX, averAcY, averAcZ;
double averGyX, averGyY, averGyZ;

// 자이로 센서 초기 캘리브레이션을 위한 오프셋
double offsetGyX = 0, offsetGyY = 0, offsetGyZ = 0;

// MG995 서보 모터 4개 사용
Servo mg995_1;
Servo mg995_2;
Servo mg995_3;
Servo mg995_4;

// ESP32에서 사용할 핀 번호 설정
int servoPin1 = 14;  // 서보 모터 1
int servoPin2 = 12;  // 서보 모터 2
int servoPin3 = 27;  // 서보 모터 3
int servoPin4 = 26;  // 서보 모터 4

// ESP32의 블루투스 시리얼 객체 생성
BluetoothSerial bluetooth;

// 이동 평균 필터용 배열 및 크기
#define FILTER_SIZE 10
double gyroXData[FILTER_SIZE];
double gyroYData[FILTER_SIZE];
double gyroZData[FILTER_SIZE];
int filterIndex = 0;

// 제어 모드: 0 = 자동 모드, 1 = 수동 모드
int controlMode = 0;
int previousControlMode = -1;  // 이전 모드를 추적하는 변수

// 서보 모터의 현재 각도를 추적하는 변수
int currentAngleX1 = 90;  // 서보 모터 1
int currentAngleX2 = 90;  // 서보 모터 2
int currentAngleY1 = 90;  // 서보 모터 3
int currentAngleY2 = 90;  // 서보 모터 4

// 수동 모드에서 서보 모터 목표 각도를 저장하는 변수
int targetAngleX = 90;
int targetAngleY = 90;

// 보정용 오프셋 각도
double offsetAngleX = 0;
double offsetAngleY = 0;

// PID 상수 (비례, 적분, 미분)
double Kp = 1.0;   // 비례 상수 (기존 2.0에서 1.0으로 감소)
double Ki = 0.005; // 적분 상수 (기존 0.01에서 0.005로 감소)
double Kd = 0.5;   // 미분 상수 (기존 1.0에서 0.5로 감소)

// 이동 평균 필터 함수
double movingAverage(double data[], double newData) {
  data[filterIndex] = newData;
  double sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += data[i];
  }
  return sum / FILTER_SIZE;
}

void setup() {
  // MPU6050 초기화
  initSensor();
  Serial.begin(9600);
  bluetooth.begin("ESP32_Gyroscope");

  // 자이로 캘리브레이션
  calibrateGyro();

  past = millis();

  // 서보 모터 핀에 연결
  mg995_1.attach(servoPin1);
  mg995_2.attach(servoPin2);
  mg995_3.attach(servoPin3);
  mg995_4.attach(servoPin4);

  mg995_1.write(90);
  mg995_2.write(90);
  mg995_3.write(90);
  mg995_4.write(90);

  delay(1000);  // 초기화 후 서보 모터가 위치를 잡을 시간 제공
}

void loop() {
  // 블루투스로 들어온 명령 확인 (모드 전환 및 수동 제어 모드의 각도 수신)
  if (bluetooth.available()) {
    String command = bluetooth.readStringUntil('c');  // 블루투스에서 들어온 값을 읽음

    // 모드 전환 명령
    if (command == "AUTO") {
      controlMode = 0;  // 자동 모드로 전환
      offsetAngleX = angleFiX;
      offsetAngleY = angleFiY;          // Y값은 그대로 유지
    } else if (command == "MANUAL") {
      controlMode = 1;  // 수동 모드로 전환
    }

    // 수동 모드에서 들어온 각도 값 (예: "X:90 Y:80" 형식으로 받는다고 가정)
    if (controlMode == 1) {
      if (command.startsWith("X:")) {
        int xIndex = command.indexOf("X:") + 2;
        targetAngleX = command.substring(xIndex).toInt();
        Serial.print("수동 모드에서 받은 목표 각도 - X: ");
        Serial.println(targetAngleX);
      } else if (command.startsWith("Y:")) {
        int yIndex = command.indexOf("Y:");
        targetAngleY = command.substring(yIndex + 2).toInt();
        Serial.print("수동 모드에서 받은 목표 각도 - Y: ");
        Serial.println(targetAngleY);
      }
    }
  }

  // 이전 모드와 현재 모드를 비교하여 출력 (모드가 변경될 때만)
  if (previousControlMode != controlMode) {
    if (controlMode == 0) {
      Serial.println("현재 모드: 자동 제어 모드");
    } else if (controlMode == 1) {
      Serial.println("현재 모드: 수동 제어 모드");
    }
    previousControlMode = controlMode;  // 이전 모드를 현재 모드로 업데이트
  }

  // 자동 제어 모드
  if (controlMode == 0) {
    // 자이로 센서 데이터 수집 및 dt 계산
    getData();
    getDT();

    // 가속도 기반 각도 계산
    angleAcX = atan(AcY / sqrt(pow(AcX, 2) + pow(AcZ, 2))) * RADIAN_TO_DEGREE;
    angleAcY = atan(-AcX / sqrt(pow(AcY, 2) + pow(AcZ, 2))) * RADIAN_TO_DEGREE;

    // 자이로 기반 각도 계산 (캘리브레이션 오프셋 적용 및 필터링)
    angleGyX = movingAverage(gyroXData, ((GyX - offsetGyX) / DEG_PER_SEC) * dt);
    angleGyY = movingAverage(gyroYData, ((GyY - offsetGyY) / DEG_PER_SEC) * dt);
    angleGyZ = movingAverage(gyroZData, ((GyZ - offsetGyZ) / DEG_PER_SEC) * dt);

    // 상보 필터 적용
    double angleTmpX = angleFiX + angleGyX * dt;
    double angleTmpY = angleFiY + angleGyY * dt;
    angleFiX = ALPHA * angleTmpX + (1.0 - ALPHA) * angleAcX;
    angleFiY = ALPHA * angleTmpY + (1.0 - ALPHA) * angleAcY;
    angleFiZ = angleGyZ;

    // 각도 오차 계산 (오프셋을 적용)
    double errorX = 0 - (angleFiX - offsetAngleX);
    double errorY = 0 - (angleFiY - offsetAngleY);

    // PID 적분 계산
    integralX += errorX * dt;
    integralY += errorY * dt;

    // PID 미분 계산
    double derivativeX = (errorX - lastErrorX) / dt;
    double derivativeY = (errorY - lastErrorY) / dt;

    // PID 제어 계산
    double correctionX = Kp * errorX + Ki * integralX + Kd * derivativeX;
    double correctionY = Kp * errorY + Ki * integralY + Kd * derivativeY;

    // 서보 모터 각도 계산 및 제한 (0 ~ 180도 사이로 제한)
    int servoAngleX1 = constrain(90 - correctionX, 0, 180);
    int servoAngleX2 = constrain(90 + correctionX, 0, 180);
    int servoAngleY1 = constrain(90 + correctionY, 0, 180);
    int servoAngleY2 = constrain(90 - correctionY, 0, 180);

    // 서보 모터 제어 (자동 모드)
    mg995_1.write(servoAngleX1);
    mg995_2.write(servoAngleX2);
    mg995_3.write(servoAngleY1);
    mg995_4.write(servoAngleY2);

    // 자이로 데이터 블루투스로 전송 (angleFiX, angleFiY 값)
    sendGyroDataToBluetooth();

    // 이전 오차 저장
    lastErrorX = errorX;
    lastErrorY = errorY;
  }

  // 수동 제어 모드
  if (controlMode == 1) {
    // 현재 각도에서 목표 각도로 서서히 이동 (1도씩)
    if (currentAngleX1 < targetAngleX) {
      currentAngleX1++;
      currentAngleX2--;
    } else if (currentAngleX1 > targetAngleX) {
      currentAngleX1--;
      currentAngleX2++;
    }

    if (currentAngleY1 < targetAngleY) {
      currentAngleY1++;
      currentAngleY2--;
    } else if (currentAngleY1 > targetAngleY) {
      currentAngleY1--;
      currentAngleY2++;
    }

    // 서보 모터에 새로운 각도 적용
    mg995_1.write(currentAngleX1);
    mg995_2.write(currentAngleX2);
    mg995_3.write(currentAngleY1);
    mg995_4.write(currentAngleY2);
  }

  delay(10);  // 주기적으로 제어 (딜레이 줄임)

  filterIndex = (filterIndex + 1) % FILTER_SIZE;
}

void initSensor() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void getData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

void getDT() {
  now = millis();
  dt = (now - past) / 1000.0;
  past = now;
}

void calibrateGyro() {
  double sumGyX = 0, sumGyY = 0, sumGyZ = 0;
  for (int i = 0; i < 100; i++) {
    getData();
    sumGyX += GyX;
    sumGyY += GyY;
    sumGyZ += GyZ;
    delay(10);
  }
  offsetGyX = sumGyX / 100;
  offsetGyY = sumGyY / 100;
  offsetGyZ = sumGyZ / 100;
}

// 자이로 데이터를 블루투스로 전송하는 함수
void sendGyroDataToBluetooth() {
  String gyroData = String(angleFiX) + ", " + String(angleFiY);
  bluetooth.println(gyroData);  // 블루투스로 angleFiX, angleFiY 값 전송
}
