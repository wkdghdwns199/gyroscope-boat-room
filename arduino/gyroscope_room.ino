#include <Wire.h>
#include <Servo.h>

const int MPU_ADDR = 0x68;    // I2C통신을 위한 MPU6050의 주소
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;   // 가속도(Acceleration)와 자이로(Gyro)
double angleAcX, angleAcY, angleAcZ;
double angleGyX, angleGyY, angleGyZ;
double angleFiX, angleFiY, angleFiZ;

const double RADIAN_TO_DEGREE = 180 / 3.14159;  
const double DEG_PER_SEC = 32767 / 250;    // 1초에 회전하는 각도
const double ALPHA = 1 / (1 + 0.04);  // 상보필터 상수

unsigned long now = 0;   // 현재 시간 저장용 변수
unsigned long past = 0;  // 이전 시간 저장용 변수
double dt = 0;           // 한 사이클 동안 걸린 시간 변수 

double averAcX, averAcY, averAcZ;
double averGyX, averGyY, averGyZ;

Servo mg995_1;
Servo mg995_2;
Servo mg995_3;
Servo mg995_4;

int servoPin1 = 11;
int servoPin2 = 10;
int servoPin3 = 9;
int servoPin4 = 6;

void setup() {
  initSensor();
  Serial.begin(115200);
  caliSensor();   // 초기 센서 캘리브레이션 함수 호출
  past = millis(); // past에 현재 시간 저장
  mg995_1.attach(servoPin1);
  mg995_2.attach(servoPin2);
  mg995_3.attach(servoPin3);
  mg995_4.attach(servoPin4);
}

void loop() {
  getData(); 
  getDT();

  // 각도 계산
  angleAcX = atan(AcY / sqrt(pow(AcX, 2) + pow(AcZ, 2))) * RADIAN_TO_DEGREE;
  angleAcY = atan(-AcX / sqrt(pow(AcY, 2) + pow(AcZ, 2))) * RADIAN_TO_DEGREE;

  // 자이로 각도 계산
  angleGyX += ((GyX - averGyX) / DEG_PER_SEC) * dt;
  angleGyY += ((GyY - averGyY) / DEG_PER_SEC) * dt;
  angleGyZ += ((GyZ - averGyZ) / DEG_PER_SEC) * dt;

  // 상보필터 적용
  double angleTmpX = angleFiX + angleGyX * dt;
  double angleTmpY = angleFiY + angleGyY * dt;
  angleFiX = ALPHA * angleTmpX + (1.0 - ALPHA) * angleAcX;
  angleFiY = ALPHA * angleTmpY + (1.0 - ALPHA) * angleAcY;
  angleFiZ = angleGyZ;

  // 각도 오차 계산
  double errorX = 0 - angleFiX; // 목표 각도 0도에서 현재 각도를 뺌
  double errorY = 0 - angleFiY;

  // PID 제어 - 현재는 비례 제어만 사용
  double Kp = 10.0;  // 비례 제어 상수
  double correctionX = Kp * errorX;
  double correctionY = Kp * errorY;

  // 서보 모터 제어
  int servoAngleX1 = constrain(90 + correctionX, 0, 180); // 90도를 기준으로 보정
  int servoAngleX2 = constrain(90 - correctionX, 0, 180); // 반대 방향
  int servoAngleY1 = constrain(90 + correctionY, 0, 180); // 90도를 기준으로 보정
  int servoAngleY2 = constrain(90 - correctionY, 0, 180); // 반대 방향

  mg995_1.write(servoAngleX1); // 가로 방향 서보 1
  mg995_2.write(servoAngleX2); // 가로 방향 서보 2
  mg995_3.write(servoAngleY1); // 세로 방향 서보 1
  mg995_4.write(servoAngleY2); // 세로 방향 서보 2

  // 디버깅 정보 출력
  Serial.print("AngleAcX: ");
  Serial.print(angleAcX);
  Serial.print("\t FilteredX: ");
  Serial.print(angleFiX);
  Serial.print("\t AngleAcY: ");
  Serial.print(angleAcY);
  Serial.print("\t FilteredY: ");
  Serial.println(angleFiY);

  delay(20); // 주기적으로 제어
}

void initSensor() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);   // I2C 통신용 어드레스(주소)
  Wire.write(0x6B);    // MPU6050과 통신을 시작하기 위해서는 0x6B번지에    
  Wire.write(0);
  Wire.endTransmission(true);
}

void getData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);   // AcX 레지스터 위치(주소)를 지칭합니다
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);  // AcX 주소 이후의 14byte의 데이터를 요청
  AcX = Wire.read() << 8 | Wire.read(); // 두 개의 나뉘어진 바이트를 하나로 이어 붙여서 각 변수에 저장
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

// loop 한 사이클 동안 걸리는 시간을 알기 위한 함수
void getDT() {
  now = millis();   
  dt = (now - past) / 1000.0;  
  past = now;
}

// 센서의 초기값을 10회 정도 평균값으로 구하여 저장하는 함수
void caliSensor() {
  double sumAcX = 0 , sumAcY = 0, sumAcZ = 0;
  double sumGyX = 0 , sumGyY = 0, sumGyZ = 0;
  getData();
  for (int i = 0; i < 10; i++) {
    getData();
    sumAcX += AcX;  sumAcY += AcY;  sumAcZ += AcZ;
    sumGyX += GyX;  sumGyY += GyY;  sumGyZ += GyZ;
    delay(50);
  }
  averAcX = sumAcX / 10;  averAcY = sumAcY / 10;  averAcZ = sumAcZ / 10;
  averGyX = sumGyX / 10;  averGyY = sumGyY / 10;  averGyZ = sumGyZ / 10;
}
