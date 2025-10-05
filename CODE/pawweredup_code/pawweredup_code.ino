#include <Wire.h>
#include <math.h>

// MPU addresses
const int MPU1_ADDR = 0x68;   // A0
const int MPU2_ADDR = 0x69;   // A1

// Servo pins
const int servo1Pin = 2;  // GPIO2
const int servo2Pin = 3;  // GPIO3

// Tare button
const int tarePin = 8;    // D8

// Servo timing
const unsigned long periodUs = 20000UL; // 20ms
const int minPulseUs = 600;
const int maxPulseUs = 2400;

// MPU sensitivities
const float ACC_SENS = 16384.0f;
const float GYRO_SENS = 131.0f;
const float ALPHA = 0.98f;

// Angles
float roll1=0, pitch1=0;
float roll2=0, pitch2=0;

// Tare offsets
float pitchOffset1 = 0;
float pitchOffset2 = 0;

// Moving average buffers
#define AVG_SIZE 10
float pitchBuf1[AVG_SIZE]={0};
float pitchBuf2[AVG_SIZE]={0};
int bufIndex=0;

// Previous servo angles
int prevServo1 = 90;
int prevServo2 = 90;

unsigned long lastMicros;

// ---------- Functions ----------
void writeServoOnce(int pin, int angle) {
  if(angle<0) angle=0;
  if(angle>180) angle=180;
  unsigned int pulse = map(angle, 0, 180, minPulseUs, maxPulseUs);
  digitalWrite(pin,HIGH);
  delayMicroseconds(pulse);
  digitalWrite(pin,LOW);
  delayMicroseconds(periodUs-pulse);
}

void readRawMPU(int addr, int16_t &ax,int16_t &ay,int16_t &az,int16_t &gx,int16_t &gy,int16_t &gz){
  Wire.beginTransmission(addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(addr,14,true);
  ax=(Wire.read()<<8)|Wire.read();
  ay=(Wire.read()<<8)|Wire.read();
  az=(Wire.read()<<8)|Wire.read();
  Wire.read(); Wire.read(); // skip temp
  gx=(Wire.read()<<8)|Wire.read();
  gy=(Wire.read()<<8)|Wire.read();
  gz=(Wire.read()<<8)|Wire.read();
}

int pitchToServo(float p){
  if(p>80) p=80;
  if(p<-80) p=-80;
  float servo;
  if(p>=0) servo=-1.125f*p + 90.0f;
  else servo=-1.0625f*p + 90.0f;
  if(servo<0) servo=0;
  if(servo>180) servo=180;
  return (int)(servo+0.5f);
}

float averageBuffer(float buf[]) {
  float sum=0;
  for(int i=0;i<AVG_SIZE;i++) sum+=buf[i];
  return sum/AVG_SIZE;
}

// ---------- Setup ----------
void setup(){
  pinMode(servo1Pin, OUTPUT);
  pinMode(servo2Pin, OUTPUT);
  digitalWrite(servo1Pin, LOW);
  digitalWrite(servo2Pin, LOW);

  pinMode(tarePin, INPUT_PULLUP);

  Wire.begin();
  Serial.begin(115200);

  // Wake both MPUs
  Wire.beginTransmission(MPU1_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
  Wire.beginTransmission(MPU2_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
  delay(100);

  lastMicros = micros();
}

// ---------- Loop ----------
void loop(){
  // Check tare button
  if(digitalRead(tarePin)==LOW){
    pitchOffset1 = 0;
    pitchOffset2 = 0;

    /*// MPU1
    readRawMPU(MPU1_ADDR,ax,ay,az,gx,gy,gz);
    float axF=ax/ACC_SENS, ayF=ay/ACC_SENS, azF=az/ACC_SENS;
    pitchOffset1 = atan2(-axF,sqrt(ayF*ayF+azF*azF)) * 180.0f/M_PI;

    // MPU2
    readRawMPU(MPU2_ADDR,ax,ay,az,gx,gy,gz);
    axF=ax/ACC_SENS; ayF=ay/ACC_SENS; azF=az/ACC_SENS;
    pitchOffset2 = atan2(-axF,sqrt(ayF*ayF+azF*azF)) * 180.0f/M_PI;*/

    Serial.println("TARE activated!");
    delay(500); // debounce
  }

  unsigned long now = micros();
  float dt = (now - lastMicros)/1e6f;
  if(dt<=0) dt=0.001f;
  lastMicros = now;

  // ---- MPU1 ----
  int16_t ax1,ay1,az1,gx1,gy1,gz1;
  readRawMPU(MPU1_ADDR,ax1,ay1,az1,gx1,gy1,gz1);
  float axF1=ax1/ACC_SENS, ayF1=ay1/ACC_SENS, azF1=az1/ACC_SENS;
  float gxF1=gx1/GYRO_SENS, gyF1=gy1/GYRO_SENS;

  float pitchAcc1 = atan2(-axF1,sqrt(ayF1*ayF1+azF1*azF1))*180.0f/M_PI - pitchOffset1;

  static bool first1=true;
  if(first1){pitch1=pitchAcc1; first1=false;}

  pitch1 = ALPHA*(pitch1+gyF1*dt)+(1-ALPHA)*pitchAcc1;

  // moving average
  pitchBuf1[bufIndex] = pitch1;
  float avgPitch1 = averageBuffer(pitchBuf1);

  int servoAngle1 = prevServo1;
  if(fabs(avgPitch1 - prevServo1) > 2.0) { 
    servoAngle1 = pitchToServo(avgPitch1);
    writeServoOnce(servo1Pin, servoAngle1);
    prevServo1 = servoAngle1;
  }

  // ---- MPU2 ----
  int16_t ax2,ay2,az2,gx2,gy2,gz2;
  readRawMPU(MPU2_ADDR,ax2,ay2,az2,gx2,gy2,gz2);
  float axF2=ax2/ACC_SENS, ayF2=ay2/ACC_SENS, azF2=az2/ACC_SENS;
  float gxF2=gx2/GYRO_SENS, gyF2=gy2/GYRO_SENS;

  float pitchAcc2 = atan2(-axF2,sqrt(ayF2*ayF2+azF2*azF2))*180.0f/M_PI - pitchOffset2;

  static bool first2=true;
  if(first2){pitch2=pitchAcc2; first2=false;}

  pitch2 = ALPHA*(pitch2+gyF2*dt)+(1-ALPHA)*pitchAcc2;

  // moving average
  pitchBuf2[bufIndex] = pitch2;
  float avgPitch2 = averageBuffer(pitchBuf2);

  int servoAngle2 = prevServo2;
  if(fabs(avgPitch2 - prevServo2) > 2.0) {
    servoAngle2 = pitchToServo(avgPitch2);
    writeServoOnce(servo2Pin, servoAngle2);
    prevServo2 = servoAngle2;
  }

  bufIndex++;
  if(bufIndex >= AVG_SIZE) bufIndex = 0;

  // ---- Debug ----
  Serial.print("Avg MPU1 Pitch: "); Serial.print(avgPitch1,2); 
  Serial.print("  Servo1: "); Serial.print(prevServo1);
  Serial.print(" | Avg MPU2 Pitch: "); Serial.print(avgPitch2,2);
  Serial.print("  Servo2: "); Serial.println(prevServo2);
}
