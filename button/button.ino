#include "Wire.h"
#include <Servo.h>

#define NUM_CONTROL 4
#define LED_PIN LED_BUILTIN  // 13

// Choose to send I2C commands to only certain motors by setting its addres to 0.
// If a motor is not connected but this program tries to send to it, then the
// program will freeze.
const byte i2cAddr[NUM_CONTROL] = {0x09, 0x0A, 0x0B, 0x0E};
//                                {0x09, 0x0A, 0x0B, 0x0E};
const int servoPins[NUM_CONTROL] = {4, 5, 6, 7};

// Servo send
Servo servos[NUM_CONTROL];

// Grizzly send
#define writeArr(A) (A, sizeof(A))
#define WireSendArr(ADDR, DATA) {Wire.beginTransmission(ADDR);\
                                Wire.write(DATA, sizeof(DATA));\
                                Wire.endTransmission();}
// First byte is address; the rest is data.  (little endian byte order)
byte disableB[] = {0x1, 0b00000000,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
byte startB[] = {0x1, 0b00010011,0x0,0x0,0x0,0x0,100,0x0,0x0};  // Mode (byte 1) and speed (bytes 4-7)
byte startBSlow[] = {0x1, 0b00010011,0x0,0x0,0x0,0x0,30,0x0,0x0};
byte stopB[] = {0x1, 0b00010011,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
byte reverseB[] = {0x1, 0b00010011,0x0,0x0,0x0,0x0,0x9C,0xFF,0x0};
byte reverseBSlow[] = {0x1, 0b00010011,0x0,0x0,0x0,0x0,0xE2,0xFF,0x0};
byte accelLimit[] = {0x90, 50, 0};  // Acceleration limit (pwm change per ms)
byte currLimit[] = {0x82, 54, 0};  // Current limit, val=limit*(1024/5)*(1/1000)*(66), 54->4 Amps
byte setToutB[] = {0x80, 0xFA, 0x00};  // Timeout (ms)
byte setPID[] = {0x30,   0, 0, 10, 0,   0x8F,0x02,0,0,   0x9A,0x19,0,0};  // PID=(10, .01, .1)
byte pidTargetB[] = {0x1, 0b00010111, 0x0,0x0,  0x0,0x0,0x0,0x0,  0x0};  // Set target, mode=PID position

// Command recieve
#define HEADER_LEN 4
#define PACKET_LEN 24
byte buffer[HEADER_LEN+PACKET_LEN];
int bufferFill = 0;

unsigned long time = 0;
unsigned long lastTime = 0;
unsigned long lastInitTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Setup");
  // initialize the LED pin as an output:
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(250);
  
  initI2C();
  
  digitalWrite(LED_PIN, LOW);
  delay(250);
  digitalWrite(LED_PIN, HIGH);
  delay(250);
  
  // Initialize servos
  for(int i=0; i<NUM_CONTROL; i++) {
    int pin = servoPins[i];
    if(pin < 0) continue;
    servos[i].attach(pin);
    //servos[i].write(90);
  }
  
  digitalWrite(LED_PIN, LOW);
  delay(250);
  
  Serial.println("Start");
}

void loop() {
  int a = HIGH, b=0;
  time = millis();
  if(time - lastInitTime >= 2000) {
    lastInitTime = time;
    initI2C();
  }

  recieveSerial();

  if(bufferFill >= HEADER_LEN+PACKET_LEN) {
    setMotors();

    setServos();
  }
  
  digitalWrite(LED_PIN, a);
  if(bufferFill >= HEADER_LEN+PACKET_LEN) {
    a = a==HIGH ? LOW : HIGH;
  }
  
  lastTime = time;
}

void initI2C() {
  // Initialize I2C
  Wire.begin();
  for(int i=0; i<NUM_CONTROL; i++) {
    byte addr = i2cAddr[i];
    if(!addr) continue;
    WireSendArr(addr, accelLimit);
    WireSendArr(addr, setToutB);
    WireSendArr(addr, setPID);
  }
}

void recieveSerial() {
  if(bufferFill >= HEADER_LEN+PACKET_LEN) {
    bufferFill = 0;
  }

  int incoming;
  while(bufferFill < HEADER_LEN+PACKET_LEN && (incoming = Serial.read()) >= 0) {
    buffer[bufferFill] = incoming;
    if(bufferFill < HEADER_LEN) {  // Wait until valid header is recieved
      if(incoming == 0xFF ||
         (bufferFill == HEADER_LEN-1 && incoming == 0xFE)) {
        bufferFill++;
      } else {
        bufferFill = 0;
        Serial.println("Miss");
      }
    } else {  // Recieve the packet
      bufferFill++;
    }
  }
}

void setMotors() {
  int speed = (time/100)%200-100;
  
  for(int i=0; i<NUM_CONTROL; i++) {
    pidTargetB[4] = buffer[HEADER_LEN+i*4];
    pidTargetB[5] = buffer[HEADER_LEN+i*4+1];
    pidTargetB[6] = buffer[HEADER_LEN+i*4+2];
    pidTargetB[7] = buffer[HEADER_LEN+i*4+3];
    WireSendArr(i2cAddr[i], pidTargetB);
  }
}

void setServos() {
  int angle = (time/100)%180;
  
  int *angles = (int*)&(buffer[HEADER_LEN+NUM_CONTROL*4]);  // int is 16 bits
  for(int i=0; i<NUM_CONTROL; i++) {
    servos[i].write(angles[i]);
  }
}








