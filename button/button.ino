#include "Wire.h"
#include <Servo.h>

#define NUM_CONTROL 4
#define LED_PIN LED_BUILTIN  // 13

// Choose to send I2C commands to only certain motors by setting its addres to 0.
// If a motor is not connected but this program tries to send to it, then the
// program will freeze.
const byte i2cAddr[NUM_CONTROL] = {0x09, 0x00, 0x00, 0x00};
//                                {0x09, 0x0A, 0x0B, 0x0E};
const int servoPins[NUM_CONTROL] = {4, 5, 6, 7};
Servo servos[NUM_CONTROL];

#define writeArr(A) (A, sizeof(A))
#define WireSendArr(ADDR, DATA) Wire.beginTransmission(ADDR);\
                                Wire.write(DATA, sizeof(DATA));\
                                Wire.endTransmission();
// First byte is address; the rest is data.
byte disableB[] = {0x1, 0b00000000,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
byte startB[] = {0x1, 0b00010011,0x0,0x0,0x0,0x0,100,0x0,0x0};  // Mode (byte 1) and speed (bytes 4-7)
byte startBSlow[] = {0x1, 0b00010011,0x0,0x0,0x0,0x0,30,0x0,0x0};
byte stopB[] = {0x1, 0b00010011,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
byte reverseB[] = {0x1, 0b00010011,0x0,0x0,0x0,0x0,0x9C,0xFF,0x0};
byte reverseBSlow[] = {0x1, 0b00010011,0x0,0x0,0x0,0x0,0xE2,0xFF,0x0};
byte accelLimit[] = {0x90, 50, 0};  // Acceleration limit (pwm change per ms)
byte currLimit[] = {0x82, 54, 0};  // Current limit, val=limit*(1024/5)*(1/1000)*(66), 54->4 Amps
byte setToutB[] = {0x80, 0x0, 0x08};  // Timeout (ms)

unsigned long time = 0;

void setup() {
  Serial.begin(230400);
  Serial.println("Setup");
  // initialize the LED pin as an output:
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(250);
  
  // Initialize I2C
  Wire.begin();
  for(int i=0; i<NUM_CONTROL; i++) {
    byte addr = i2cAddr[i];
    if(!addr) continue;
    WireSendArr(addr, accelLimit);
    WireSendArr(addr, setToutB);
  }
  
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
  int a = HIGH;
  time = millis();
  
  WireSendArr(i2cAddr[0], startBSlow);
  
  digitalWrite(LED_PIN, a);
  a = !a;
}
