#include <IRremote.h>
#include <ESP32Servo.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

const int RECV_PIN = 15;
uint32_t lastIRCode = 0; // Stores the last received IR code

int leftLed = 12;
int rightLed = 14;
int upLed = 27;
int downLed = 26;

int spotlightLed = 21;

bool spotLightStatus = false;

Servo servo1;
Servo servo2;

int servo1_pos = 90;
int servo2_pos = 90;

int servo1Pin = 22;
int servo2Pin = 23;

void setup() {
  Serial.begin(9600);
  IrReceiver.begin(RECV_PIN, ENABLE_LED_FEEDBACK);

  SerialBT.begin("SpotLight_Controller");
  Serial.println("Bluetooth Started! Connect and send data...");
  
  pinMode(leftLed, OUTPUT);
  pinMode(rightLed, OUTPUT);
  pinMode(upLed, OUTPUT);
  pinMode(downLed, OUTPUT);

  pinMode(spotlightLed, OUTPUT);

  servo1.attach(servo1Pin, 500, 2500); // ESP32 supports 500-2500 microseconds pulse width
  servo2.attach(servo2Pin, 500, 2500);
  servo1.write(servo1_pos);
  servo2.write(servo2_pos);

  digitalWrite(leftLed, HIGH);
  delay(20);
  digitalWrite(leftLed, LOW);
  digitalWrite(rightLed, HIGH);
  delay(20);
  digitalWrite(rightLed, LOW);
  digitalWrite(upLed, HIGH);
  delay(20);
  digitalWrite(upLed, LOW);
  digitalWrite(downLed, HIGH);
  delay(20);
  digitalWrite(downLed, LOW);
}

uint32_t getIRSignal() {
  if (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
    lastIRCode = IrReceiver.decodedIRData.decodedRawData;
  }
  IrReceiver.resume();
  return lastIRCode;
}

void loop() {
  digitalWrite(leftLed, (servo1_pos == 0) ? HIGH : LOW);
  digitalWrite(rightLed, (servo1_pos == 180) ? HIGH : LOW);
  digitalWrite(upLed, (servo2_pos == 180) ? HIGH : LOW);
  digitalWrite(downLed, (servo2_pos == 0) ? HIGH : LOW);

  if (SerialBT.available()) {
      if (SerialBT.read() == 0xFF) {
        int data[6];
        for (int i = 0; i < 6; i++) {
            while (!SerialBT.available());
            data[i] = SerialBT.read();
        }
        processBltData(data);
      }
  }

  if (IrReceiver.decode()) {  
    uint32_t irValue = getIRSignal();
    Serial.println(irValue, HEX);

    if (irValue == 0xF708FF00) controlServo1(-1);
    else if (irValue == 0xA55AFF00) controlServo1(1);
    else if (irValue == 0xE718FF00) controlServo2(1);
    else if (irValue == 0xAD52FF00) controlServo2(-1);
    else if (irValue == 0xE31CFF00) centerAll();
    else if (irValue == 0xF30CFF00) topLeft();
    else if (irValue == 0xA15EFF00) topRight();
    else if (irValue == 0xBD42FF00) bottomLeft();
    else if (irValue == 0xB54AFF00) bottomRight();
    else if (irValue == 0xBA45FF00) { 
      toggleSpotLight(); 
      delay(500);
    }
  }

}

void controlServo1(int direction){
  int target = servo1_pos + (direction * 5);
  target = constrain(target, 0, 180); // Ensure within valid range
  
  for (; servo1_pos != target; servo1_pos += (direction < 0 ? -1 : 1)) {
    servo1.write(servo1_pos);
    delay(10); // Small delay for smooth transition
  }
  
  if (servo1_pos == 0) digitalWrite(leftLed, HIGH);
  if (servo1_pos == 180) digitalWrite(rightLed, HIGH);
}


void controlServo2(int direction){
  int target = servo2_pos + (direction * 5);
  target = constrain(target, 0, 180); // Ensure within valid range
  
  for (; servo2_pos != target; servo2_pos += (direction < 0 ? -1 : 1)) {
    servo2.write(servo2_pos);
    delay(10); // Small delay for smooth transition
  }
  
  if (servo2_pos == 0) digitalWrite(downLed, HIGH);
  if (servo2_pos == 180) digitalWrite(upLed, HIGH);
}


void processBltData(int packet[6]){
  Serial.print("Motor: ");
  Serial.print(packet[1]);
  Serial.print("  Degree : ");
  Serial.println(packet[4]);
  if(packet[1]==3){
    servo1_pos=packet[4];
    servo1.write(servo1_pos);
  }
  if(packet[1]==4){
    servo2_pos=packet[4];
    servo2.write(servo2_pos);
  }
}

void centerAll(){
  servo1_pos = 90;
  servo2_pos = 90;
  servo1.write(servo1_pos);
  servo2.write(servo2_pos);
}
void topLeft(){
  servo1_pos = 0;
  servo2_pos = 180;
  servo1.write(servo1_pos);
  servo2.write(servo2_pos);
}
void topRight(){
  servo1_pos = 180;
  servo2_pos = 180;
  servo1.write(servo1_pos);
  servo2.write(servo2_pos);
}
void bottomLeft(){
  servo1_pos = 0;
  servo2_pos = 0;
  servo1.write(servo1_pos);
  servo2.write(servo2_pos);
}
void bottomRight(){
  servo1_pos = 180;
  servo2_pos = 0;
  servo1.write(servo1_pos);
  servo2.write(servo2_pos);
}

void toggleSpotLight(){
  digitalWrite(spotlightLed, (spotLightStatus == true) ? LOW : HIGH);
  spotLightStatus = (spotLightStatus == true)? false : true;
  Serial.print(spotLightStatus);
}
