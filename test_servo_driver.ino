#include <Wire.h>
#include <Servo.h>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <message.pb.h>

// PINS
#define LR_PIN 15
#define UD_PIN 14
#define BL_PIN 13
#define TL_PIN 12
#define BR_PIN 11
#define TR_PIN 10
#define SDA 4
#define SCL 5

// CONSTANT PARAMETERS
#define I2C_ADDRESS 0x08
#define BAUD 115200
#define DELAY_MS 200

// DEPENDENT CONSTANTS
#define NUM_SERVOS 8
const String servos[NUM_SERVOS] = {"LR", "UD", "BL", "TL", "BR", "TR"};

// VARIABLES
Servo lookLR;  
Servo lookUD;  
Servo lidBL;  
Servo lidTL;  
Servo lidBR;  
Servo lidTR;  

int UDState = 90;
int LRState = 90;
int buttonState = 0;
int lidMod = 0;
uint8_t rx_buffer[EYE_MSGS_MESSAGE_PB_H_MAX_SIZE];
eye_msgs_ServoRequest message = eye_msgs_ServoRequest_init_zero;

int pos = 0;    // variable to store the servo position

void setup() {
  lookLR.attach(LR_PIN);
  lookUD.attach(UD_PIN);
  lidBL.attach(BL_PIN);
  lidTL.attach(TL_PIN);
  lidBR.attach(BR_PIN);
  lidTR.attach(TR_PIN);

  Serial.begin(BAUD);  // for print debugging
  Wire.setSDA(SDA);
  Wire.setSCL(SCL);
  Wire.begin(I2C_ADDRESS);  // for i2c receiving
  Wire.onReceive(handleProtobuf);

  delay(1000);
  Serial.print("Communication started! address ");
  Serial.println(I2C_ADDRESS);
}

void loop() {
  delay(DELAY_MS);
}

void handleProtobuf(int numBytes) {
  int idx = 0;

  while (Wire.available() && idx < EYE_MSGS_MESSAGE_PB_H_MAX_SIZE) {  // loop through all but the last
    // rx_buffer[idx++] = Wire.read(); // receive byte as a character
    Serial.print("Received: ");
    Serial.println(Wire.read(), HEX);
  }

  pb_istream_t stream = pb_istream_from_buffer(rx_buffer, idx);

  if (!pb_decode(&stream, eye_msgs_ServoRequest_fields, &message)) {
    Serial.print("Decoding failed: ");
    Serial.println(PB_GET_ERROR(&stream));
    return;
  }

  Serial.print("angle: ");
  Serial.print(message.angle);
  Serial.print(" | number: ");
  Serial.println(message.servo_number);

  // buttonState =  digitalRead(BUTPin);
  // UDState = map(analogRead(UDPin), 0, 1023, 50, 130);
  // lidMod = (40 - UDState)/2;
  // LRState = map(analogRead(LRPin), 0, 1023, 40, 140);  
  
  // //Serial.println(buttonState);

  // lookUD.write(UDState);
  // lookLR.write(LRState);

  // if (buttonState == 0){
  //   lidBL.write(90);
  //   lidTL.write(90);
  //   lidBR.write(90);
  //   lidTR.write(90);
  // } else {
  //   lidBL.write(160+lidMod);
  //   lidTL.write(70+lidMod);
  //   lidBR.write(30-lidMod);
  //   lidTR.write(110-lidMod);
  // }
}
