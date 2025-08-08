#include <Arduino.h>
#include <Wire.h>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <message.pb.h>

// PINS
const int X_PIN = 26;
const int Y_PIN = 27;
const int SWITCH_PIN = 22;
const int LED_PIN = 25;

// CONSTANT PARAMETERS
const int X_MIN = 0;
const int X_MAX = 1023;
const int Y_MIN = 0;
const int Y_MAX = 1023;

const int DEADZONE = 3;
const int NUM_SERVOS = 6;
const int EYE_ADDRESS = 0x43;
const unsigned long DELAY_MS = 100;  // 10 Hz

// DEPENDENT CONSTANTS
const int X_MID = (X_MIN - X_MAX) / 2;
const int Y_MID = (Y_MIN - Y_MAX) / 2;

// VARIABLE DECLARATIONS
int x = 0;
int y = 0;
int sw = 0;
int servoNum = 0;
int led = 0;
byte errorCode = 0;
uint8_t buffer[256];
pb_ostream_t stream;
eye_msgs_ServoRequest request = eye_msgs_ServoRequest_init_default;


void setup() {
  pinMode(SWITCH_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);  // for print debugging
  Wire.begin();        // for i2c transmission

 
  Serial.println("All communication started!");
}

// Included for debugging protobuf serialization
void decodeMessage(uint8_t *buffer, size_t bytes_written) {
  eye_msgs_ServoRequest message = eye_msgs_ServoRequest_init_zero;
  pb_istream_t stream = pb_istream_from_buffer(buffer, bytes_written);
  if (!pb_decode(&stream, eye_msgs_ServoRequest_fields, &message)) {
    Serial.println("Decoding failed");
    return;
  }

  Serial.print("angle: ");
  Serial.print(message.angle);
  Serial.print(" | number: ");
  Serial.println(message.servo_number);
}

void loop() {
  // READ AND PROCESS INPUT
  x = analogRead(X_PIN);
  y = analogRead(Y_PIN);

  // if switch goes from low to high
  if (!sw && digitalRead(SWITCH_PIN)) {
    servoNum++;
    if (servoNum >= NUM_SERVOS) {
      servoNum = 0;
    }
  }
  sw = digitalRead(SWITCH_PIN);

  // MAP INTPUT TO SERVO REQUEST
  request.angle = map(constrain(x, X_MIN, X_MAX), X_MIN, X_MAX, 0, 180);
  if (abs(int(request.angle) - 90) <= DEADZONE) {
    request.angle = 90;
  }
  request.servo_number = servoNum;

  // ENCODE SERVO REQUEST
  if (!pb_encode(&stream, eye_msgs_ServoRequest_fields, &request)) {
    Serial.println("Encoding failed");
    return;
  }

  // SEND SERVO REQUEST OVER I2C and SERIAL DEBUG
  Wire.beginTransmission(EYE_ADDRESS);
  for (int i = 0; i < stream.bytes_written; i++) {
    Wire.write(buffer[i]);
  }
  errorCode = Wire.endTransmission();

  // LOG RESULTS TO SERIAL
  if (!errorCode) {
    Serial.print(request.angle);
    Serial.print(" | ");
    Serial.println(request.servo_number);
  } else {
    Serial.print("I2C transmission failed: ");
    Serial.println(errorCode);
  }

  digitalWrite(LED_PIN, led? HIGH : LOW);
  led = !led;
  delay(DELAY_MS);
}
