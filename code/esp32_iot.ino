#include <Wire.h>
#include "thingProperties.h"            // this link us to cloud
#include <ESP32Servo.h>                 // Servo library
#include <NewPing.h>                    // Ultrasonic Sensor library
#include <Adafruit_MPU6050.h>           // for mpu 6050 
#include <Adafruit_Sensor.h>



//................................................... for servo movement
static const int servoPin = 23;
Servo myservo;
int minangle =40;
int maxangle =140;

//.................................................... for ultrasonic sensor
#define TRIGGER_PIN 12
#define ECHO_PIN 14
#define MAX_DISTANCE 100
int maxDistance = 60;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// ...................................................for arc reactor led
#define LED_PIN 13
int limit = 5; 


//...................................................for mpu 6050
Adafruit_MPU6050 mpu;



void setup() {


  Serial.begin(9600);
  delay(1500);

  //                                                      [servo setup]
  myservo.attach(servoPin);

  //                                                      [led setup]
  pinMode(LED_PIN, OUTPUT);


  //                                                     [mpu 6050 setup]
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Connected Successfully!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);


  //                                                     [cloud setup]
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
}


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


void controlLEDBrightness(int distance) {
  int brightness = 0;
  //if (distance > limit && distance <= maxDistance) {   
  brightness = map(distance, limit , maxDistance, 255, 0);
  light = brightness;
  ArduinoCloud.update();
  analogWrite(LED_PIN, brightness);
  } 
  
  /*else {
    digitalWrite(LED_PIN, LOW); // Turn off LED if no object detected
  }*/
  

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


void blinkLED(int times, int delayTime) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayTime);
    digitalWrite(LED_PIN, LOW);
    delay(delayTime);
  }
}

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


void moveHead() {
  static int position = minangle;
  static int increment = 3;
  bool objectDetected = false;
  myservo.write(position);
  servo = position;

  unsigned int distance = sonar.ping_cm();
  ultrasonic = distance;
  ArduinoCloud.update();

  // Led brightness 

  if (distance == 0 || distance > maxDistance) 
  {                                                       // when nothing detected turn of led and save battery 
     digitalWrite(LED_PIN, LOW);
  } 

  
  if (distance > 1 && distance <= limit)
  {                                                        // for detection of very close object it will blick led
    objectDetected = true;
    blinkLED(3 , 500);
  }


  if (distance > limit && distance <= maxDistance) 
  {                                                        //for detection of person nearby at given distance
    objectDetected = true;
    controlLEDBrightness(distance);
  }
    
  
  
  
  Serial.print("Servo position: ");
  Serial.print(position);
  Serial.print(" | Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (!objectDetected) {                                  // if detecs something no incrementation value given to servo... i will stop rotating
    position += increment;
    if (position >= maxangle || position <= minangle ) {
      increment = -increment;
    }
  }

  delay(10);  // head rotation speed
}

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

void mpusensor() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  acceX = a.acceleration.x;
  acceY = a.acceleration.y;

  ArduinoCloud.update();

  Serial.print(" | aX: "); Serial.print(acceX);
  Serial.print(" | aY: "); Serial.println(acceY);
  delay(100);

}




// ##########################################################################################################


void loop() {
  
  ArduinoCloud.update();
  moveHead();
  mpusensor(); 

}

// #########################################################################################################


void onAcceXChange() {
  
}

void onServoChange() {
  
}

void onLightChange() {
 
}

void onUltrasonicChange() {
  
}

void onAcceYChange(){

}


