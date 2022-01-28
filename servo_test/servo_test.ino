/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP32Servo.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates



int g_move[][6]={{135,45,135,135,45,135},{45,135,45,45,135,45}}; // movement angle for first joint from the body
int leg_1_pos[3]={90,90,90};
int leg_1_move[2][3]={{135,45,45},{45,135,135}};



void setup() {
  Serial.begin(115200);
  Serial.println("8 channel Servo test!");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);
  int init_pos_val = 90;
  init_pos_val = map(init_pos_val,0,180,SERVOMIN,SERVOMAX);
  Serial.println(init_pos_val); //375
  int val = 0;
  for (int i = 0 ; i < 6; i ++) {
  val = map(leg_1_pos[i],0,180,SERVOMIN,SERVOMAX);
    pwm.setPWM(i, 0, val);
  }
  delay(2000);
}

void loop() {
  // Drive each servo one at a time using setPWM()
  int val = 0;
//  val = 20;
//  Serial.println(val);
//
//  for (int i = 0 ; i < 6; i ++) {
//    val = map(g_move[0][i],0,180,SERVOMIN,SERVOMAX);
//    pwm.setPWM(i, 0, val);
//  }
//  delay(750);
//  for (int i = 0 ; i < 3; i ++) {
//    val = map(leg_1_pos[i],0,180,SERVOMIN,SERVOMAX);
//    pwm.setPWM(i,0, val);
//  }
//  delay(750);

  for (int i = 0 ; i < 3; i++) {
    val = map(leg_1_move[0][i],0,180,SERVOMIN,SERVOMAX);
    pwm.setPWM(i,0, val);
    delay(100);
  }
  delay(500);
  for (int i = 0 ; i < 3; i++) {
    val = map(leg_1_move[1][i],0,180,SERVOMIN,SERVOMAX);
    pwm.setPWM(i,0, val);
    delay(100);
  }
  delay(500);
}
