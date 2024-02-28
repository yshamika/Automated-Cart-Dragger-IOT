#include <AFMotor.h>
#include <SoftwareSerial.h>
#include<NewPing.h>
#include<Servo.h>

#define TRIGGER_PIN A1
#define ECHO_PIN A0
#define MAX_DISTANCE 100

SoftwareSerial bluetoothSerial(9, 10); // RX, TX
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

//motors pin
AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

char command;

void setup()
{
  Serial.begin(9600);
  bluetoothSerial.begin(9600);  // Bt module.
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

}

void loop() {
  if (bluetoothSerial.available() > 0) {
    command = bluetoothSerial.read();
    Stop();
    switch (command) {
      case 'F':
        b_forward();
        break;
      case 'B':
        b_back();
        break;
      case 'L':
        b_left();
        break;
      case 'R':
        b_right();
        break;
        case 'W':
        pk();
        break;
    }
  }
  else{
    unsigned int distance = sonar.ping_cm();
    Serial.print("distance");
    Serial.println(distance);

    int Right_Value = digitalRead(A2);
    int Left_Value = digitalRead(A3);

    Serial.print("RIGHT");
    Serial.println(Right_Value);
    Serial.print("LEFT");
    Serial.println(Left_Value);

    if((Right_Value==1) && (distance>=10 && distance<=30)&&(Left_Value==1)){
      forward();
    }
    else if((Right_Value==1) && (distance<10) &&(Left_Value==1)){
      back();
    }
    else if((Right_Value==0) && (Left_Value==1)) {
      right();
    }   
    else if((Right_Value==1)&&(Left_Value==0)) {
      left();
    }
    else if((Right_Value==1)&&(Left_Value==1)) {
      Stop();
    }
    else if(distance > 1 && distance < 10) {
      Stop();
    }


  }
}




//controllers
void forward()
{
  delay(2);
  motor1.setSpeed(200); 
  motor1.run(FORWARD);  
  motor2.setSpeed(200); 
  motor2.run(FORWARD);  
  motor3.setSpeed(200); 
  motor3.run(FORWARD);  
  motor4.setSpeed(200); 
  motor4.run(FORWARD); 
} 

void back()
{
  delay(2);
  motor1.setSpeed(200);
  motor1.run(BACKWARD); 
  motor2.setSpeed(200);
  motor2.run(BACKWARD); //rotate the motor anti-clockwise
  motor3.setSpeed(200); //Define maximum velocity
  motor3.run(BACKWARD); //rotate the motor anti-clockwise
  motor4.setSpeed(200); //Define maximum velocity
  motor4.run(BACKWARD); //rotate the motor anti-clockwise
}

void left()
{
  delay(2);
  motor1.setSpeed(180); //Define maximum velocity
  motor1.run(BACKWARD); //rotate the motor anti-clockwise
  motor2.setSpeed(180); //Define maximum velocity
  motor2.run(BACKWARD); //rotate the motor anti-clockwise
  motor3.setSpeed(180); //Define maximum velocity
  motor3.run(FORWARD);  //rotate the motor clockwise
  motor4.setSpeed(180); //Define maximum velocity
  motor4.run(FORWARD);  //rotate the motor clockwise
}

void right()
{
  delay(2);
  motor1.setSpeed(180); //Define maximum velocity
  motor1.run(FORWARD);  //rotate the motor clockwise
  motor2.setSpeed(180); //Define maximum velocity
  motor2.run(FORWARD);  //rotate the motor clockwise
  motor3.setSpeed(180); //Define maximum velocity
  motor3.run(BACKWARD); //rotate the motor anti-clockwise
  motor4.setSpeed(180); //Define maximum velocity
  motor4.run(BACKWARD); //rotate the motor anti-clockwise
}

void Stop()
{
  delay(2);
  motor1.setSpeed(0);  //Define minimum velocity
  motor1.run(RELEASE); //stop the motor when release the button
  motor2.setSpeed(0);  //Define minimum velocity
  motor2.run(RELEASE); //rotate the motor clockwise
  motor3.setSpeed(0);  //Define minimum velocity
  motor3.run(RELEASE); //stop the motor when release the button
  motor4.setSpeed(0);  //Define minimum velocity
  motor4.run(RELEASE); //stop the motor when release the button
}

// for blutooth

void b_forward()
{
  delay(2);
  motor1.setSpeed(255); //Define maximum velocity
  motor1.run(FORWARD);  //rotate the motor clockwise
  motor2.setSpeed(255); //Define maximum velocity
  motor2.run(FORWARD);  //rotate the motor clockwise
  motor3.setSpeed(255); //Define maximum velocity
  motor3.run(FORWARD);  //rotate the motor clockwise
  motor4.setSpeed(255); //Define maximum velocity
  motor4.run(FORWARD);  //rotate the motor clockwise
}

void b_back()
{
  delay(2);
  motor1.setSpeed(255); //Define maximum velocity
  motor1.run(BACKWARD); //rotate the motor anti-clockwise
  motor2.setSpeed(255); //Define maximum velocity
  motor2.run(BACKWARD); //rotate the motor anti-clockwise
  motor3.setSpeed(255); //Define maximum velocity
  motor3.run(BACKWARD); //rotate the motor anti-clockwise
  motor4.setSpeed(255); //Define maximum velocity
  motor4.run(BACKWARD); //rotate the motor anti-clockwise
}

void b_left()
{
  delay(2);
  motor1.setSpeed(255); //Define maximum velocity
  motor1.run(BACKWARD); //rotate the motor anti-clockwise
  motor2.setSpeed(255); //Define maximum velocity
  motor2.run(BACKWARD); //rotate the motor anti-clockwise
  motor3.setSpeed(255); //Define maximum velocity
  motor3.run(FORWARD);  //rotate the motor clockwise
  motor4.setSpeed(255); //Define maximum velocity
  motor4.run(FORWARD);  //rotate the motor clockwise
}

void b_right()
{
  delay(2);
  motor1.setSpeed(255); //Define maximum velocity
  motor1.run(FORWARD);  //rotate the motor clockwise
  motor2.setSpeed(255); //Define maximum velocity
  motor2.run(FORWARD);  //rotate the motor clockwise
  motor3.setSpeed(255); //Define maximum velocity
  motor3.run(BACKWARD); //rotate the motor anti-clockwise
  motor4.setSpeed(255); //Define maximum velocity
  motor4.run(BACKWARD); //rotate the motor anti-clockwise
}

// my one
void pk()
{
  delay(2);
  motor1.setSpeed(200); 
  motor1.run(FORWARD);  
  motor2.setSpeed(200); 
  motor2.run(FORWARD);  
  motor3.setSpeed(200); 
  motor3.run(FORWARD);  
  motor4.setSpeed(200); 
  motor4.run(FORWARD);
  delay(5);
  motor1.setSpeed(255); //Define maximum velocity
  motor1.run(FORWARD);  //rotate the motor clockwise
  motor2.setSpeed(255); //Define maximum velocity
  motor2.run(FORWARD);  //rotate the motor clockwise
  motor3.setSpeed(255); //Define maximum velocity
  motor3.run(BACKWARD); //rotate the motor anti-clockwise
  motor4.setSpeed(255); //Define maximum velocity
  motor4.run(BACKWARD); //rotate the motor anti-clockwise
  delay(5);
  motor1.setSpeed(255); //Define maximum velocity
  motor1.run(BACKWARD); //rotate the motor anti-clockwise
  motor2.setSpeed(255); //Define maximum velocity
  motor2.run(BACKWARD); //rotate the motor anti-clockwise
  motor3.setSpeed(255); //Define maximum velocity
  motor3.run(FORWARD);  //rotate the motor clockwise
  motor4.setSpeed(255); //Define maximum velocity
  motor4.run(FORWARD);  //rotate the motor clockwise
  delay(5);
  motor1.setSpeed(255); //Define maximum velocity
  motor1.run(BACKWARD); //rotate the motor anti-clockwise
  motor2.setSpeed(255); //Define maximum velocity
  motor2.run(BACKWARD); //rotate the motor anti-clockwise
  motor3.setSpeed(255); //Define maximum velocity
  motor3.run(BACKWARD); //rotate the motor anti-clockwise
  motor4.setSpeed(255); //Define maximum velocity
  motor4.run(BACKWARD); //rotate the motor anti-clockwise
}