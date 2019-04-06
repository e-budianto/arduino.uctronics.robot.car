/*
  HG7881_Motor_Driver_Example - Arduino sketch
   
  This example shows how to drive a motor with using HG7881 (L9110) Dual
  Channel Motor Driver Module.  For simplicity, this example shows how to
  drive a single motor.  Both channels work the same way.
   
  This example is meant to illustrate how to operate the motor driver
  and is not intended to be elegant, efficient or useful.
   
  Connections:
   
    Arduino digital output D10 to motor driver input B-IA.
    Arduino digital output D11 to motor driver input B-IB.
    Motor driver VCC to operating voltage 5V.
    Motor driver GND to common ground.
    Motor driver MOTOR B screw terminals to a small motor.
     
  Related Banana Robotics items:
   
    BR010038 HG7881 (L9110) Dual Channel Motor Driver Module
    https://www.BananaRobotics.com/shop/HG7881-(L9110)-Dual-Channel-Motor-Driver-Module
 
  https://www.BananaRobotics.com
*/
 
// wired connections
#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>

//#define HG7881_B_IA 16 // D10 --> Motor B Input A --> MOTOR B +
//#define HG7881_B_IB 4 // D11 --> Motor B Input B --> MOTOR B -
//#define HG7881_A_IA 5 // D10 --> Motor B Input A --> MOTOR B +
//#define HG7881_A_IB 0 // D11 --> Motor B Input B --> MOTOR B -

#define HG7881_A_IA 16 // D0 --> Motor B Input A --> MOTOR B +
#define HG7881_B_IA 5 // D1 --> Motor B Input A --> MOTOR B +
#define HG7881_A_IB 4 // D2 --> Motor B Input B --> MOTOR B -
#define HG7881_B_IB 0 // D3 --> Motor B Input B --> MOTOR B -

// functional connections
#define MOTOR_B_PWM HG7881_B_IA // Motor B PWM Speed
#define MOTOR_B_DIR HG7881_B_IB // Motor B Direction
#define MOTOR_A_PWM HG7881_A_IA // Motor B PWM Speed
#define MOTOR_A_DIR HG7881_A_IB // Motor B Direction

//HC-SR04 (5V) - ultrasonic sensor to measure distance
#define SONAR_TRIG_PIN 14  // D5
#define SONAR_ECHO_PIN 12  // D6
#define SONAR_INTERRUPT 13 //Note: pin D7
//SG90 9G Mini Servo (5V)
#define SERVO_PIN 2 //D4 


// the actual values for "fast" and "slow" depend on the motor
#define PWM_SLOW 50  // arbitrary slow speed PWM duty cycle
#define PWM_FAST 200 // arbitrary fast speed PWM duty cycle
#define DIR_DELAY 1000 // brief delay for abrupt motor changes
 
const char *ssid = "Home";
const char *password = "edward77";
const char *mqtt_server = "192.168.2.112";
const char *userName = "mqtt_user";
const char *passWord = "edward";

volatile int distance = -1;
volatile long distCalcStartTime = 0;

Servo servo;
WiFiClient espClient;
PubSubClient client(espClient);


String switch1;
String strTopic;
String strPayload;
int deg;

long lastMsg=0;

void Stop()
{
        digitalWrite( MOTOR_B_DIR, LOW );
        digitalWrite( MOTOR_A_DIR, LOW );
        digitalWrite( MOTOR_B_PWM, LOW );
        digitalWrite( MOTOR_A_PWM, LOW );
}


void moveForward(float seconds)
{
  lastMsg = millis();
  while (millis() - lastMsg < (seconds * 1000))
     {
        digitalWrite( MOTOR_B_DIR, HIGH ); // direction = forward
        digitalWrite( MOTOR_B_PWM, LOW ); // direction = forward
        digitalWrite( MOTOR_A_DIR, HIGH ); // direction = forward
        digitalWrite( MOTOR_A_PWM, LOW ); // direction = forward
        yield();
     }

}

void moveBackward(float seconds)
{
  lastMsg = millis();
  while (millis() - lastMsg < (seconds * 1000))
     {
        digitalWrite( MOTOR_B_DIR, LOW ); // direction = reverse
        digitalWrite( MOTOR_A_DIR, LOW ); // direction = reverse
        digitalWrite( MOTOR_B_PWM, HIGH ); // direction = reverse
        digitalWrite( MOTOR_A_PWM, HIGH ); // direction = reverse
        yield();
     }

}

void spinLeft(float seconds)
{
  lastMsg = millis();
  while (millis() - lastMsg < (seconds * 1000))
     {
        digitalWrite( MOTOR_B_DIR, HIGH );
        digitalWrite( MOTOR_B_PWM, LOW );
        digitalWrite( MOTOR_A_DIR, LOW );
        digitalWrite( MOTOR_A_PWM, HIGH );
        yield();
     }

}

void turnLeft(float seconds)
{
  lastMsg = millis();
  while (millis() - lastMsg < (seconds * 1000))
     {
        digitalWrite( MOTOR_B_DIR, HIGH );
        digitalWrite( MOTOR_B_PWM, LOW );
        digitalWrite( MOTOR_A_DIR, LOW );
        digitalWrite( MOTOR_A_PWM, LOW );
        yield();
     }

}

void spinRight(float seconds)
{
  lastMsg = millis();
  while (millis() - lastMsg < (seconds * 1000))
     {
        digitalWrite( MOTOR_B_DIR, LOW );
        digitalWrite( MOTOR_B_PWM, HIGH );
        digitalWrite( MOTOR_A_DIR, HIGH );
        digitalWrite( MOTOR_A_PWM, LOW );
        yield();
     }

}

void turnRight(float seconds)
{
  lastMsg = millis();
  while (millis() - lastMsg < (seconds * 1000))
     {
        digitalWrite( MOTOR_B_DIR, LOW );
        digitalWrite( MOTOR_B_PWM, LOW );
        digitalWrite( MOTOR_A_DIR, HIGH );
        digitalWrite( MOTOR_A_PWM, LOW );
        yield();
     }

}

void startDistanceCalculation()
{
   //Clear the SONAR_TRIG_PIN
   digitalWrite(SONAR_TRIG_PIN, LOW);
   delayMicroseconds(2);

   //Set the SONAR_TRIG_PIN on HIGH state for 10 micro seconds
   digitalWrite(SONAR_TRIG_PIN, HIGH);
   delayMicroseconds(10);
   digitalWrite(SONAR_TRIG_PIN, LOW);
}

void calculateDistance()
{
  switch(digitalRead(SONAR_ECHO_PIN))
  {
    //Start of pulse
    case HIGH:
      distCalcStartTime = micros();
      break;

    //Pulse done; calculate distance in cm
    case LOW:
      distance = (micros() - distCalcStartTime)*0.034/2;
      distCalcStartTime = 0;
      break;
  }
}

long calculateDistanceNow()
{
  startDistanceCalculation();
  long duration = pulseIn(SONAR_ECHO_PIN, HIGH);
  return (duration/2) / 29.1;
}

void moveServo(int angle)
{
   servo.write(angle); 
}


void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("smartcar", userName, passWord)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.subscribe("car/#");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  strTopic = String((char*)topic);
  if(strTopic == "car/move")
    {
    switch1 = String((char*)payload);
    if(switch1 == "W")
      {
         moveForward(2);
         Stop();
      }
    else if(switch1 == "A")
      {
         turnLeft(3);
      }
    else if(switch1 == "D")
      {
         turnRight(3);
      }
    else if(switch1 == "Z")
      {
        moveBackward(1);
        Stop();
      }
    else if(switch1 == "Q")
      {
        spinLeft(1);
        Stop();
      }
    else if(switch1 == "E")
      {
        spinRight(1);
        Stop();
      }
    else if(switch1 == "S")
      {
        Stop();
      }


    }

}


void setup()
{
  Serial.begin( 9600 );
  setup_wifi(); 
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

   //HC-SR04
   pinMode(SONAR_TRIG_PIN, OUTPUT);
   pinMode(SONAR_ECHO_PIN, INPUT);
   //interrupt when SONAR_ECHO_PIN changes; used instead of pulseIn(SONAR_ECHO_PIN, HIGH);
   attachInterrupt(SONAR_INTERRUPT, calculateDistance, CHANGE);

     
   //SG90
   servo.attach(SERVO_PIN);
   moveServo(90);

  pinMode( MOTOR_B_DIR, OUTPUT );
  pinMode( MOTOR_B_PWM, OUTPUT );
  digitalWrite( MOTOR_B_DIR, LOW );
  digitalWrite( MOTOR_B_PWM, LOW );
  pinMode( MOTOR_A_DIR, OUTPUT );
  pinMode( MOTOR_A_PWM, OUTPUT );
  digitalWrite( MOTOR_A_DIR, LOW );
  digitalWrite( MOTOR_A_PWM, LOW );


}

void loop()
{
  if (!client.connected()) {
    reconnect();
  }

   //Calculate distance
   Serial.print("Distance: ");
   Serial.println(distance);
   if(!distCalcStartTime) {
      startDistanceCalculation();
   }

   if(distance > 40) {
      //No obstacles detected - full speed ahead!
      moveForward(3);
   }
   else if(distance <= 40 && distance > 20) {
      //Upcoming obstacle detected - slow down
      moveForward(1);
      Stop();
   }
   else {
      //Detected obstacle way too close for comfort - backup and turn
      Stop();

      //Scan area
      moveServo(120);
      delay(250);
      long leftDist = calculateDistanceNow();
      moveServo(60);
      delay(250);
      long rightDist = calculateDistanceNow();
      moveServo(90);
    
      moveBackward(3);
      delay(500);

      if(leftDist >= rightDist) {
        spinLeft(2);
      }
      else {
        spinRight(2);
      }
      delay(350);

      Stop();
      calculateDistanceNow();

      delay(15);
   }

  client.loop();
}

