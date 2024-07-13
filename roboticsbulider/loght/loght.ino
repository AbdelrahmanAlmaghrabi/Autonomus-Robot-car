#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

#define IN1 6  // Left motor forward
#define IN2 7  // Left motor backward
#define IN3 9  // Right motor forward
#define IN4 8  // Right motor backward
#define ENA 5  // Enable pin for the left motor
#define ENB 10 // Enable pin for the right motor
#define LEFT_ENCODER_PIN 2  // Example pin number
#define RIGHT_ENCODER_PIN 3 // Example pin number

#define IR_SENSOR_PIN A0      // Analog pin connected to IR sensor


ros::NodeHandle nh;

std_msgs::Int32 left_encoder_msg;
ros::Publisher left_encoder_pub("left_encoder_count", &left_encoder_msg);

std_msgs::Int32 right_encoder_msg;
ros::Publisher right_encoder_pub("right_encoder_count", &right_encoder_msg);

void leftEncoderISR() {
  left_encoder_msg.data++;
  left_encoder_pub.publish(&left_encoder_msg);
}

void rightEncoderISR() {
  right_encoder_msg.data++;
  right_encoder_pub.publish(&right_encoder_msg);
}

void onTwist(const geometry_msgs::Twist &msg) {
  // Enable both motors initially
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);

  // Stop motors if IR sensor value is less than the threshold
  if (analogRead(IR_SENSOR_PIN) < 900) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  } else {
    // Move forward
    if (msg.linear.x > 0 || msg.angular.z == 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN3, HIGH);
    }
    // Turn left
    else if (msg.angular.z > 0) {
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
    }
    // Turn right
    else if (msg.angular.z < 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN4, HIGH);
    }
    // Move backward
    else if (msg.linear.x < 0) {
      digitalWrite(IN2, HIGH);
      digitalWrite(IN4, HIGH);
    } else {
      // Stop motors
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", onTwist);

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IR_SENSOR_PIN, INPUT); // Set IR sensor pin as input

  // Encoder pins as inputs
  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);

  // Attach interrupt for encoder pins
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, RISING);

  // Initially disable both motors
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);

  nh.initNode();
  nh.subscribe(sub);

  nh.advertise(left_encoder_pub);
  nh.advertise(right_encoder_pub);
}

void loop() {
  nh.spinOnce();

  // Print encoder counts to the serial monitor
  Serial.print("Left Encoder Count: ");
  Serial.println(left_encoder_msg.data);
  Serial.print("Right Encoder Count: ");
  Serial.println(right_encoder_msg.data);

  delay(100); // Delay for readability
}