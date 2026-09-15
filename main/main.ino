#include <Arduino.h>
#include "src/geeWhiz/geeWhiz.h"
#include "src/transfer.h"

// ================== Pins ==================
int MOT_PIN = A0;   // motor angle sensor
int BAL_PIN = A1;   // ball position sensor


void calculate_stiction(void);

// ================== Setup ==================
void setup() {

  analogReadResolution(14);
  pinMode(A5, OUTPUT);   // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads
  Serial.begin(115200);
  delay(300);

  geeWhizBegin();                 
  // set_control_interval_ms(100); // 100 ms loop
  // setMotorVoltage(0.0f);
  calculate_stiction();

  Serial.println("geeWhiz Started");
}

// ================== Loop ==================
void loop() {
  
}
// motor voltage: 0.12200007 | theta: -0.13642
// motor voltage: 0.11400005 | theta: -0.21910
// motor voltage: 0.12100007 | theta: -0.28667
// motor voltage: 0.11200005 | theta: -0.35174

// ================== Control ISR ==================
void interval_control_code(void) {
  // ---- For Serial Plotter Scaling
  int maxy = 17000;
  int miny = 0;
  // ---- Read sensors ----
  int motor = analogRead(MOT_PIN);
  int ball  = analogRead(BAL_PIN);

  digitalWrite(A5,HIGH);   // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads
  Serial.print(maxy);
    Serial.print(",");
  Serial.print(miny);
    Serial.print(",");
  Serial.print(ball);
  Serial.print(",");
  Serial.println(angle_pot_to_rad(motor));
  digitalWrite(A5,LOW);   // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads
 
}

void calculate_stiction(void) {
  float initial_theta = angle_pot_to_rad(analogRead(MOT_PIN));
  float theta = initial_theta;
  float motor_voltage = 0.0;
  while (abs(theta - initial_theta) < 0.06) {
    setMotorVoltage(motor_voltage);
    delay(200);
    theta = angle_pot_to_rad(analogRead(MOT_PIN));

    Serial.print("motor voltage: ");
    Serial.print(motor_voltage, 8);
    Serial.print(" | theta: ");
    Serial.println(theta, 5);

    motor_voltage+=0.001;
  }
  setMotorVoltage(0.0);

  while (true) {}
}