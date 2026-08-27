#include <ESP32Encoder.h>
#include <ArduPID.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

ESP32Encoder encoderA;
ESP32Encoder encoderB;

// L293D motor-driver pins, matching test_motors.ino.
const int MOTOR_A_DIRECTION_PIN = 27;
const int MOTOR_A_PWM_PIN = 14;
const int MOTOR_B_DIRECTION_PIN = 13;
const int MOTOR_B_PWM_PIN = 12;

// Encoder pins for Motor A
const int MOT_A_ENCODER_A = 23;
const int MOT_A_ENCODER_B = 22;
// Encoder pins for Motor B
const int MOT_B_ENCODER_A = 19;
const int MOT_B_ENCODER_B = 21;

// ArduPID requires doubles for all inputs/outputs. Setpoints/inputs are signed encoder ticks per control interval.
double setpointA = 0, inputA = 0, outputA = 0;
double setpointB = 0, inputB = 0, outputB = 0;

ArduPID controllerA;
ArduPID controllerB;

double p = 0.4;
double i = 0.001;
double d = 0.05;

void motors(int speedA, int speedB) {
  if (speedA >= 0) {
    analogWrite(MOTOR_A_PWM_PIN, speedA);
    digitalWrite(MOTOR_A_DIRECTION_PIN, LOW);
  } else {
    analogWrite(MOTOR_A_PWM_PIN, 256 + speedA);
    digitalWrite(MOTOR_A_DIRECTION_PIN, HIGH);
  }

  if (speedB >= 0) {
    analogWrite(MOTOR_B_PWM_PIN, speedB);
    digitalWrite(MOTOR_B_DIRECTION_PIN, LOW);
  } else {
    analogWrite(MOTOR_B_PWM_PIN, 256 + speedB);
    digitalWrite(MOTOR_B_DIRECTION_PIN, HIGH);
  }
}

void controlTask(void *pvParameters) {
  (void) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(40);

  long prevCountA = 0;
  long prevCountB = 0;

  for (;;) {
    long currCountA = (long)encoderA.getCount();
    inputA = (double)(currCountA - prevCountA);
    prevCountA = currCountA;
    controllerA.compute();

    long currCountB = (long)encoderB.getCount();
    inputB = (double)(currCountB - prevCountB);
    prevCountB = currCountB;
    controllerB.compute();

    motors((int)outputA, (int)outputB);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_A_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_A_PWM_PIN, OUTPUT);
  pinMode(MOTOR_B_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_B_PWM_PIN, OUTPUT);
  motors(0, 0);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoderA.attachFullQuad(MOT_A_ENCODER_A, MOT_A_ENCODER_B);
  encoderA.clearCount();
  encoderB.attachFullQuad(MOT_B_ENCODER_A, MOT_B_ENCODER_B);
  encoderB.clearCount();

  // ArduPID Initialization
  controllerA.begin(&inputA, &outputA, &setpointA, p, i, d);
  controllerB.begin(&inputB, &outputB, &setpointB, p, i, d);

  controllerA.setOutputLimits(-255, 255);
  controllerB.setOutputLimits(-255, 255);

  controllerA.start();
  controllerB.start();

  // Create closed-loop velocity control RTOS task (runs independently)
  xTaskCreatePinnedToCore(
    controlTask,     // Task function
    "PID_Task",      // Name
    4096,            // Stack size in words
    NULL,            // Parameter
    1,               // Priority
    NULL,            // Task handle
    1                // Run on core 1
  );

  Serial.println("Closed-loop velocity control task started. Encoders ready.");
}

void loop() {
  // Serial command parser for velocity setpoints (ticks per 40 ms control interval)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("A:")) {
      setpointA = cmd.substring(2).toDouble();
      Serial.print("Setpoint A updated: "); Serial.println(setpointA);
    } else if (cmd.startsWith("B:")) {
      setpointB = cmd.substring(2).toDouble();
      Serial.print("Setpoint B updated: "); Serial.println(setpointB);
    }
  }

  // Use Serial Plotter (Ctrl+Shift+L) to visualize
  Serial.print("setP_A:");  Serial.print(setpointA); Serial.print(",");
  Serial.print("input_A:"); Serial.print(inputA);    Serial.print(",");
  Serial.print("output_A:"); Serial.print(outputA);  Serial.print(",");
  Serial.print("setP_B:");  Serial.print(setpointB); Serial.print(",");
  Serial.print("input_B:"); Serial.print(inputB);    Serial.print(",");
  Serial.print("output_B:"); Serial.println(outputB);

  delay(100);
}