#include <ESP32Encoder.h>
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

void motorTask(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    Serial.println("motor_A_FWD");
    motors(180, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    motors(0, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    Serial.println("motor_A_BWD");
    motors(-180, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    motors(0, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    Serial.println("motor_B_FWD");
    motors(0, 180);
    vTaskDelay(pdMS_TO_TICKS(500));
    motors(0, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    Serial.println("motor_B_BWD");
    motors(0, -180);
    vTaskDelay(pdMS_TO_TICKS(500));
    motors(0, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    Serial.println("motors_FWD");
    motors(80, 80);
    vTaskDelay(pdMS_TO_TICKS(1000));
    motors(0, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    Serial.println("motors_BWD");
    motors(-80, -80);
    vTaskDelay(pdMS_TO_TICKS(1000));
    motors(0, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
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

  // Create motor control RTOS task (runs independently)
  xTaskCreatePinnedToCore(
    motorTask,       // Task function
    "MotorTask",     // Name
    2048,            // Stack size in words
    NULL,            // Parameter
    1,               // Priority
    NULL,            // Task handle
    1                // Run on core 1
  );

  Serial.println("Motor control task started. Encoders ready.");
}

void loop() {
  long countA = encoderA.getCount();
  long countB = encoderB.getCount();
  Serial.print("A:");
  Serial.print(countA);
  Serial.print(",B:");
  Serial.println(countB);
  delay(50);
}