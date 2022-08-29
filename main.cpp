#include <Arduino.h>
#include <MPU_sensor.h>
#include <motor.h>
#include <InverseKinematics.h>
#include "QuickPID.h"
#include <PID_v1.h>



struct IO_send_data {
  float roll;
  float pitch;
  float kp[3];  
  float ki[3];
  float kd[3];
};


struct pid_param
{
  float Kp[3];  
  float Ki[3];
  float Kd[3];
};

struct mpu_data
{
  float row;
  float pitch;
};

IO_send_data my_send_data;
QueueHandle_t roll_pitch_queue;
QueueHandle_t roll_pitch_uart_queue;
QueueHandle_t pid_param_queue;

void MPU_kalman(void * parameter){
  mpu_data my_mpu_data;
  MPU_sensor MPU;
  MPU.init();
  unsigned long prev_time = millis();
  for(;;){ // infinite loop
    MPU.update();
    my_mpu_data.row = MPU.get_roll_angle();
    my_mpu_data.pitch = MPU.get_pitch_angle();
    xQueueOverwrite(roll_pitch_queue, &my_mpu_data);
    if(millis() - prev_time > 1000) {
      xQueueOverwrite(roll_pitch_uart_queue, &my_mpu_data);
      prev_time = millis();
    }
    vTaskDelay(1);
    //Serial.print("ROLL = ");  Serial.print(MPU.get_roll_angle());  Serial.print("    ");
    //Serial.print("PITCH = "); Serial.print(MPU.get_pitch_angle()); Serial.println("    ");
  }
}

void uart_send(const IO_send_data* send_data) {
  Serial2.write((const char*)send_data, sizeof(IO_send_data));
}
float my_map(float x, float in_min, float in_max, float out_min, float out_max) {
    const float dividend = out_max - out_min;
    const float divisor = in_max - in_min;
    const float delta = x - in_min;
    if(divisor == 0){
        //log_e("Invalid map input range, min == max");
        return -1; //AVR returns -1, SAM returns 0
    }
    return (delta * dividend + (divisor / 2)) / divisor + out_min;
}

void send_via_uart(void * parameter){
  mpu_data my_mpu_data;
  pid_param my_pid_param;
  for(;;){ // infinite loop
    xQueueReceive(roll_pitch_uart_queue, &my_mpu_data, 0);
    xQueueReceive(pid_param_queue, &my_pid_param, 0);
    my_send_data.pitch = my_mpu_data.pitch;
    my_send_data.roll  = my_mpu_data.row;

    my_send_data.kp[0] = my_pid_param.Kp[0];
    my_send_data.kp[1] = my_pid_param.Kp[1];
    my_send_data.kp[2] = my_pid_param.Kp[2];

    my_send_data.ki[0] = my_pid_param.Ki[0];
    my_send_data.ki[1] = my_pid_param.Ki[1];
    my_send_data.ki[2] = my_pid_param.Ki[2];

    my_send_data.kd[0] = my_pid_param.Kd[0];
    my_send_data.kd[1] = my_pid_param.Kd[1];
    my_send_data.kd[2] = my_pid_param.Kd[2];

    //Serial.print("ROLL = ");  Serial.print(my_send_data.pitch);  Serial.print("    ");
    //Serial.print("PITCH = "); Serial.print(my_send_data.roll); Serial.println("    ");
    uart_send(&my_send_data);
    vTaskDelay(1000);
  }
}


void inverse_pid(void * parameter){

  double Kp[3], Ki[3], Kd[3];
  pid_param my_pid_param;

  my_pid_param.Kp[0] = 6;
  my_pid_param.Kp[1] = 6;
  my_pid_param.Kp[2] = 6;

  my_pid_param.Ki[0] = 1.5;
  my_pid_param.Ki[1] = 1.5;
  my_pid_param.Ki[2] = 1.5;

  my_pid_param.Kd[0] = 0.1;
  my_pid_param.Kd[1] = 0.1;
  my_pid_param.Kd[2] = 0.1;

  xQueueOverwrite(pid_param_queue, &my_pid_param);
  motor my_motor;
  mpu_data my_mpu_data;
  InverseKinematics my_inverse(0, 50, 50, 50);
  my_motor.init();
  my_motor.move(1, 73, true);
  my_motor.move(2, 80, true);
  my_motor.move(3, 72, true);
  delay(10000);
  my_motor.move(1, 0, true);
  my_motor.move(2, 0, true);
  my_motor.move(3, 0, true);

  my_motor.MOTOR1.encoder.clearCount();
  my_motor.MOTOR2.encoder.clearCount();
  my_motor.MOTOR3.encoder.clearCount();

  double Setpoint[3], Input[3], Output[3];
  
  PID myPID_M1(&Input[0], &Output[0], &Setpoint[0], my_pid_param.Kp[0], my_pid_param.Ki[0], my_pid_param.Kd[0], DIRECT);
  PID myPID_M2(&Input[1], &Output[1], &Setpoint[1], my_pid_param.Kp[1], my_pid_param.Ki[1], my_pid_param.Kd[1], DIRECT);
  PID myPID_M3(&Input[2], &Output[2], &Setpoint[2], my_pid_param.Kp[2], my_pid_param.Ki[2], my_pid_param.Kd[2], DIRECT);

  myPID_M1.SetMode(AUTOMATIC);
  myPID_M2.SetMode(AUTOMATIC);
  myPID_M3.SetMode(AUTOMATIC);

  myPID_M1.SetOutputLimits(-255, 255);
  myPID_M2.SetOutputLimits(-255, 255);
  myPID_M3.SetOutputLimits(-255, 255);

  myPID_M1.SetSampleTime(1);
  myPID_M2.SetSampleTime(1);
  myPID_M3.SetSampleTime(1);
  
  for(;;){ // infinite loop
    xQueueReceive(roll_pitch_queue, &my_mpu_data, portMAX_DELAY);
   // my_inverse.update(my_mpu_data.pitch, my_mpu_data.row);
    my_inverse.update(my_mpu_data.row, my_mpu_data.pitch);

    //set input
    Setpoint[0] = my_map(my_inverse.get_L_0(), -5, 90, 0, 1567);
    Setpoint[1] = my_map(my_inverse.get_L_1(), -5, 90, 0, 1567);
    Setpoint[2] = my_map(my_inverse.get_L_2(), -5, 90, 0, 1567);

    Input[0] = my_motor.get_encoder_value(1);
    Input[1] = my_motor.get_encoder_value(2);
    Input[2] = my_motor.get_encoder_value(3);
    //compute
    myPID_M1.Compute();
    myPID_M2.Compute();
    myPID_M3.Compute();
    //update output 1 step econcoder = 0.0873 degree
    if(Output[0] < 0) {
      my_motor.move(1, abs(Output[0]), true);
    } else {
       my_motor.move(1, Output[0], false);
    }

    if(Output[1] < 0) {
      my_motor.move(2, abs(Output[1]), true);
    } else {
       my_motor.move(2, Output[1], false);
    }
    if(Output[2] < 0) {
      my_motor.move(3, abs(Output[2]), true);
    } else {
       my_motor.move(3, Output[2], false);
    }
    //Serial.print("SP3 = ");  Serial.println(Setpoint[2]);  
    //Serial.print("IN3 = ");  Serial.println(Input[2]); 
    //Serial.print("OUT3 = ");  Serial.println(Output[2]);  
    //Serial.print("EN1 = "); Serial.println(my_motor.get_encoder_value(2)); 
    //Serial.print("EN2 = "); Serial.println(my_motor.get_encoder_value(3)); 
    vTaskDelay(1);
  }
}

void setup() {

  roll_pitch_queue = xQueueCreate( 1, sizeof(mpu_data));
  roll_pitch_uart_queue =  xQueueCreate( 1, sizeof(mpu_data));
  pid_param_queue =  xQueueCreate( 1, sizeof(pid_param));
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(1000000);
  xTaskCreate(
    MPU_kalman,    // Function that should be called
    "MPU kalman filter aplied",   // Name of the task (for debugging)
    10000,            // Stack size (bytes)
    NULL,            // Parameter to pass
    1,               // Task priority
    NULL             // Task handle
  );

  xTaskCreate(
    inverse_pid,    // Function that should be called
    "inversed and pid processing",   // Name of the task (for debugging)
    10000,            // Stack size (bytes)
    NULL,            // Parameter to pass
    1,               // Task priority
    NULL             // Task handle
  );

  xTaskCreate(
    send_via_uart,    // Function that should be called
    "send data to display esp32",   // Name of the task (for debugging)
    10000,            // Stack size (bytes)
    NULL,            // Parameter to pass
    1,               // Task priority
    NULL             // Task handle
  );
}

void loop() {
  delay(1);
  // put your main code here, to run repeatedly:
  //uart_receive(&my_rev_data);
  
  //Serial.println(my_rev_data.kd[2]);
  //my_send_data.pitch = MPU.get_pitch_angle();
  //my_send_data.roll  = MPU.get_roll_angle();
}