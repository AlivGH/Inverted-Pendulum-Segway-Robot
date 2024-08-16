#include <Wire.h>
#include <MPU9250.h>
#include <IRremote.h>
#include <SimpleKalmanFilter.h>
#include <PID_v1.h>

MPU9250 mpu;

//SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);
const long SERIAL_REFRESH_TIME = 10;
long refresh_time;

const int motor1PWM = 6;
const int motor1IN1 = 8;
const int motor1IN2 = 7;
const int motor2PWM = 11;
const int motor2IN1 = 9;
const int motor2IN2 = 10;


double MOTORSLACK_A = 50;    // 18                // Compensate for motor slack range (low PWM values which result in no motor engagement)
double MOTORSLACK_B = 50;    //3                 // Compensate for motor slack range (low PWM values which result in no motor engagement) l298
double A_speed, B_speed;

// Low-pass filter parameters
float lowPassAlpha = 0.1;  // Low-pass smoothing factor
float lowPassFilteredValue = 0.0;  // Low-pass filtered output

// High-pass filter parameters
float highPassAlpha = 0.1;  // High-pass smoothing factor
float highPassFilteredValue = 0.0;  // High-pass filtered output

float angle = 0.0;
double Kp = 50.0; 
double Ki = 0.1; 
double Kd = 0.6 ; 
double setpoint = 88.0;
double error, lastError = 0.0;
double integral = 0.0;
double input, output;

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if (!mpu.setup(0x68)) {  // change to your own address
      while (1) {
        Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
        
      }
  }
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1IN1, OUTPUT);
  pinMode(motor1IN2, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2IN1, OUTPUT);
  pinMode(motor2IN2, OUTPUT);

  // Set the PID sample time (in milliseconds)
  //myPID.SetSampleTime(100);
  myPID.SetOutputLimits(-200, 200);  // Assuming output is in the range [0, 255]
  myPID.SetMode(AUTOMATIC);  // Enable PID control

}

void loop() {
  if (mpu.update()) {
      static uint32_t prev_ms = millis();
      if (millis() > prev_ms + 25) {
        prev_ms = millis();
      }
  }
    // read a reference value from A0 and map it from 0 to 100

//MEASURE TIME IN LOOP
  /*startTime = millis();

  // Your code here (the portion you want to measure)

  // Record the end time
  endTime = millis();

  // Calculate and print the runtime
  unsigned long runtime = endTime - startTime;
  Serial.print("Runtime: ");
  Serial.print(runtime);
  Serial.println(" milliseconds");*/




  input = mpu.getRoll();

  // Low-pass filter
  lowPassFilteredValue = lowPassAlpha * input + (1 - lowPassAlpha) * lowPassFilteredValue;

  // High-pass filter
  highPassFilteredValue = highPassAlpha * (input - lowPassFilteredValue) + (1 - highPassAlpha) * highPassFilteredValue;

  // Band-pass filtered value
  float input = lowPassFilteredValue + highPassFilteredValue;



//  PID Control Portion
  myPID.Compute();
  error = setpoint - input ; 
  integral += error;

  double motorSpeed = Kp * error + Ki * integral + Kd * (error - lastError);


  /* double* result =compensate_slack(motorSpeed) ;
       Serial.println(result[1]);*/

    A_speed = compensate_slack(motorSpeed, MOTORSLACK_A);
    B_speed = compensate_slack(motorSpeed, MOTORSLACK_B);

//////////////*****************************Debug section******************************////////////

/*
Serial.print("P:  " );
Serial.print( Kp * error);
Serial.print(",  " );
Serial.print( Ki * integral);
Serial.print(",  " );
Serial.print(Kd * (error - lastError));
Serial.print(",  -" );
Serial.print(motorSpeed);
Serial.print("--   ");
//Serial.println(input);
Serial.println(error);
/*
Serial.print(A_speed);
Serial.print("--   ");
Serial.println(B_speed);
*/


Serial.println(output);
//////////////*****************************Send PWM to Motors******************************////////////
 if (output >= 0) {
    // Motor 1 forward

    analogWrite(motor1PWM, A_speed);
    digitalWrite(motor1IN1, HIGH);
    digitalWrite(motor1IN2, LOW);



    // Motor 2 forward
    analogWrite(motor2PWM, B_speed);
    digitalWrite(motor2IN1, HIGH);
    digitalWrite(motor2IN2, LOW);

/*Serial.print("plus  :");
    Serial.print(motorSpeed);
Serial.print("   ");
Serial.println(Roll);*/



  } else {
    // Motor 1 backward

   
    analogWrite(motor1PWM, -A_speed);
    digitalWrite(motor1IN1, LOW);
    digitalWrite(motor1IN2, HIGH);

    // Motor 2 backward
    analogWrite(motor2PWM, -B_speed);
    digitalWrite(motor2IN1, LOW);
    digitalWrite(motor2IN2, HIGH);

/*
Serial.print("minus  :");
    Serial.print(motorSpeed);
Serial.print("   ");
Serial.println(Roll);*/
  }

  ///**********************************************************************************************************************
 lastError = error;
 
}





double compensate_slack(double Output, double MOTORSLACK)
  {
   // Compensate for DC motor non-linear "dead" zone around 0 where small values don't result in movement
   //yOutput is for left,right control
double speed;
   if (Output >= 0) {
    speed = Output + MOTORSLACK ;
    speed = max(0, speed);
  }
   else 
   { speed = Output - MOTORSLACK ;
   speed = min(0,speed);
   }
   
   speed = constrain(speed, -255, 255); 


  return speed;
}

