#include <ECE3.h>

int blackSum;
int prevBlackSum;
int discSum;
const int LED_RF = 41;
uint16_t sensorValues[8];
int white_array[8];
int sensorData[8];
int weights[8] = {-15, -14, -12, -8, 8, 12, 14, 15};
int totalSum;
int prevTotalSum;
int leftWheelSpeed;
int rightWheelSpeed;
double kp = 0.0025;
double kd =0.0008;
double correction;
int currentLeftSpeed =0;
int currentRightSpeed = 0;
int turnAroundCounter = 0;
int firstBreakCounter = 0;
int startingPoint =0;
int straightAwayCounter =0;

const int left_nslp_pin = 31;
const int left_direction =29;
const int right_nslp_pin = 11;
const int right_direction =30;
const int left_pwm_pin =40;
const int right_pwm_pin = 39;

void ChangeWheelSpeeds(int initialLeftSpd,int finalLeftSpd,int initialRightSpd,int
finalRightSpd) {
  /*
  * This function changes the car speed gradually (in about 30 ms) from initial
  * speed to final speed. This non-instantaneous speed change reduces the load
  * on the plastic geartrain, and reduces the failure rate of the motors.
  */
  int diffLeft = finalLeftSpd-initialLeftSpd;
  int diffRight = finalRightSpd-initialRightSpd;
  int stepIncrement = 20;
  int numStepsLeft = abs(diffLeft)/stepIncrement;
  int numStepsRight = abs(diffRight)/stepIncrement;
  int numSteps = max(numStepsLeft,numStepsRight);
  int pwmLeftVal = initialLeftSpd; // initialize left wheel speed
  int pwmRightVal = initialRightSpd; // initialize right wheel speed
  int deltaLeft = (diffLeft)/numSteps; // left in(de)crement
  int deltaRight = (diffRight)/numSteps; // right in(de)crement
  for(int k=0;k<numSteps;k++) {
  pwmLeftVal = pwmLeftVal + deltaLeft;
  pwmRightVal = pwmRightVal + deltaRight;
  analogWrite(left_pwm_pin,pwmLeftVal);
  analogWrite(right_pwm_pin,pwmRightVal);
  delay(30);
  } // end for int k
  analogWrite(left_pwm_pin,finalLeftSpd);
  analogWrite(right_pwm_pin,finalRightSpd);
}

void setup()
{
  ECE3_Init();
  //Setting up Pins
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);
  pinMode(right_direction,OUTPUT);
  pinMode(left_direction,OUTPUT);

  //init pins
  digitalWrite(left_direction, LOW);
  digitalWrite(right_direction,LOW);
  digitalWrite(left_nslp_pin, HIGH);
  digitalWrite(right_nslp_pin,HIGH);

  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  //delay(2000);

  //Delay to Get Setup
  
  //set up arrays
  for (int i; i < 8; i++){
    white_array[i] = 0;
    weights[i] = 0;
  }
  rightWheelSpeed =0;
  leftWheelSpeed =0;
  totalSum =0;
  //Serial.println();
  //Serial.println();
  //Serial.println();
  //Serial.println();
  for(int j =0; j < 10; j++){
    ECE3_read_IR(sensorValues);
    for (int i =0; i < 8; i++){
      white_array[i] += sensorValues[i];
      //Serial.print(sensorValues[i]);
    //Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
    }
    //Serial.println();
    delay(200); 
  }
  
  //Serial.println();
  for (int i = 0; i < 8; i++){
    white_array[i] = white_array[i]/10;
    //Serial.print(white_array[i]); 
    //Serial.print('\t');
  }
  //Serial.println();
  //Serial.println();
  //Serial.println();
  //Serial.println();
  
  digitalWrite(LED_RF,HIGH);
  delay(4000); //10000
  digitalWrite(LED_RF,LOW);
  //leftWheelSpeed = 50;
  //rightWheelSpeed = 50;
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
  prevTotalSum = 0;
  totalSum =0;
  blackSum = 0;
  prevBlackSum =0;
}

int average(){
  int getL = getEncoderCount_left();
  int getR = getEncoderCount_right();
  return ((getL +getR)/2);
}

void loop()
{
  // read raw sensor values
  ECE3_read_IR(sensorValues);

  currentRightSpeed = 100;
  currentLeftSpeed = 100;
 
  if (average() < 75){
    if (startingPoint ==0){
      ChangeWheelSpeeds(currentLeftSpeed, 35, currentRightSpeed, 35);
      startingPoint++;
    }
    currentLeftSpeed = 35;
    currentRightSpeed =35;
    kp = 0.001;
    kd = 0.020;
  }
  //curve
  if (average() > 75 && average() < 1800){
    kp = 0.003;
    kd =0.001;
    if (startingPoint ==1){
      ChangeWheelSpeeds(currentLeftSpeed, 75, currentRightSpeed, 75);
      startingPoint++;
    }
    currentLeftSpeed = 75;
    currentRightSpeed = 75;
  }
  //Straight Away
  if (average() >= 1800 && average() < 3400){
    //digitalWrite(LED_RF,HIGH);
    if (straightAwayCounter == 0){
        ChangeWheelSpeeds(currentLeftSpeed, 250 , currentRightSpeed, 250);
        straightAwayCounter++;
    }
    currentLeftSpeed =250;
    currentRightSpeed =250;
    kp = 0.004;
    kd =0.045;
  }
  
  //First Break Discontinuity
   if (average() >= 3400 && average() <= 3900){
    kp = 0.0025;
    kd = 0.0008;
    if (firstBreakCounter == 0){
      ChangeWheelSpeeds(currentLeftSpeed, 40, currentRightSpeed, 40);
      firstBreakCounter++;
    }
    currentLeftSpeed =40;
    currentRightSpeed =40;
  }
 if (average() >= 3600 && average() <= 4000){
    //digitalWrite(LED_RF,HIGH);
    weights[0] =0;
    //weights[1] = 0;
    //weights[6] = 0;
    weights[7] =0;
    kd = 0.015;
  }
  if (average() >= 3900 && average() < 5800 ){
    //digitalWrite(LED_RF,LOW);
    if (firstBreakCounter == 1){
      ChangeWheelSpeeds(currentLeftSpeed, 175, currentRightSpeed, 175);
      firstBreakCounter++;
    }
    currentLeftSpeed =175;
    currentRightSpeed =175;
    kp = 0.004;
    kd =0.040;
  }
  if (average() > 4400 && average() < 4500){
    weights[0] =-15;
    weights[7] = 15; 
  }
  
  //BLACK BARS WAY BACK
  if (average() > 5900 && average() < 6000){
    
    weights[0] = 0;
    weights[7] = 0; 
  }
  
  if (average() >= 6000 && average() <= 6500){
    digitalWrite(LED_RF,LOW);
    if (firstBreakCounter == 2){
      weights[0] =-15;
      weights[7] = 15;
      ChangeWheelSpeeds(currentLeftSpeed, 110, currentRightSpeed, 110);
      firstBreakCounter++;
    }
    currentLeftSpeed =110;
    currentRightSpeed =110;
    kp = 0.0025;
    kd = 0.0008;
  }
   // Second Straight Away
  if (average() >= 6500 && average() <= 7650){
    digitalWrite(LED_RF,HIGH);
    if (straightAwayCounter == 2){
        ChangeWheelSpeeds(currentLeftSpeed, 240, currentRightSpeed, 240);
        straightAwayCounter++;
    }
    currentLeftSpeed =240;
    currentRightSpeed =240;
    kp = 0.004;
    kd =0.040;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
  }

  if (average() > 7650){
    kp = 0.003;
    kd =0.001;
    if (startingPoint ==2 ){
      ChangeWheelSpeeds(currentLeftSpeed, 70, currentRightSpeed, 70);
      
      startingPoint++;
    }
    currentLeftSpeed = 70;
    currentRightSpeed = 70;
  }
  
  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
  // 2500 means minimum reflectance
  prevTotalSum = totalSum;
  totalSum =0;
  prevBlackSum = blackSum;
  blackSum =0;
  
  for (unsigned char i = 0; i < 8; i++)
  {
    //Checking for all black condition to turnaround
    if(i !=0 || i != 7){
      blackSum += sensorValues[i];
    }
    if (i!= 0 || i!=1 || i!= 6 || i!=7){
      discSum += sensorValues[i];
    }
    
    //Subtract Min
    sensorData[i] = sensorValues[i];
    sensorData[i] -= white_array[i];
    
    //Serial.print(sensorData[i]);
    
    //Normalize

    //Weighting Scheme
    sensorData[i] *= weights[i];
    totalSum += sensorData[i];
    
    /*
    //Test Sensor Values
    Serial.print(sensorValues[i]);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
    Serial.println();
    */
  }
  
  //TURN AROUND ALGORITHUM

  //END
  if (prevBlackSum >= 2500*6 && blackSum >= 2500*6 && turnAroundCounter == 1 && average() > 6200){ //only using 6 sensors
    ChangeWheelSpeeds(currentLeftSpeed, 0, currentRightSpeed, 0);
    digitalWrite(left_nslp_pin, LOW);
    digitalWrite(right_nslp_pin,LOW);
    //digitalWrite(LED_RF,HIGH);
    delay(10);
  }
  
 if (average() >= 4510 && average() < 4550){
    if (turnAroundCounter == 0){ //only using 6 sensors
      ChangeWheelSpeeds(currentLeftSpeed, 0, currentRightSpeed, 0);
      digitalWrite(left_nslp_pin, LOW);
      digitalWrite(right_nslp_pin,LOW);
      //digitalWrite(LED_RF,HIGH);
      delay(10);
      digitalWrite(left_nslp_pin, HIGH);
      digitalWrite(right_nslp_pin,HIGH);
      digitalWrite(right_direction,HIGH);
      digitalWrite(left_direction,LOW);
      //Set speeds
      ChangeWheelSpeeds(0, 150, 0, 150);
      delay(225);
      digitalWrite(right_direction,LOW);
      digitalWrite(left_direction,LOW);
      turnAroundCounter++;
      ChangeWheelSpeeds(150,currentLeftSpeed, 150, currentRightSpeed);
    }
  }
  
  correction = kp* totalSum + kd*(totalSum- prevTotalSum);

  /*
  // NOTE: (POSSIBLE SOLUTION) IF CORRECTION IS VERY LARGE --> bump up kd and kp values
  // As a debug, add array of all values
  if (correction > 11 || correction < -11){
    currentRightSpeed = 20;
    currentLeftSpeed = 20;
    kp = 0.0025;
    kd = 0.0032;
  }
  if (correction < 0){
    //ChangeWheelSpeeds(currentLeftSpeed, currentLeftSpeed + (correction*-1), currentRightSpeed, currentRightSpeed);
    currentLeftSpeed += correction*-1;
    analogWrite(left_pwm_pin, currentLeftSpeed);
  }
  if (correction > 0){
    //ChangeWheelSpeeds(currentLeftSpeed, currentLeftSpeed , currentRightSpeed, currentRightSpeed + correction);
    currentRightSpeed += correction; 
    analogWrite(right_pwm_pin, currentRightSpeed);
  }
  */
  
  //Correction
  currentRightSpeed += correction; 
  analogWrite(right_pwm_pin, currentRightSpeed);
  currentLeftSpeed -= correction; 
  analogWrite(left_pwm_pin, currentLeftSpeed);
}
