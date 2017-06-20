
#include <BNO055.h>
#include "BNO055.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#define A 0X28  //I2C address selection pin LOW
#define B 0x29  //                          HIGH
 Servo myservo[4];
 SoftwareSerial esp8266(2, 3); //RX,TX
float pid_p_gain_roll = 1.7;               //Gain setting for the roll P-controller
float pid_i_gain_roll =0;              //Gain setting for the roll I-controllerint pid_d_gain_roll =0;              //Gain setting for the roll D-controller
int pid_d_gain_roll = 12; //Gain setting for the pitch D-controller.
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch =1.7;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = 0;  //Gain setting for the pitch I-controller.
int pid_d_gain_pitch = 12;
; //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

int pid_p_gain_yaw = 0;                //Gain setting for the pitch P-controller. //4.0
int pid_i_gain_yaw = 0;               //Gain setting for the pitch I-controller. //0.02
int pid_d_gain_yaw = 0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

int gyro_pitch, gyro_roll, gyro_yaw;
int gyro_axis_cal[4];
int pid_error_temp;
int pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
int pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
int pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
int angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
boolean gyro_angles_set;

BNO055 mySensor(B);
int throttle=1100;
 char   c=' ';
 String response = "";
  String num="";

 float rolloffset;
 float pitchoffset;
void  wifi_setup();
 
void setup(){   esp8266.begin(9600);
      Serial.begin(9600);
myservo[0].attach(4);
myservo[1].attach(5);
myservo[2].attach(6);
myservo[3].attach(7);
//
//delay(2000);
// for(int i=0;i<4;i++){
//  myservo[i].writeMicroseconds(2000);
//   Serial.print("connect esc\r\n");
// }
//  delay(8000);
//   for(int i=0;i<4;i++)
//  myservo[i].writeMicroseconds(1000);

  
    Wire.begin();  

    mySensor.init();
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
    pid_pitch_setpoint=0;
    pid_roll_setpoint=0;
            myservo[0].writeMicroseconds(1000);
            myservo[1].writeMicroseconds(1000);
            myservo[3].writeMicroseconds(1000);
            myservo[2].writeMicroseconds(1000);
            throttle=1000;
             
             delay(500);    
            myservo[0].writeMicroseconds(1200);
            myservo[1].writeMicroseconds(1200);
            myservo[3].writeMicroseconds(1200);
            myservo[2].writeMicroseconds(1200);

  mySensor.readEul();
 rolloffset=-mySensor.euler.y;
 pitchoffset=-mySensor.euler.z;
 wifi_setup();
 
}



 
int del=10;
 String readString="";
void loop(){
 if (esp8266.available()) {
  unsigned long start = micros();
// Call to your function
receiver();
// Compute the time it took
unsigned long end = micros();
unsigned long delta = end - start;
Serial.println(delta);

 }
 

//Serial.print("Roll:");
//Serial.print(  gyro_roll_input);
//Serial.print("        Pitch:");
//Serial.println(  gyro_pitch_input);

    mySensor.readEul();
     gyro_pitch_input=mySensor.euler.z+pitchoffset;
      gyro_roll_input=mySensor.euler.y+ rolloffset;

 
    calculate_pid();
      
    double esc_0 = throttle - pid_output_pitch - pid_output_roll;   // + pid_output_roll - pid_output_yaw; Calculate the pulse for esc 1 (front-right - CCW)
    double esc_3 = throttle + pid_output_pitch - pid_output_roll; //+ pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    double esc_2 = throttle + pid_output_pitch + pid_output_roll; // - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    double esc_1= throttle - pid_output_pitch  + pid_output_roll;// - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
       
            myservo[0].writeMicroseconds(esc_0);
            myservo[1].writeMicroseconds(esc_1);
            myservo[3].writeMicroseconds(esc_3);
            myservo[2].writeMicroseconds(esc_2); 
             
//    receiver();
//    if( num.toInt()>=1000 && num.toInt()<=2000){
//      int no=num.toInt();
//      Serial.println(response[0]);
//        Serial.println(n); 
//        if(response[0]=='t'){
//          throttle=no;
//           }
//        }


//Serial.print(esc_2);
//Serial.print("         ");
//Serial.println(esc_0);
//manual calibration 


//Serial.print("Motor 1 :");  
//Serial.print( esc_0);
//Serial.print("    2 :");  
//Serial.print( esc_1);
//Serial.print("    3 :");  
//Serial.print( esc_2);
//Serial.print("    4 :");  
//Serial.println( esc_3);
  }

void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
//  if((pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error))/abs(pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error)) != (pid_error_temp/abs(pid_error_temp)))
//  {
//
//     pid_d_gain_pitch =105;
//     pid_d_gain_roll =105;
//    pid_p_gain_roll=4;
//       pid_p_gain_pitch=4;
//        pid_max_roll = 500;                    //Maximum output of the PID-controller (+/-)
//      pid_max_pitch = 500;                    //Maximum output of the PID-controller (+/-)
//   
//  }
//  else if((pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error))/abs(pid_d_gain_pitch  * (pid_error_temp - pid_last_pitch_d_error)) != (pid_error_temp/abs(pid_error_temp)))
//  {
//     pid_p_gain_roll=4;
//       pid_p_gain_pitch=4;
//     pid_d_gain_pitch =95;
//     pid_d_gain_roll =95;
//     pid_max_roll = 500;                    //Maximum output of the PID-controller (+/-)
//      pid_max_pitch = 500;                    //Maximum output of the PID-controller (+/-)
//
//
//  }
//  else
//  {
//    pid_d_gain_pitch =7;
//     pid_d_gain_roll =7;
//        pid_p_gain_roll=5;
//       pid_p_gain_pitch=5;
//       if(abs(gyro_roll_input)<2 && abs(gyro_pitch_input)<2){
//         pid_p_gain_roll=55;
//       pid_p_gain_pitch=55;
//          pid_max_roll = 150;                    //Maximum output of the PID-controller (+/-)
//      pid_max_pitch = 150; 
//       } 
//       else
//       {
//         pid_max_roll = 205;                    //Maximum output of the PID-controller (+/-)
//      pid_max_pitch = 205;                    //Maximum output of the PID-controller (+/-)
//       }
//       
//  }
  
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;
  
          
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
  

            
//  Serial.println(pid_output_roll);
//  
//  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
//  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
//  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
//
//  pid_last_yaw_d_error = pid_error_temp;
}



String sendData(String command, const int timeout)
{
    String response = "";
    
    esp8266.print(command); // send the read character to the esp8266
    
    long int time = millis();
    
    while( (time+timeout) > millis())
    {
      while(esp8266.available())
      {
      
        // The esp has data so display its output to the serial window 
        char c = esp8266.read(); // read the next character.
        response+=c;
      }  
    }
 
     Serial.print(response);
    return response;
}



void receiver(){
  
    c=' ';
   response = "";
  num="";
  
  if (esp8266.available() && c!='\0' )     {
     c = esp8266.read();
     if(c=='h')
     for(int u=0 ; u>-1; u++) {
      for(int o=0;o<4;o++){
myservo[o].writeMicroseconds(1000);
while (esp8266.available()  )     {
     c = esp8266.read();
  
  if(c=='s')
      u=-2;
    
     }
   
  }

}
     
      
     if(c=='p' || c=='t' || c=='y' || c=='r')
     {
        response = c;
         delay(del);
        c = esp8266.read();
         response += c;
        num=num+c;
         delay(del);
        c = esp8266.read();
     
         response += c;
     num=num+c;    
         delay(del);
        c = esp8266.read();
        
         response += c;
         num=num+c;
         delay(del);
        c = esp8266.read();
        
        response += c;
        num=num+c;
        c='\0';
     num=num+c;
     }
    
   
  }
      if( num.toInt()>=1000 && num.toInt()<=2000){
      int n=num.toInt();
      Serial.print(response[0]);
        Serial.println(n); 
        if(response[0]=='t'){
          throttle=n;
           }
        }
  }


  void wifi_setup(){
     esp8266.begin(9600);//this to enter 
 
Serial.print("setting up server\r\n");

sendData("AT+CWMODE=2\r\n",2000);
sendData("AT+CWSAP=\"Dronecontrol\",,5,0\r\n",2000);
sendData("AT+CWDHCP=0,0\r\n",2000);

sendData("AT+CIPMUX=1\r\n",2000);
sendData("AT+CIPSERVER=1,21\r\n",2000);
sendData("AT+CIPAP=\"192.168.5.152\"\r\n",2000); // this 
  //sendData("AT+CIPSTART=\"TCP\",\"192.168.5.203\",21\r\n",2000);
  }
  

