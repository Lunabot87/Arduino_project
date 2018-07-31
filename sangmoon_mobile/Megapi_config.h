#ifndef   MEGAPI_CONFIG_H_
#define   MEGAPI_CONFIG_H_

#include "TimerOne.h"

#define CH_L    2
#define CH_L_K  A1
#define CH_R    19
#define CH_R_K  38
#define DIR_1_L A5
#define DIR_2_L A4
#define DIR_1_R 36
#define DIR_2_R 37
#define PWM_L   5
#define PWM_R   8

#define kp      8
#define ki      0.25
#define kd      1.7

extern volatile long enLPos = 0;
volatile long enLPos_k = 0;
extern volatile long spdL_pulse = 0;
extern volatile long enRPos = 0;
volatile long enRPos_k = 0;
extern volatile long spdR_pulse = 0;
extern unsigned int flag_t1 = 0;

extern long spdL_err_k   = 0;
long dspdL_err    = 0;
long sum_spdL_err = 0;
long spdL_err_k_1 = 0;

extern long spdR_err_k   = 0;
extern long dspdR_err    = 0;
extern long sum_spdR_err = 0;
extern long spdR_err_k_1 = 0;

void ISR_L();
void ISR_R();
void Motor_control_L(long ref_L_pulse);
void Motor_control_R(long ref_R_pulse);

void megapi_init();
void Timer1_ISR();
extern void Motor_control(long ref_L_pulse, long ref_R_pulse);

void megapi_init(){
  pinMode(CH_L,    INPUT);
  pinMode(CH_R,    INPUT);
  pinMode(CH_L_K,  INPUT);
  pinMode(CH_R_K,  INPUT);
  pinMode(DIR_1_L, OUTPUT);
  pinMode(DIR_2_L, OUTPUT);
  pinMode(DIR_1_R, OUTPUT);
  pinMode(DIR_2_R, OUTPUT);
  pinMode(PWM_L,   OUTPUT);
  pinMode(PWM_R,   OUTPUT);

  attachInterrupt(digitalPinToInterrupt(CH_L), ISR_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH_R), ISR_R, CHANGE);

  Timer1.initialize(20000);
  Timer1.attachInterrupt(Timer1_ISR);
}

void ISR_L(){
  if(digitalRead(CH_L) == HIGH) {
    if(digitalRead(CH_L_K) == HIGH) {
      enLPos = enLPos - 1;
    }
    else {
      enLPos = enLPos + 1;
    }
   }
  else {
      if(digitalRead(CH_L_K) == LOW) {
        enLPos = enLPos - 1;
      }
      else {
        enLPos = enLPos + 1;
      }
    }
}



void ISR_R(){
    if(digitalRead(CH_R) == HIGH) {
      if(digitalRead(CH_R_K) == HIGH) {
        enRPos = enRPos - 1; 
        }
      else {
        enRPos = enRPos +1; 
        }
     }
      // must be a high-to-low edge on channel B 
    else {
      if(digitalRead(CH_R_K) == LOW) {
         enRPos = enRPos - 1; 
     }
      else {
        enRPos = enRPos + 1; 
        }
     }
}

void Timer1_ISR(){
    flag_t1 = 1;
    spdL_pulse = enLPos - enLPos_k;
    spdR_pulse = enRPos - enRPos_k;
    enLPos_k = enLPos;
    enRPos_k = enRPos;
    digitalWrite(13, HIGH-digitalRead(13));
}

void Motor_control_L(long ref_L_pulse){
    float up, ui,ud, usum  = 0;
    unsigned int u_in = 0;
    
    spdL_err_k  =  ref_L_pulse - spdL_pulse;
    dspdL_err   =  spdL_err_k - spdL_err_k_1;
    sum_spdL_err += spdL_err_k;
  
    up = kp * (float)spdL_err_k;
    ui = ki * (float)sum_spdL_err;
    ud = kd * (float)dspdL_err;
    usum = up + ui + ud;
  
    spdL_err_k_1 = spdL_err_k;
  
    if(usum < 0){
      u_in = usum * (-1);
      digitalWrite(DIR_1_L, LOW);
      digitalWrite(DIR_2_L, HIGH);
    }
    else{
      u_in = usum;
      digitalWrite(DIR_1_L, HIGH);
      digitalWrite(DIR_2_L, LOW);
    }
  
    if(u_in > 255){
      u_in = 255;
    }
    
    analogWrite(PWM_L, u_in);
}

void Motor_control_R(long ref_R_pulse){
    float up, ui,ud, usum  = 0;
    unsigned int u_in = 0;
    
    spdR_err_k  =  ref_R_pulse - spdR_pulse;
    dspdR_err   =  spdR_err_k - spdR_err_k_1;
    sum_spdR_err += spdR_err_k;
  
    up = kp * (float)spdR_err_k;
    ui = ki * (float)sum_spdR_err;
    ud = kd * (float)dspdR_err;
    usum = (int)up + (int)ui + (int)ud;
  
    spdR_err_k_1 = spdR_err_k;
  
    if(usum < 0){
      u_in = usum * (-1);
      digitalWrite(DIR_1_R, LOW);
      digitalWrite(DIR_2_R, HIGH);
    }
    else{
      u_in = usum;
      digitalWrite(DIR_1_R, HIGH);
      digitalWrite(DIR_2_R, LOW);
    }
  
    if(u_in > 255){
      u_in = 255;
    }

    analogWrite(PWM_R, u_in);
}

void Motor_control(long ref_L_pulse, long ref_R_pulse){
    ref_L_pulse = (-1)*ref_L_pulse;
    Motor_control_L(ref_L_pulse);
    Motor_control_R(ref_R_pulse) ;  
}

#endif
