#include "Megapi_config.h"
#include "Serial_config.h"


void setup() {
  // put your setup code here, to run once:
  megapi_init();
  Serial_init();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(flag_t1){
    flag_t1 = 0;
    Motor_control(y_value+x_value, y_value-x_value);
    //Serial.println(x_value);
  }
  Joystick_update();
}

