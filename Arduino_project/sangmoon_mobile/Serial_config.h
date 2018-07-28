#ifndef   SERIAL_CONFIG_H_
#define   SERIAL_CONFIG_H_

#include <string.h>

#define BT_UP     0
#define BT_RIGHT  1
#define BT_DOWN   2
#define BT_LEFT   3
#define BT_E      4
#define BT_F      5
#define BT_JOY    6

int BT[7] = {0,0,0,0,0,0,0};
int rx_led = 0;

extern int joyval_x = 0;
extern int joyval_y = 0;
byte  bt_temp = 0;

//for uart3
byte  rx_buffer[10];
byte  rx_data[10];
byte  rcv_checksum = 0;
int   rcv_count = 0;
int   rcv_ready = 0;
int   rcv_index = 0;
int   rcv_status = 0;
extern int   x_value = 0;
extern int   y_value = 0;

void Serial_init() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  Serial3.begin(115200);
}

void Joystick_update() { 
  if(rcv_status == 1){
    rcv_status = 0;
    
    memcpy(&joyval_x, &rx_data[0], 2);
    memcpy(&joyval_y, &rx_data[2], 2);

    bt_temp = rx_data[4];
    BT[BT_UP] = bt_temp & 0x01;
    BT[BT_RIGHT] = (bt_temp & 0x02) >> 1;
    BT[BT_DOWN] = (bt_temp & 0x04) >> 2;
    BT[BT_LEFT] = (bt_temp & 0x08) >> 3;
    BT[BT_E] = (bt_temp & 0x10) >> 4;
    BT[BT_F] = (bt_temp & 0x20) >> 5;
    BT[BT_JOY] = (bt_temp & 0x40) >> 6;
    x_value = map(joyval_x, 0, 1023, -20, 21);
    y_value = map(joyval_y, 0, 1023, -30 ,31);
    
    //Serial.println(y_value);
  }
  
}

void serialEvent3(){
  byte  rcv_data;
  rcv_data = (byte)Serial3.read();

  switch(rcv_count){
    case 0:
      if((rcv_ready==0) && (rcv_data==0xFF)){
        rcv_count = 1;
      }
      else
        rcv_count = 0;
      break;

    case 1:
      if((rcv_ready==0) && (rcv_data==0xFF)){
        rcv_count = 2;
        rcv_ready = 1;
      }
      else
        rcv_count = 0;
      break;

    case 2:
      rx_buffer[rcv_index] = rcv_data;
      //rcv_checksum ^= rx_buffer[rcv_index];
      rcv_index++;

      if(rcv_index > 5){  // 수신 data 개수 6
        rcv_checksum = 0;
        for (int i=0; i<5; i++){
          rcv_checksum ^= rx_buffer[i];
        }
        rcv_checksum += 1;
        if(rcv_checksum == rx_buffer[rcv_index-1]){
          memcpy(&rx_data[0], &rx_buffer[0], 5);
          rcv_status = 1;
        }
        rcv_count = 0;
        rcv_index = 0;
        rcv_ready = 0;
        rcv_checksum = 0;
        rx_led ^= 1;
      }
      break;
  }
    
}


#endif
