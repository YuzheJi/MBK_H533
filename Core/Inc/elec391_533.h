#ifndef __ELEC391_533_H
#define __ELEC391_533_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define BT_BAUD_RATE 921600
#define PI 3.141593f
#define GEAR 33.8f

#ifdef GLOBAL_DEFINITION
    #define __EXTERN 
#else
    #define __EXTERN extern
#endif

#ifndef FRAME_RATE
 #define FRAME_RATE 25  // in ms
#endif

#ifndef MEASURE_RATE
 #define MEASURE_RATE 10  // in ms
#endif

#ifndef BT_RX_LEN
 #define BT_RX_LEN 64  
#endif

#ifndef NUM_LYRICS_LINE
    #define NUM_LYRICS_LINE 6  
#endif

#ifndef MAX_CHORD_NOTES
    #define MAX_CHORD_NOTES 10  
#endif

#ifndef SONG_EVENT
    #define SONG_EVENT 10  
#endif

// wait till settle: t = SETTLE_TH * 10ms
#ifndef SETTLE_TH
    #define SETTLE_TH 5  
#endif

__EXTERN float vofa_buf[7]; 

__EXTERN uint8_t BT_DMA_rx_buff[BT_RX_LEN]; 

__EXTERN uint8_t msec_count_frame;
__EXTERN uint8_t half_msec_count_measure;

__EXTERN volatile uint8_t homing_flag;
__EXTERN volatile uint8_t system_mode;
__EXTERN volatile uint8_t mode3_state;

__EXTERN volatile uint8_t main_update;
__EXTERN volatile int32_t counter_acc;
__EXTERN volatile int16_t counter;

__EXTERN volatile float speed;
__EXTERN volatile float rad_s;
__EXTERN volatile float location;
__EXTERN volatile float target;
__EXTERN volatile uint8_t settle_count;
__EXTERN volatile uint16_t press_count;

__EXTERN volatile uint8_t event_index;

__EXTERN volatile float err_prev;
__EXTERN volatile float err_acc;
__EXTERN volatile float err;

__EXTERN float Kp;
__EXTERN float Ki;
__EXTERN float Kd;


__EXTERN uint8_t temp;

typedef struct ChordEvent {
    float positions[2]; // target position for both left hand and right hand
    bool pressed[MAX_CHORD_NOTES]; // True -> pressed, False -> not pressed
    uint16_t duration_ms; // how long it needs to be pressed
    uint16_t delay_to_next_ms; // to next postions how many time left
} ChordEvent_t;

extern ChordEvent_t SONG[SONG_EVENT];

#endif