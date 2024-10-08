//
#pragma once
//generated by create_comp_h.py DO NOT EDIT

typedef struct { // found in sserial.c
   float error;
   float x_pos;
   float y_pos;
   float z_pos;   
   float pos_fb; 
   float vel_fb;      
   float crc_error;
   float connected;
   float timeout;
   float scale;
   float clock_scale;
   float available;
   float phase;
   float in0;
   float in1;
   float in2;
   float in3;
   float fault;
   float out0;
   float out1;
   float out2;
   float out3;
   float enable;
   float current;  
   float index_clear;
   float index_out;
}sserial_pin_ctx_t;
