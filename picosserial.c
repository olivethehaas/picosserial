/*

*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "sserial_comp.h"
#include "sserial.h"
#include "hardware/clocks.h"
#include "hardware/structs/systick.h"
#include "hardware/dma.h"

#define ALARM_NUM 0
#define ALARM_IRQ timer_hardware_alarm_get_irq_num(timer_hw, ALARM_NUM)
#define DELAY 50 // 20 KHZ frt_func call

sserial_pin_ctx_t pin_struct = {
    .error = 0.0,
    .crc_error = 2.0,
    .connected = 3.0,
    .timeout = 4.0,
    .scale = 5.0,
    .clock_scale = 6.0,
    .available = 7.0,
    .phase = 8.0,
    .in0 = 9.0,
    .in1 = 10.0,
    .in2 = 11.0,
    .in3 = 12.0,
    .fault = 13.0,
    .out0 = 14.0,
    .out1 = 15.0,
    .out2 = 16.0,
    .out3 = 17.0,
    .enable = 18.0,
};



static void timer_irq(void)
{
  // Clear the alarm irq
  hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);
  //reset alarm register
  timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY ;
  frt_func(&pin_struct);


}

int main()
{
  stdio_init_all();
  gpio_init(16);
  gpio_set_dir(16, GPIO_OUT);

  hw_init(&pin_struct);
  hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
  // Set irq handler for alarm irq
  irq_set_exclusive_handler(ALARM_IRQ, timer_irq);
  // Enable the alarm irq
  irq_set_enabled(ALARM_IRQ, true);
  // Enable interrupt in block and at processor

  // Alarm is only 32 bits so if trying to delay more
  // than that need to be careful and keep track of the upper
  // bits
  uint64_t target = timer_hw->timerawl + DELAY;

  // Write the lower 32 bits of the target time to the alarm which
  // will arm it
  timer_hw->alarm[ALARM_NUM] = (uint32_t)target;

  printf("look good !\n");

  while (1)
  {
    printf("dro X = %f, Y = %f, Z = %f\n",pin_struct.x_pos , pin_struct.y_pos, pin_struct.z_pos);
    sleep_ms(100);
  }
}
