/*
* This file is part of the stmbl project.
*
* Copyright (C) 2013 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2015 Ian McMahon <facetious.ian@gmail.com>
* Copyright (C) 2013 Nico Stute <crinq@crinq.de>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include "sserial_comp.h"
#include "math.h"
#include "sserial.h"
#include "crc8.h"
#include <string.h>
#include "pico/unique_id.h"
#include "hardware/structs/systick.h"
#include "hardware/clocks.h"



static volatile uint8_t rxbuf[128];  //rx dma buffer
static volatile uint8_t txbuf[128];  //tx dma buffer
static volatile uint8_t *adress_rxbuf = &rxbuf[0];
static uint16_t address;             //current address pointer
static int rxpos;                    //read pointer for rx ringbuffer
static uint32_t timeout;
static lbp_t lbp;
static const char name[] = LBPCardName;
static unit_no_t unit;
static uint32_t max_waste_ticks;
static uint32_t block_bytes;
static pico_unique_board_id_t unit_id;

#pragma pack(push, 1) // alignement sur un octet
//*****************************************************************************
uint8_t sserial_slave[] = {
  0x0B,0x09,0x8B,0x01,0xA5,0x01,0x00,0x00,// 0..7
  0x00,0x00,0x00,0x00,0xA0,0x20,0x10,0x80,// 8..15
  0x00,0x00,0x80,0xFF,0x00,0x00,0x80,0x7F,// 16..23
  0x08,0x00,0x72,0x61,0x64,0x00,0x70,0x6F,// 24..31
  0x73,0x5F,0x63,0x6D,0x64,0x00,0x00,0x00,// 32..39
  0x00,0x00,0x00,0x00,0xA0,0x20,0x10,0x80,// 40..47
  0x00,0x00,0x80,0xFF,0x00,0x00,0x80,0x7F,// 48..55
  0x26,0x00,0x72,0x61,0x64,0x00,0x76,0x65,// 56..63
  0x6C,0x5F,0x63,0x6D,0x64,0x00,0x00,0x00,// 64..71
  0xA0,0x04,0x01,0x80,0x00,0x00,0x00,0x00,// 72..79
  0x00,0x00,0x80,0x3F,0x46,0x00,0x6E,0x6F,// 80..87
  0x6E,0x65,0x00,0x6F,0x75,0x74,0x00,0x00,// 88..95
  0xA0,0x01,0x07,0x80,0x00,0x00,0x00,0x00,// 96..103
  0x00,0x00,0x80,0x3F,0x5F,0x00,0x6E,0x6F,// 104..111
  0x6E,0x65,0x00,0x65,0x6E,0x61,0x62,0x6C,// 112..119
  0x65,0x00,0x00,0x00,0x00,0x00,0x00,0x00,// 120..127
  0xA0,0x20,0x10,0x00,0x00,0x00,0x80,0xFF,// 128..135
  0x00,0x00,0x80,0x7F,0x7A,0x00,0x72,0x61,// 136..143
  0x64,0x00,0x70,0x6F,0x73,0x5F,0x66,0x62,// 144..151
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,// 152..159
  0xA0,0x20,0x10,0x00,0x00,0x00,0x80,0xFF,// 160..167
  0x00,0x00,0x80,0x7F,0x99,0x00,0x72,0x61,// 168..175
  0x64,0x00,0x76,0x65,0x6C,0x5F,0x66,0x62,// 176..183
  0x00,0x00,0x00,0x00,0xA0,0x08,0x03,0x00,// 184..191
  0x00,0x00,0xF0,0xC1,0x00,0x00,0xF0,0x41,// 192..199
  0xB9,0x00,0x41,0x00,0x63,0x75,0x72,0x72,// 200..207
  0x65,0x6E,0x74,0x00,0x00,0x00,0x00,0x00,// 208..215
  0xA0,0x04,0x01,0x00,0x00,0x00,0xC8,0xC2,// 216..223
  0x00,0x00,0xC8,0x42,0xD4,0x00,0x6E,0x6F,// 224..231
  0x6E,0x65,0x00,0x69,0x6E,0x00,0x00,0x00,// 232..239
  0xA0,0x01,0x07,0x00,0x00,0x00,0x00,0x00,// 240..247
  0x00,0x00,0x80,0x3F,0xEE,0x00,0x6E,0x6F,// 248..255
  0x6E,0x65,0x00,0x66,0x61,0x75,0x6C,0x74,// 256..263
  0x00,0x00,0x00,0x00,0xA0,0x01,0x07,0x40,// 264..271
  0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x3F,// 272..279
  0x09,0x01,0x6E,0x6F,0x6E,0x65,0x00,0x69,// 280..287
  0x6E,0x64,0x65,0x78,0x5F,0x65,0x6E,0x61,// 288..295
  0x62,0x6C,0x65,0x00,0x00,0x00,0x00,0x00,// 296..303
  0xA0,0x20,0x10,0x80,0x00,0x00,0x80,0xFF,// 304..311
  0x00,0x00,0x80,0x7F,0x2C,0x01,0x6E,0x6F,// 312..319
  0x6E,0x65,0x00,0x73,0x63,0x61,0x6C,0x65,// 320..327
  0x00,0xB0,0x00,0x01,0x00,0x50,0x6F,0x73,// 328..335
  0x69,0x74,0x69,0x6F,0x6E,0x20,0x6D,0x6F,// 336..343
  0x64,0x65,0x00,0x00,0xA0,0x02,0x00,0x00,// 344..351
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,// 352..359
  0x5B,0x01,0x00,0x70,0x61,0x64,0x64,0x69,// 360..367
  0x6E,0x67,0x00,0x00,0xA0,0x02,0x00,0x80,// 368..375
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,// 376..383
  0x73,0x01,0x00,0x70,0x61,0x64,0x64,0x69,// 384..391
  0x6E,0x67,0x00,0x0C,0x00,0x2C,0x00,0x48,// 392..399
  0x00,0x60,0x00,0x80,0x00,0xA0,0x00,0xBC,// 400..407
  0x00,0xD8,0x00,0xF0,0x00,0x0C,0x01,0x5C,// 408..415
  0x01,0x74,0x01,0x00,0x00,0x30,0x01,0x49,// 416..423
  0x01,0x00,0x00,
};

const discovery_rpc_t discovery = {
    .ptocp  = 0x018B,
    .gtocp  = 0x01A5,
    .input  = 11,
    .output = 9,
};

typedef struct {
  float pos_cmd;
  float vel_cmd;
  uint32_t out_0 : 1;
  uint32_t out_1 : 1;
  uint32_t out_2 : 1;
  uint32_t out_3 : 1;
  uint32_t enable : 1;
  uint32_t index_enable : 1;
  uint32_t padding : 2;
} sserial_out_process_data_t;  //size:9 bytes
//_Static_assert(sizeof(sserial_out_process_data_t) == 9, "sserial_out_process_data_t size error!");

typedef struct {
  float pos_fb;
  float vel_fb;
  int8_t current;
  uint32_t in_0 : 1;
  uint32_t in_1 : 1;
  uint32_t in_2 : 1;
  uint32_t in_3 : 1;
  uint32_t fault : 1;
  uint32_t index_enable : 1;
  uint32_t padding : 2;
} sserial_in_process_data_t;  //size:10 bytes
//_Static_assert(sizeof(sserial_in_process_data_t) == 10, "sserial_in_process_data_t size error!");
//global name:scale addr:0x12c size:32 dir:0x80
#define scale_address 300
//******************************************************************************
#pragma pack(pop) // retour a l'alignement standard  

static sserial_out_process_data_t data_out;
static sserial_in_process_data_t data_in;

static uint8_t crc_reuest(uint8_t len) {
  uint8_t crc = crc8_init();
  for(int i = rxpos; i < rxpos + len; i++) {
    crc = crc8_update(crc, (void *)&(rxbuf[i % sizeof(rxbuf)]), 1);
  }
  crc = crc8_finalize(crc);
  return crc == rxbuf[(rxpos + len) % sizeof(rxbuf)];
}

static uint8_t crc8(uint8_t *addr, uint8_t len) {
  uint8_t crc = crc8_init();
  crc         = crc8_update(crc, addr, len);
  return crc8_finalize(crc);
}

static void send(uint8_t len, uint8_t docrc) {
  timeout = 0;
  dma_channel_set_read_addr(DMA_UART_TX, txbuf, false);
  if(docrc) {
    txbuf[len] = crc8((uint8_t *)txbuf, len);
    dma_channel_set_trans_count(DMA_UART_TX, (uint32_t)len + 1, true);
   
  } else {
    dma_channel_set_trans_count(DMA_UART_TX, (uint32_t)len, true);
  }
}


void enableSysTick(void)
{
  systick_hw->csr = 0;
  systick_hw->rvr = M0PLUS_SYST_RVR_BITS;                                                                        // 24 bits
  systick_hw->csr = 0x5; //processor clock, no exception
}

uint32_t hal_get_systick_value(void)
{
  return ((uint32_t)systick_hw->cvr);
}

uint32_t hal_get_systick_reload(void)
{
  return ((uint32_t)systick_hw->rvr);
}
uint32_t hal_get_systick_freq(void)
{
  return (clock_get_hz(clk_sys));
}



//v3
//pb13 txen
//pc12 usart5 tx
//pa9 usart1 tx as rx
//USART5 TX DMA1 stream7 channel4
//USART1 RX DMA2 stream5 channel4

//v4.1
//pa0 usart4 tx DMA1 stream4 channel4 CMD_36
//pa10 usart1 rx DMA2 stream5 channel4 CMD_12

//TODO: lbp command 0xe6 to set mode

void hw_init(sserial_pin_ctx_t *pins) {
  // struct sserial_ctx_t * ctx = (struct sserial_ctx_t *)ctx_ptr;

  pins->timeout = 100.0;  // 20khz / 1khz * 2 reads = 40


// init gpio for uart 

  // Set up our UART with a basic baud rate.
  uart_init(UART_ID, BAUD_RATE);
  // Set the TX and RX pins by using the function select on the GPIO
  // Set datasheet for more information on function select
  gpio_set_function(UART1_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART1_RX_PIN, GPIO_FUNC_UART);
  // Actually, we want a different speed
  // The call will return the actual baud rate selected, which will be as close as
  // possible to that requested
  int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);
  // Set UART flow control CTS/RTS, we don't want these, so turn them off
  uart_set_hw_flow(UART_ID, false, false);
  // Set our data format
  uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
  // Turn off FIFO's - we want to do this character by character
  uart_set_fifo_enabled(UART_ID, false);


/*setupt the control channel
  write rx buffer to data channel and restart 
  again and again
*/

  dma_channel_config c = dma_channel_get_default_config(DMA_CTRL);

  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, false);
  channel_config_set_chain_to(&c, DMA_UART_RX);  //chain_to trigger 

  dma_channel_configure(
        DMA_CTRL, 
        &c,
        &dma_hw->ch[DMA_UART_RX].write_addr, // write adress 
        &adress_rxbuf, //read adress
        1,
        false
        );

  dma_channel_config c2 = dma_channel_get_default_config(DMA_UART_RX);

  channel_config_set_transfer_data_size(&c2, DMA_SIZE_8);
  channel_config_set_read_increment(&c2, false);
  channel_config_set_write_increment(&c2, true);
  channel_config_set_chain_to(&c2, DMA_CTRL);
  channel_config_set_dreq(&c2, uart_get_dreq(uart1, false)); // DMA paced by UART RX


  dma_channel_configure(
        DMA_UART_RX, 
        &c2,
        rxbuf,    
        &uart1_hw->dr,
        sizeof(rxbuf),
        false
        );

 dma_channel_config c3 = dma_channel_get_default_config(DMA_UART_TX);

  channel_config_set_transfer_data_size(&c3, DMA_SIZE_8);
  channel_config_set_read_increment(&c3, true);
  channel_config_set_write_increment(&c3, false);
  channel_config_set_dreq(&c3, uart_get_dreq(uart1, true));// DMA paced by UART TX

  dma_channel_configure(
        DMA_UART_TX, 
        &c3,
        &uart1_hw->dr, // UART data register
        txbuf,
        sizeof(txbuf),
        false
        );

  //generate unit number from 96bit unique chip ID

  pico_get_unique_board_id(&unit_id);
  unit.byte[0] = unit_id.id[0];
  unit.byte[1] = unit_id.id[1];
  unit.byte[2] = unit_id.id[2];
  unit.byte[3] = unit_id.id[3]; 
  
  rxpos   = 0;
  timeout = 1000;  //make sure we start in timeout

  //bytes to wait before expected end of transmission to prevent timeouts
  block_bytes = 5;
  enableSysTick();
  //calculate timeout in systicks for block_bytes
  max_waste_ticks = (1.0 / 2500000.0) * 11.0 * (float)block_bytes / (1.0f / (float)hal_get_systick_freq());

  pins->clock_scale= 1.0;
  pins->phase = 0;

  dma_start_channel_mask(1u << DMA_CTRL); // start CTRL DMA
  dma_start_channel_mask(1u << DMA_UART_TX);
}

void frt_func(sserial_pin_ctx_t *pins) {
  gpio_put(16, true);
  //next received packet will be written to bufferpos
  uint32_t bufferpos= sizeof(rxbuf) - (dma_hw->ch[DMA_UART_RX].transfer_count);
  //how many packets we have the the rx buffer for processing
  uint32_t available = (bufferpos - rxpos + sizeof(rxbuf)) % sizeof(rxbuf);

  pins->phase += 1.0;


  uint32_t goal = 5;
  pins->clock_scale= 1.0;
  if(pins->phase > 3){
    pins->phase = 0;
    if(available > goal){
    pins->clock_scale = 0.9;
    }
    else if(available < goal && available > 0){
      pins->clock_scale = 1.1;
    }
  }

  pins->available = available;

  if(available >= 1) {
    lbp.byte = rxbuf[rxpos];

    if(lbp.ct == CT_LOCAL && lbp.wr == 0) {  //local read, cmd+crc = 2b
      timeout = 0;
      if(available >= 2) {
        switch(lbp.byte) {
          case LBPCookieCMD:
            txbuf[0] = LBPCookie;
            break;
          case LBPStatusCMD:  //TODO: return status
            txbuf[0] = 0x00;
            break;
          case LBPCardName0Cmd ... LBPCardName3Cmd:
            txbuf[0] = name[lbp.byte - LBPCardName0Cmd];
            break;
          default:  //TODO: handle unknown command condition
            txbuf[0] = 0x00;
        }
        send(1, 1);
        rxpos += 2;
      }
    } else if(lbp.ct == CT_LOCAL && lbp.wr == 1) {  //local write, cmd+data+crc = 3b
      timeout = 0;
      //0xFF and 0xFC are not followed by crc
      if(rxbuf[rxpos] == 0xFF) {
        // reset parser
        rxpos += 1;
      } else if(rxbuf[rxpos] == 0xFC) {
        // todo
        rxpos += 1;
      } else if(available >= 3) {  //writes do not expect crc in reply
        txbuf[0] = 0x00;
        send(1, 0);
        rxpos += 3;
      }
    } else if(lbp.ct == CT_RPC) {  //RPC TODO: check for ct should not required for rpc
      timeout = 0;
      if(lbp.byte == UnitNumberRPC && available >= 2) {  //unit number, cmd+crc = 2b
        txbuf[0] = unit.byte[0];
        txbuf[1] = unit.byte[1];
        txbuf[2] = unit.byte[2];
        txbuf[3] = unit.byte[3];
        send(4, 1);
        rxpos += 2;
      } else if(lbp.byte == DiscoveryRPC && available >= 2) {  //discovery, cmd+crc = 2b
        memcpy((void *)txbuf, ((uint8_t *)&discovery), sizeof(discovery));
        send(sizeof(discovery), 1);
        rxpos += 2;
      } else if(lbp.byte == ProcessDataRPC && available >= discovery.output + 2 - block_bytes) {  //process data, requires cmd+output bytes+crc
        uint32_t t1         = hal_get_systick_value();
        uint32_t wait_ticks = 0;
        //wait with timeout until rest of process data is received
        do {
          uint32_t t2 = hal_get_systick_value();
          if(t1 < t2) {
            t1 += hal_get_systick_reload();
          }
          wait_ticks = t1 - t2;
          //next received packet will be written to bufferpos
          
          bufferpos = sizeof(rxbuf) - (dma_hw->ch[DMA_UART_RX].transfer_count);  // remaining data

          //how many packets we have the the rx buffer for processing
          available = (bufferpos - rxpos + sizeof(rxbuf)) % sizeof(rxbuf);
        } while(available < discovery.output + 2 && wait_ticks <= max_waste_ticks);
        //TODO: fault handling on timeout...
        //set input pins
        data_in.pos_fb  = pins->pos_fb + pins->vel_fb * pins->pos_advance;
        data_in.vel_fb  = pins->vel_fb;
        data_in.current = pins->current;
        data_in.in_0    = (pins->in0 > 0) ? 1 : 0;
        data_in.in_1    = (pins->in1 > 0) ? 1 : 0;
        data_in.in_2    = (pins->in2 > 0) ? 1 : 0;
        data_in.in_3    = (pins->in3 > 0) ? 1 : 0;
        data_in.fault   = (pins->fault > 0) ? 1 : 0;

        //copy output pins from rx buffer
        for(int i = 0; i < discovery.output; i++) {
          ((uint8_t *)(&data_out))[i] = rxbuf[(rxpos + i + 1) % sizeof(rxbuf)];
        }

        //set bidirectional pins
        pins->index_out       = data_out.index_enable;
        data_in.index_enable = (pins->index_clear > 0) ? 0 : data_out.index_enable;

        //copy input pins to tx buffer
        txbuf[0] = 0x00;  //fault byte
        for(int i = 0; i < (discovery.input - 1); i++) {
          txbuf[i + 1] = ((uint8_t *)(&data_in))[i];
        }
        if(crc_reuest(discovery.output + 1)) {
          //send buffer
          dma_channel_abort(DMA_UART_TX);
          dma_channel_set_read_addr(DMA_UART_TX, txbuf, false);
          dma_channel_set_trans_count(DMA_UART_TX, (uint32_t)discovery.input + 1, true);

          txbuf[discovery.input] = crc8((uint8_t *)txbuf, discovery.input);
          //send(discovery.input, 1);
          timeout = 0;
          //set output pins

          pins->pos_cmd   = data_out.pos_cmd;
          pins->pos_cmd_d = data_out.vel_cmd;
          pins->out0      = data_out.out_0;
          pins->out1      = data_out.out_1;
          pins->out2      = data_out.out_2;
          pins->out3      = data_out.out_3;
          pins->enable    = data_out.enable;
        } else {
          pins->crc_error
          ++;
          pins->connected = 0;
          pins->error     = 1;
          pins->pos_cmd   = 0;
          pins->pos_cmd_d = 0;
          pins->out0      = 0;
          pins->out1      = 0;
          pins->out2      = 0;
          pins->out3      = 0;
          pins->enable    = 0;
        }
        rxpos += discovery.output + 2;
      }
    } else if(lbp.ct == CT_RW && lbp.wr == 0) {  //read
      //size = 1 + 2*lbp.as  + 1
      int size = 2 * lbp.as + 2;
      timeout  = 0;
      if(available >= size) {
        if(lbp.as) {  //address included in command = cmd+addr+addr+crc
          address = rxbuf[(rxpos + 1) % sizeof(rxbuf)] + (rxbuf[(rxpos + 2) % sizeof(rxbuf)] << 8);
          rxpos += 4;
        } else {  //address not included in command = cmd+crc
          rxpos += 2;
        }
        //TODO: causes timeouts...
        //if((address + (1 << lbp.ds)) < ARRAY_SIZE(sserial_slave)) {  //check if address is valid
        memcpy((void *)txbuf, &sserial_slave[address], (1 << lbp.ds));
        send((1 << lbp.ds), 1);
        //}
        if(lbp.ai) {  //auto increment address by datasize
          address += (1 << lbp.ds);
        }
      }
    } else if(lbp.ct == CT_RW && lbp.wr == 1) {  // lbp (addr1 addr2) data0, data1,...
      //size = 1 + 2*ai +ds +crc
      int size = 2 * lbp.as + (1 << lbp.ds) + 2;
      timeout  = 0;
      if(available >= size) {
        if(lbp.as) {  //address included in command = cmd+addr+addr+crc
          address = rxbuf[(rxpos + 1) % sizeof(rxbuf)] + (rxbuf[(rxpos + 2) % sizeof(rxbuf)] << 8);
          rxpos += 3;
        } else {  //address not included in command = cmd+crc
          rxpos += 1;
        }
        //TODO: check size
        if((address + (1 << lbp.ds)) < ARRAY_SIZE(sserial_slave)) {  //check if address is valid
          for(int i = 0; i < (1 << lbp.ds); i++) {
            sserial_slave[address + i] = rxbuf[(rxpos + i) % sizeof(rxbuf)];
          }
        }
        rxpos += (1 << lbp.ds) + 1;
        //update globals
        float tmp;
        memcpy(&tmp, &sserial_slave[scale_address], 4);
        pins->scale = tmp;
        if(lbp.ai) {  //auto increment address by datasize
          address += (1 << lbp.ds);
        }
      }
    } else {
      //TODO: handle unkown packet
    }
  }

  if(timeout > pins->timeout) {  //TODO: clamping
    pins->connected = 0;
    pins->error     = 1;
    pins->pos_cmd   = 0;
    pins->pos_cmd_d = 0;
    pins->out0      = 0;
    pins->out1      = 0;
    pins->out2      = 0;
    pins->out3      = 0;
    pins->enable    = 0;
    rxpos          = bufferpos;
    timeout = 0;
  } else {
    pins->connected = 1;
    pins->error     = 0;
  }
  rxpos = rxpos % sizeof(rxbuf);
  timeout++;
  gpio_put(16, false);
}

