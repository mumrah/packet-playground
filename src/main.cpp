#include <Wire.h>
#include <Arduino.h>
#include <CircularBuffer.h>
#include <cc1101.h>
#include <ccpacket.h>

#include "ax25.h"
#include "HDLC.h"

#define CC1101Interrupt 0 // Pin 2
#define CC1101_GDO0 2

CC1101 radio;

byte syncWord[2] = {199, 10};
bool packetWaiting;

unsigned long lastSend = 0;
unsigned int sendDelay = 5000;

void messageReceived() {
    packetWaiting = true;
}

#define FEND 0xC0
#define FESC 0xDB
#define TFEND 0xDC
#define TFESC 0xDD

// KISS commands
#define CMD_UNKNOWN 0xFE
#define CMD_DATA 0x00
#define CMD_TXDELAY 0x01
#define CMD_P 0x02
#define CMD_SLOTTIME 0x03
#define CMD_TXTAIL 0x04
#define CMD_FULLDUPLEX 0x05
#define CMD_SETHARDWARE 0x06
#define CMD_RETURN 0xFF

#define AX25_MAX_FRAME_LEN 330

#define I2C_ADDRESS 0x04

#define DEBUG true

#if DEBUG
#define PRINTS(s)   { Serial.print(F(s)); }
#define PRINT(v)    { Serial.print(v); }
#define PRINTLN(v)  { Serial.println(v); }
#define PRINTLNS(s)  { Serial.println(F(s)); }
#define PRINT2(s,v)  { Serial.print(F(s)); Serial.print(v); }
#define PRINT3(s,v,b) { Serial.print(F(s)); Serial.print(v, b); }
#else
#define PRINTS(s)
#define PRINT(v)
#define PRINTLN(v)
#define PRINTLNS(s)
#define PRINT2(s,v)
#define PRINT3(s,v,b)
#endif

// types
typedef struct KISSCtx {
    size_t frame_len = 0;
    bool in_escape = false;
    bool in_frame = false;
    uint8_t command = CMD_UNKNOWN;
    uint8_t hdlc_port = 0;
    uint8_t buffer[AX25_MAX_FRAME_LEN];
} KISSCtx;

// Get signal strength indicator in dBm.
// See: http://www.ti.com/lit/an/swra114d/swra114d.pdf
int rssi(char raw) {
    uint8_t rssi_dec;
    // TODO: This rssi_offset is dependent on baud and MHz; this is for 38.4kbps and 433 MHz.
    uint8_t rssi_offset = 74;
    rssi_dec = (uint8_t) raw;
    if (rssi_dec >= 128)
        return ((int)( rssi_dec - 256) / 2) - rssi_offset;
    else
        return (rssi_dec / 2) - rssi_offset;
}

// Get link quality indicator.
int lqi(char raw) {
    return 0x3F - raw;
}

// globals
CircularBuffer<uint8_t, 400> i2c_input_buffer;
CircularBuffer<uint8_t, 100> i2c_output_buffer;

KISSCtx kissCtx;
AX25Call dst;
AX25Call src;

HDLC incomingFrame;
HDLC outgoingFrame;
CCPACKET packet;

/*
 * Called once we've decoded an entire packet from a KISS frame
 */
void on_ax25_packet(uint8_t buffer[], size_t len) {
  detachInterrupt(CC1101Interrupt);
  CCPACKET packet;
  for(uint8_t i=0; i<len; i+=20) {
    size_t n = min(20, len-i);
    memset((char *)packet.data, '\0', 64);
    memcpy((char *)packet.data, buffer + i, n);
    packet.length = n + 1;
    radio.sendData(packet);
    delay(100);
  }
  attachInterrupt(CC1101Interrupt, messageReceived, FALLING);
}

/*
 * Called as each byte of a KISS frame is received over i2c. Bytes are buffered
 * in the global KISSCtx buffer which has enough room for the max length AX25
 * frame (330 bytes).
 */
void poll_kiss(uint8_t b, KISSCtx *kiss) {
  if (kiss->in_frame && b == FEND && kiss->command == CMD_DATA) {
    // end of a data frame
    kiss->in_frame = false;
    on_ax25_packet(kiss->buffer, kiss->frame_len);
  } else if (b == FEND) {
    // beginning of data frmae
    kiss->in_frame = true;
    kiss->command = CMD_UNKNOWN;
    kiss->frame_len = 0;
  } else if (kiss->in_frame && kiss->frame_len < AX25_MAX_FRAME_LEN) {
    // in a frame, check for commands first
    if (kiss->frame_len == 0 && kiss->command == CMD_UNKNOWN) {
      kiss->hdlc_port = b & 0xF0; // multiple HDLC ports are supported by KISS, apparently
      kiss->command = b & 0x0F;
    } else if (kiss->command == CMD_DATA) {
      if (b == FESC) {
        kiss->in_escape = true;
      } else {
        if (kiss->in_escape) {
          if (b == TFEND) b = FEND;
          if (b == TFESC) b = FESC;
          kiss->in_escape = false;
        }
        kiss->buffer[kiss->frame_len++] = b;
      }
    } // TODO implement other commands
  }
}

/**
 * Given a buffer and its length, write the buffer out over Serial encoded
 * with KISS framing
 */
void serial_kiss_wrapper(uint8_t buffer[], size_t len) {
  Serial.write(FEND);
  Serial.write(0x00);
  for (unsigned i = 0; i < len; i++) {
    uint8_t b = buffer[i];
    if (b == FEND) {
      Serial.write(FESC);
      Serial.write(TFEND);
    } else if (b == FESC) {
      Serial.write(FESC);
      Serial.write(TFESC);
    } else {
      Serial.write(b);
    }
  }
  Serial.write(FEND);
}

/*
 * i2c master is requesting data from us. Write out anything in the output
 * buffer, otherwise send 0x0E to indicate we have nothing to write
 */
void on_i2c_read_request() {
  if(i2c_output_buffer.isEmpty()) {
    //debug("Writing 0x0E to TNC");
    Wire.write(0x0E); // Tell the TNC we've got nothing
  } else {
    PRINTS("Writing data to TNC");

    uint8_t tmp[8];
    uint8_t i;
    for(i=0; i<min(i2c_output_buffer.size(), 8); i++) {
      tmp[i] = i2c_output_buffer.pop();
    }
    Wire.write(tmp, i);
  }
}

/*
 * i2c master is writing to us, read as many bytes from the i2c
 * buffer as we can into our own input buffer
 */
void on_i2c_write_receive(int n) {
  //debug("on_i2c_write_receive");
  while (Wire.available() && i2c_input_buffer.available() > 0) {
    uint8_t byte = Wire.read();
    //Serial.println(byte, HEX);
    bool res = i2c_input_buffer.push(byte);
    if(!res) {
      PRINTLN("!!!! Buffer overrun !!!!");
    }
  }
}

uint8_t next_seq_num = 0;

uint8_t get_next_seq_num() {
  next_seq_num = (next_seq_num + 1) % 8;
  return next_seq_num;
}

void hdlc_send_data(HDLC * hdlc, CCPACKET * packet) {
  packet->data[0] = hdlc->address & 0xFF;
  packet->data[1] = hdlc->control & 0xFF;
  memcpy(&(packet->data[2]), hdlc->data, hdlc->data_length);
  packet->length = hdlc->data_length + 2;
  //delayMicroseconds(75); // settling time for other receiver
  detachInterrupt(CC1101Interrupt);
  radio.sendData(*packet);
  attachInterrupt(CC1101Interrupt, messageReceived, FALLING);
  hdlc->send_attempts += 1;
  hdlc->time_sent = millis();
  hdlc->do_retry = false;
  hdlc->ack = false;
}

HDLC supervisory;
void hdlc_send_ack(CCPACKET * packet, uint8_t seq) {
  hdlc_new_ack_frame(&supervisory, seq);
  hdlc_send_data(&supervisory, packet);
}

void read_hdlc(CCPACKET * packet, HDLC * hdlc) {
  hdlc->address = packet->data[0];
  hdlc->control = packet->data[1];
  memcpy(hdlc->data, &(packet->data[2]), packet->length - 2);
  hdlc->data_length = packet->length - 2;
}

void debug_state(char * msg) {
  PRINT("=================="); PRINT(msg); PRINTLN("==================");
  PRINT2("packetWaiting: ",packetWaiting); PRINTLN("");

  PRINT2("incomingFrame#ack: ", incomingFrame.ack); PRINTLN("");
  PRINT2("incomingFrame#time_sent: ", incomingFrame.time_sent); PRINTLN("");
  PRINT2("incomingFrame#send_attempts: ", incomingFrame.send_attempts); PRINTLN("");
  PRINT2("incomingFrame#do_retry: ", incomingFrame.do_retry); PRINTLN("");

  PRINT2("outgoingFrame#ack: ", outgoingFrame.ack); PRINTLN("");
  PRINT2("outgoingFrame#time_sent: ", outgoingFrame.time_sent); PRINTLN("");
  PRINT2("outgoingFrame#send_attempts: ", outgoingFrame.send_attempts); PRINTLN("");
  PRINT2("outgoingFrame#do_retry: ", outgoingFrame.do_retry); PRINTLN("");

}

void setup() {
  radio.init();
  radio.setSyncWord(syncWord);
  radio.setCarrierFreq(CFREQ_433);
  radio.disableAddressCheck();
  radio.setTxPowerAmp(PA_LowPower);
  attachInterrupt(CC1101Interrupt, messageReceived, FALLING);

  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(on_i2c_read_request);
  Wire.onReceive(on_i2c_write_receive);

  Serial.begin(9600);
  PRINTLN("CC1101_PARTNUM ");
  PRINTLN(radio.readReg(CC1101_PARTNUM, CC1101_STATUS_REGISTER));
  PRINTLN("CC1101_VERSION ");
  PRINTLN(radio.readReg(CC1101_VERSION, CC1101_STATUS_REGISTER));
  PRINTLN("CC1101_MARCSTATE ");
  PRINTLN(radio.readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1f);

  PRINTLN("CC1101 radio initialized.");
  PRINTLN("Begin!");
}

uint8_t serial_read_buffer[48];

#define TX_COUNTDOWN_MS 50

unsigned long last_tx = 0;

CircularBuffer<uint8_t, 8> ack_queue;

void loop() {
  bool newFrame = false;
  // Check incoming packet
  if(packetWaiting) {
    detachInterrupt(CC1101Interrupt);
    packetWaiting = false;
    if (radio.receiveData(&packet) > 0) {
      if (packet.crc_ok && packet.length > 0) {
        read_hdlc(&packet, &incomingFrame);
        newFrame = true;
      }
    }
    attachInterrupt(CC1101Interrupt, messageReceived, FALLING);
  }

  // Handle new HDLC frame
  if(newFrame) {
    switch(hdlc_get_frame_type(&incomingFrame)) {
      case HDLC_I_FRAME: { // Information (data) frame
        uint8_t seq = hdlc_get_i_frame_send_seq(&incomingFrame);

        // Buffer output
        for(uint8_t i=0; i<incomingFrame.data_length; i++) {
          bool res = i2c_output_buffer.push(incomingFrame.data[i]);
          if(!res) {
            PRINTLN("!!! Output Buffer Overrun !!!");
            break;
          }
        }

        // Buffer ACK
        ack_queue.push(seq);
      } break;
      case HDLC_S_FRAME: { // Supervisory frame
        uint8_t recv_seq = hdlc_get_s_frame_recv_seq(&incomingFrame);
        if(hdlc_get_s_frame_type(&incomingFrame) == HDLC_S_TYPE_RR) {
          uint8_t sent_seq = hdlc_get_i_frame_send_seq(&outgoingFrame);
          if(sent_seq != recv_seq) {
            // TODO what to do here?
          }
        }
      } break;
      case HDLC_U_FRAME: {
      } break;
      default:
        break;
    }
  }

  // Check incoming serial data
  while (Serial.available() && i2c_input_buffer.available() > 0) {
    uint8_t byte = Serial.read();
    bool res = i2c_input_buffer.push(byte);
    if(!res) {
      // Shouldn't happen
      PRINTLN("!!!! Input Buffer Overrun !!!!");
      break;
    }
  }

  // Decide if we need to TX
  unsigned long now = millis();
  if(now - last_tx > TX_COUNTDOWN_MS) {
    if(ack_queue.size() > 0) {
      hdlc_send_ack(&packet, ack_queue.shift());
      last_tx = now;
    } else if(i2c_input_buffer.size() > 0) {
      uint8_t n = min(i2c_input_buffer.size(), 48);
      if(n > 0) {
        for(uint8_t i=0; i<n; i++) {
          serial_read_buffer[i] = i2c_input_buffer.shift();
        }
        hdlc_new_data_frame(&outgoingFrame, serial_read_buffer, n, get_next_seq_num());
        hdlc_send_data(&outgoingFrame, &packet);
        last_tx = now;
      }
    }
  }

  // Output Serial, if any
  while(i2c_output_buffer.size() > 0) {
    Serial.write(i2c_output_buffer.shift());
  }
}

void loop2() {
  // Check for incoming packet and copy it to a frame
  if (packetWaiting) {
    detachInterrupt(CC1101Interrupt);
    packetWaiting = false;
    bool gotPacket = false;
    if (radio.receiveData(&packet) > 0) {
      if (packet.crc_ok && packet.length > 0) {
        //PRINT2("Got packet with ", packet.length);
        //PRINTLN(" bytes");
        read_hdlc(&packet, &incomingFrame);
        gotPacket = true;
      }
    }
    attachInterrupt(CC1101Interrupt, messageReceived, FALLING);

    // Handle the incoming frame
    if(gotPacket) {
      switch(hdlc_get_frame_type(&incomingFrame)) {
        case HDLC_I_FRAME: {
          // incoming data frame
          uint8_t seq = hdlc_get_i_frame_send_seq(&incomingFrame);
          PRINT2("RECV_DATA[", seq);
          PRINTLN("]");
          //debug_state("RECV_DATA");

          Serial.write(incomingFrame.data, incomingFrame.data_length);
          delay(500);
          hdlc_send_ack(&packet, seq);
          PRINT2("SEND_ACK[", seq);
          PRINTLN("]");
          //debug_state("SEND_ACK");
        } break;
        case HDLC_S_FRAME: {
          uint8_t recv_seq = hdlc_get_s_frame_recv_seq(&incomingFrame);
          if(hdlc_get_s_frame_type(&incomingFrame) == HDLC_S_TYPE_RR) {
            PRINT2("RECV_ACK[", recv_seq);
            PRINTLN("]");
            //debug_state("RECV_ACK");
            uint8_t sent_seq = hdlc_get_i_frame_send_seq(&outgoingFrame);
            if(sent_seq != recv_seq) {
              // TODO what to do here?
              PRINT2("Unexpected seq number, expected ", sent_seq);
              PRINT2(" got ", recv_seq);
              PRINTLN("");
            }
            outgoingFrame.ack = true;
          } else if(hdlc_get_s_frame_type(&incomingFrame) == HDLC_S_TYPE_REJ) {
            PRINT2("RECV_NACK[", recv_seq);
            PRINTLN("]");
            outgoingFrame.ack = false;
            outgoingFrame.do_retry = true;
          }
        } break;
        case HDLC_U_FRAME: {
          PRINTLN("Unexpected U frame received, ignoring");
        } break;
        default:
          break;
      }
    }
  }

  // Check Serial for more data
  while (Serial.available() && i2c_input_buffer.available() > 0) {
    uint8_t byte = Serial.read();
    bool res = i2c_input_buffer.push(byte);
    if(!res) {
      PRINTLN("!!!! Buffer overrun !!!!");
    }
  }

  // Check if we're waiting on an ACK message
  if(outgoingFrame.ack == false) {
    if(millis() - outgoingFrame.time_sent > (2000 + outgoingFrame.send_attempts * 20)) { // timeout
      outgoingFrame.do_retry = true;
    }

    // Check if we need to resend last frame
    if(outgoingFrame.do_retry == true) {
      if(outgoingFrame.send_attempts > 7) { // too many retries
        PRINT2("DROP[", hdlc_get_i_frame_send_seq(&outgoingFrame));
        PRINTLN("]");
        //debug_state("drop");
        outgoingFrame.do_retry = false;
        outgoingFrame.failed = true;
        outgoingFrame.ack = true;
      } else {
        PRINT2("RESEND[", hdlc_get_i_frame_send_seq(&outgoingFrame));
        PRINTLN("]");
        //debug_state("resend");
        hdlc_send_data(&outgoingFrame, &packet);
      }
    } else {
      // still waiting for ACK, loop
      //delay(1);
    }
  } else {
    // last frame is ACK'd we're ready to send next byte


    // Send data if we have any waiting
    uint8_t n = min(i2c_input_buffer.size(), 48);
    if(n > 0) {
      for(uint8_t i=0; i<n; i++) {
        serial_read_buffer[i] = i2c_input_buffer.shift();
      }
      uint8_t seq = get_next_seq_num();
      PRINT2("SEND[", seq);
      PRINTLN("]");
      hdlc_new_data_frame(&outgoingFrame, serial_read_buffer, n, seq);
      hdlc_send_data(&outgoingFrame, &packet);
    }
    /*
    if(n > 0) {
      i2c_input_buffer
      Serial.readBytes(serial_read_buffer, n);
      uint8_t seq = get_next_seq_num();
      PRINT2("SEND[", seq);
      PRINTLN("]");
      //debug_state("send");
      hdlc_new_data_frame(&outgoingFrame, serial_read_buffer, n, seq);
      hdlc_send_data(&outgoingFrame, &packet);
    } else {
      // not waiting for anything, but no data ready to send, loop
      //delay(1);
    }
    */
  }

}
