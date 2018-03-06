#include <Wire.h>
#include <Arduino.h>
#include <CircularBuffer.h>
#include <cc1101.h>
#include <ccpacket.h>

#include "ax25.h"

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
#define DEBUG false

// types
typedef struct KISSCtx {
    size_t frame_len = 0;
    bool in_escape = false;
    bool in_frame = false;
    uint8_t command = CMD_UNKNOWN;
    uint8_t hdlc_port = 0;
    uint8_t buffer[AX25_MAX_FRAME_LEN];
} KISSCtx;

// util functions

void debug(const char * message) {
  if(DEBUG) {
    //Serial.println(message);
  }
}

void debug(uint8_t message) {
  if(DEBUG) {
    //Serial.println(message);
  }
}

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

/*
 * Called once we've decoded an entire packet from a KISS frame
 */
void on_ax25_packet(uint8_t buffer[], size_t len) {
/*
  debug("Got AX.25 packet!");
  // do a little ax.25 parsing to print out DST and SRC
  char dst_call[7];
  for(uint8_t i=0; i<6; i++) {
    dst_call[i] = buffer[i] >> 1;
  }
  dst_call[6] = 0;

  uint8_t dst_ssid = (buffer[6] >> 1) & 0xf;

  char src_call[7];
  for(uint8_t i=0; i<6; i++) {
    src_call[i] = buffer[i+7] >> 1;
  }
  src_call[6] = 0;

  uint8_t src_ssid = (buffer[13] >> 1) & 0xf;

  Serial.println("###");
  Serial.print(src_call);
  Serial.print("-");
  Serial.print(src_ssid);
  Serial.print(" > ");
  Serial.print(dst_call);
  Serial.print("-");
  Serial.println(dst_ssid);
  Serial.println("###");

*/
  detachInterrupt(CC1101Interrupt);


  CCPACKET packet;

  /*char message[50];
  uint8_t n = sprintf(message, "About to send: %d bytes", len);
  memset((char *)packet.data, '\0', 64);
  memcpy((char *)packet.data, message, n);
  packet.length = n + 1;
  radio.sendData(packet);
*/
  //debug("Sent packet...");

  //memcpy((char *)packet.data, buffer, len);
  //packet.length = len + 1;
  //radio.sendData(packet);

  for(uint8_t i=0; i<len; i+=20) {
    size_t n = min(20, len-i);
    memset((char *)packet.data, '\0', 64);
    memcpy((char *)packet.data, buffer + i, n);
    packet.length = n + 1;
    radio.sendData(packet);
    delay(100);

    //const char *message = "on_ax25_packet";
    //CCPACKET packet;
    // We also need to include the 0 byte at the end of the string
    //packet.length = strlen(message)  + 1;
    //strncpy((char *) packet.data, message, packet.length);

    //radio.sendData(packet);
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
    debug("Writing data to TNC");
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
      debug("!!!! Buffer overrun !!!!");
    }
  }
}

void setup() {
  radio.init();
  radio.setSyncWord(syncWord);
  radio.setCarrierFreq(CFREQ_433);
  radio.disableAddressCheck();
  radio.setTxPowerAmp(PA_LongDistance);

  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(on_i2c_read_request);
  Wire.onReceive(on_i2c_write_receive);
  attachInterrupt(CC1101Interrupt, messageReceived, FALLING);

  Serial.begin(9600);
  debug("CC1101_PARTNUM ");
  debug(radio.readReg(CC1101_PARTNUM, CC1101_STATUS_REGISTER));
  debug("CC1101_VERSION ");
  debug(radio.readReg(CC1101_VERSION, CC1101_STATUS_REGISTER));
  debug("CC1101_MARCSTATE ");
  debug(radio.readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1f);

  debug("CC1101 radio initialized.");
  debug("TNC Begin");
}

void loop() {
  // Check incoming RF packets
  if (packetWaiting) {
    detachInterrupt(CC1101Interrupt);
    packetWaiting = false;
    CCPACKET packet;
    if (radio.receiveData(&packet) > 0) {
      /*
      Serial.println(F("Received packet..."));
      if (!packet.crc_ok) {
        Serial.println(F("crc not ok"));
      }
      Serial.print(F("lqi: "));
      Serial.println(lqi(packet.lqi));
      Serial.print(F("rssi: "));
      Serial.print(rssi(packet.rssi));
      Serial.println(F("dBm"));
      */
      if (packet.crc_ok && packet.length > 0) {
        //Serial.print(F("packet: len "));
        //Serial.println(packet.length);
        //Serial.println(F("data: "));
        //Serial.println((const char *) packet.data);

        // Send this packet out over Serial with KISS framing
        serial_kiss_wrapper(packet.data, packet.length);
      }
    }
    attachInterrupt(CC1101Interrupt, messageReceived, FALLING);
  }

  // Check i2c buffer
  uint8_t n = i2c_input_buffer.size();
  if(n > 0) {
    //Serial.print("Input buffer has ");
    //Serial.print(n);
    //Serial.println(" bytes");
    for(uint8_t i=0; i<n; i++) {
      poll_kiss(i2c_input_buffer.shift(), &kissCtx);
    }
  }

  // Read KISS framed packets from Serial
  while(Serial.available() > 0) {
    poll_kiss(Serial.read(), &kissCtx);
  }
}
