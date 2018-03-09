#include <Arduino.h>

#define HDLC_I_FRAME 0x00
#define HDLC_S_FRAME 0x01
#define HDLC_U_FRAME 0x03

#define HDLC_S_TYPE_RR 0x00
#define HDLC_S_TYPE_REJ 0x01

#define HDLC_U_TYPE_UI 0x00
#define HDLC_U_TYPE_DM 0x03
#define HDLC_U_TYPE_SABM 0x07
#define HDLC_U_TYPE_DISC 0x08
#define HDLC_U_TYPE_UA 0x0C
#define HDLC_U_TYPE_FRMR 0x11

#define HDLC_POLL 0x10
#define HDLC_FINAL 0x00

#define HDLC_MODE_BEST_EFFORT 0x00
#define HDLC_MODE_RELIABLE 0x01

typedef struct HDLC {
  uint8_t address;
  uint8_t control;
  uint8_t data[64];
  uint8_t data_length;
  bool ack = true; // prevent retries of initial "null" frame
  bool do_retry = false;
  unsigned long time_sent = 0;
  uint8_t send_attempts = 0;
  bool failed = false;
} HDLC;

uint8_t hdlc_get_frame_type(HDLC * hdlc) {
  if((hdlc->control & 0x03) == 0x03) {
    return HDLC_U_FRAME;
  } else if((hdlc->control & 0x03) == 0x01) {
    return HDLC_S_FRAME;
  } else {
    return HDLC_I_FRAME;
  }
}

uint8_t hdlc_get_s_frame_type(HDLC * hdlc) {
  return (hdlc->control >> 2) & 0x03;
}

uint8_t hdlc_get_s_frame_recv_seq(HDLC * hdlc) {
  return (hdlc->control >> 5) & 0x07;
}

uint8_t hdlc_get_i_frame_recv_seq(HDLC * hdlc) {
  return (hdlc->control >> 5) & 0x07;
}

uint8_t hdlc_get_i_frame_send_seq(HDLC * hdlc) {
  return (hdlc->control >> 1) & 0x07;
}

uint8_t hdlc_get_u_frame_type(HDLC * hdlc) {
  // bits:      7  6  5     4     3   2       1   0
  // fields:  type upper | p/f | type lower | 1 | 1
  return ((hdlc->control >> 3) & 0x1C) | ((hdlc->control >> 2) & 0x03);
}

void hdlc_new_ack_frame(HDLC * hdlc, uint8_t seq) {
  hdlc->address = 0xFF;
  // RECV SEQ | P/F | TYPE | S
  hdlc->control = ((seq << 5) & 0xE0) | HDLC_FINAL | (HDLC_S_TYPE_RR << 2) | HDLC_S_FRAME;
  hdlc->data_length = 0;
  hdlc->ack = false;
  hdlc->time_sent = 0;
  hdlc->send_attempts = 0;
  hdlc->do_retry = false;
}

void hdlc_new_nack_frame(HDLC * hdlc, uint8_t seq) {
  hdlc->address = 0xFF;
  // RECV SEQ | P/F | TYPE | S
  hdlc->control = ((seq << 5) & 0xE0) | HDLC_FINAL | (HDLC_S_TYPE_REJ << 2) | HDLC_S_FRAME;
  hdlc->data_length = 0;
  hdlc->ack = false;
  hdlc->time_sent = 0;
  hdlc->send_attempts = 0;
  hdlc->do_retry = false;
}

void hdlc_new_data_frame(HDLC * hdlc, uint8_t data[], uint8_t length, uint8_t seq) {
  hdlc->address = 0xFF;
  // type upper | P/F | type lower | 1 1
  hdlc->control = 0x00 | HDLC_FINAL | ((seq << 1) & 0x0E) | HDLC_I_FRAME;
  hdlc->data_length = length;
  memcpy(hdlc->data, data, length);
  hdlc->ack = false;
  hdlc->time_sent = 0;
  hdlc->send_attempts = 0;
  hdlc->do_retry = false;
}

void hdlc_new_ui_frame(HDLC * hdlc, uint8_t data[], uint8_t length) {
  hdlc->address = 0xFF;
  // assuming final bit set, control frame is all zeros except frame identifier
  hdlc->control = HDLC_U_FRAME;
  hdlc->data_length = length;
  memcpy(hdlc->data, data, length);
  hdlc->ack = true; // disable downstream flow control
  hdlc->time_sent = 0;
  hdlc->send_attempts = 0;
  hdlc->do_retry = false;
}
