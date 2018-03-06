#include <stdio.h>

typedef struct AX25Call {
    char call[7];
    uint8_t ssid;
} AX25Call;

typedef struct AX25Msg {
    AX25Call src;
    AX25Call dst;
    AX25Call rpt_list[8];
    uint8_t  rpt_count;
    uint8_t  rpt_flags;
    uint16_t ctrl;
    uint8_t  pid;
    const uint8_t *info;
    size_t len;
} AX25Msg;
