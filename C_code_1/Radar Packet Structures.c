#include <stdint.h>
#include <math.h>
#include <string.h>

#define MMWDEMO_UART_MSG_DETECTED_POINTS 1
#define MAGIC_WORD_LEN 8

// Magic word: 02 01 04 03 06 05 08 07
static const uint8_t magic_word[MAGIC_WORD_LEN] =
     {0x02,0x01,0x04,0x03,0x06,0x05,0x08,0x07};

typedef struct {
    float x;
    float y;
    float z;
    float v;
    float range;
    float azimuth;
    float elev;
    uint16_t snr;
    uint16_t noise;
} radar_obj_t;

// Parsed frame output
typedef struct {
    uint32_t frame_number;
    uint32_t det_count;
    radar_obj_t objects[256];   // Adjust as needed
} radar_frame_t;
