#include "envsensing.h"

uint8_t envSensingDesc[ES_MEAS_DESCR_SIZE] = {
    // Flags - reserved
    0x00, 0x00,
    // Samlpling function - instantaneous
    0x01,
    // Measurement period - Not in use
    0x00, 0x00, 0x00,
    // Internal update interval - Not in use
    0x00, 0x00, 0x00,
    // Application - air
    0x01,
    // Uncertainty
    0x00,
}; 


