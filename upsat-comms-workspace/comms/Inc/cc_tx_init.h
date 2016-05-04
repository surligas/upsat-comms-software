#ifndef __CC_TX_INIT_H
#define __CC_TX_INIT_H

#include "cc_definitions.h"
#include "cc112x_spi.h"
#include <stdint.h>

void manualCalibrationtx();
void manualCalibrationrx() ;

void rx_registerConfig();
void tx_registerConfig();

void cc_Tx_INIT();

#endif
