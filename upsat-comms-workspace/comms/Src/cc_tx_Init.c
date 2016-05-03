/*# Modulation format = 2-FSK 
# Whitening = true 
# Bit rate = 9.6 
# Device address = 0 
# Packet length = 3 
# Performance mode = High Performance 
# Packet bit length = 0 
# PA ramping = true 
# Carrier frequency = 435.800049 
# Deviation = 3.997803 
# Manchester enable = false 
# Address config = No address check 
# TX power = 15 
# Packet length mode = Fixed 
# Symbol rate = 9.6 
# RX filter BW = 25.000000 
*/
//
// Rf settings for CC1120
//
#include <cc_tx_init.h>

typedef struct
{
  unsigned int   addr;
  unsigned short dat;
}registerSetting_t;


static const registerSetting_t preferredSettings[]= 
{
  {IOCFG3,             0xB0},
  {IOCFG2,             0x06},
  {IOCFG1,             0xB0},
  {IOCFG0,             0x40},
  {SYNC3,              0x00},
  {SYNC2,              0x00},
  {SYNC1,              0x7A},
  {SYNC0,              0x0E},
  {SYNC_CFG1,          0x0B},
  {SYNC_CFG0,          0x0B},
  {DCFILT_CFG,         0x1C},
  {PREAMBLE_CFG1,      0x22},
  {IQIC,               0xC6},
  {CHAN_BW,            0x08},
  {MDMCFG0,            0x05},
  {SYMBOL_RATE2,       0x73},
  {AGC_REF,            0x20},
  {AGC_CS_THR,         0x19},
  {AGC_CFG1,           0xA9},
  {FIFO_CFG,           0x78},
  {SETTLING_CFG,       0x03},
  {FS_CFG,             0x14},
  //{PKT_CFG0,           0x28},
  {PKT_CFG0,           0x20},
  //{PA_CFG2,            0x2D},
  {PA_CFG2,            0x22},
  {PA_CFG0,            0x7D},
  {PKT_LEN,            0xFF},
  {IF_MIX_CFG,         0x00},
  {FREQOFF_CFG,        0x22},
  {FREQ2,              0x6C},
  {FREQ1,              0xF3},
  {FREQ0,              0x54},
  {FS_DIG1,            0x00},
  {FS_DIG0,            0x5F},
  {FS_CAL1,            0x40},
  {FS_CAL0,            0x0E},
  {FS_DIVTWO,          0x03},
  {FS_DSM0,            0x33},
  {FS_DVC0,            0x17},
  {FS_PFD,             0x50},
  {FS_PRE,             0x6E},
  {FS_REG_DIV_CML,     0x14},
  {FS_SPARE,           0xAC},
  {FS_VCO4,            0x13},
  {FS_VCO1,            0xAC},
  {FS_VCO0,            0xB4},
  {XOSC5,              0x0E},
  {XOSC1,              0x03},
  {DCFILTOFFSET_I1,    0xF8},
  {DCFILTOFFSET_I0,    0x39},
  {DCFILTOFFSET_Q1,    0x0E},
  {DCFILTOFFSET_Q0,    0x9B},
  {IQIE_I1,            0xEF},
  {IQIE_I0,            0xDE},
  {IQIE_Q1,            0x02},
  {IQIE_Q0,            0x2F},
  {AGC_GAIN1,          0x13},
  {SERIAL_STATUS,      0x10},
};

void registerConfig() {
  unsigned char writeByte;
  unsigned i;
  // Reset radio
  cc_tx_cmd(SRES);

  // Write registers to radio
  for(i = 0; i < (sizeof(preferredSettings)/sizeof(registerSetting_t)); i++) {
    writeByte = preferredSettings[i].dat;
    cc_tx_writeReg(preferredSettings[i].addr, writeByte);
  }
}

#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX 1
#define FS_CHP_INDEX 2

void manualCalibration() {

    uint8_t original_fs_cal2;
    uint8_t calResults_for_vcdac_start_high[3];
    uint8_t calResults_for_vcdac_start_mid[3];
    uint8_t marcstate;
    uint8_t writeByte;

    // 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
    writeByte = 0x00;
    cc_tx_writeReg(FS_VCO2, writeByte);

    // 2) Start with high VCDAC (original VCDAC_START + 2):
    cc_tx_readReg(FS_CAL2, &original_fs_cal2);
    writeByte = original_fs_cal2 + VCDAC_START_OFFSET;
    cc_tx_writeReg(FS_CAL2, writeByte);

    // 3) Calibrate and wait for calibration to be done
    //   (radio back in IDLE state)
    cc_tx_cmd(SCAL);

    do {
        cc_tx_readReg(MARCSTATE, &marcstate);
    } while (marcstate != 0x41);

    // 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with
    //    high VCDAC_START value
    cc_tx_readReg(FS_VCO2, &calResults_for_vcdac_start_high[FS_VCO2_INDEX]);
    cc_tx_readReg(FS_VCO4,&calResults_for_vcdac_start_high[FS_VCO4_INDEX]);
    cc_tx_readReg(FS_CHP, &calResults_for_vcdac_start_high[FS_CHP_INDEX]);

    // 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
    writeByte = 0x00;
    cc_tx_writeReg(FS_VCO2, writeByte);

    // 6) Continue with mid VCDAC (original VCDAC_START):
    writeByte = original_fs_cal2;
    cc_tx_writeReg(FS_CAL2, writeByte);

    // 7) Calibrate and wait for calibration to be done
    //   (radio back in IDLE state)
    cc_tx_cmd(SCAL);

    do {
        cc_tx_readReg(MARCSTATE, &marcstate);
    } while (marcstate != 0x41);

    // 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained
    //    with mid VCDAC_START value
    cc_tx_readReg(FS_VCO2, &calResults_for_vcdac_start_mid[FS_VCO2_INDEX]);
    cc_tx_readReg(FS_VCO4, &calResults_for_vcdac_start_mid[FS_VCO4_INDEX]);
    cc_tx_readReg(FS_CHP, &calResults_for_vcdac_start_mid[FS_CHP_INDEX]);

    // 9) Write back highest FS_VCO2 and corresponding FS_VCO
    //    and FS_CHP result
    if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] >
        calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) {
        writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
        cc_tx_writeReg(FS_VCO2, writeByte);
        writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
        cc_tx_writeReg(FS_VCO4, writeByte);
        writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
        cc_tx_writeReg(FS_CHP, writeByte);
    } else {
        writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
        cc_tx_writeReg(FS_VCO2, writeByte);
        writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
        cc_tx_writeReg(FS_VCO4, writeByte);
        writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
        cc_tx_writeReg(FS_CHP, writeByte);
    }
}

void manualCalibration_manos() {

  uint8_t orgnlfscal2;

  uint8_t Resultsvcdac_strtH0 = 0x00;
  uint8_t Resultsvcdac_strtH1 = 0x00;
  uint8_t Resultsvcdac_strtH2 = 0x00;

  uint8_t Resultsvcdac_strtM0 = 0x00;
  uint8_t Resultsvcdac_strtM1 = 0x00;
  uint8_t Resultsvcdac_strtM2 = 0x00;

  uint8_t writeByte3;

  /*1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)*/
  writeByte3 = 0x00;

  cc_tx_writeReg(FS_VCO2, writeByte3);


  /*2) Start with high VCDAC (original VCDAC_START + 2): IT IS A READ OP*/
  cc_tx_readReg(FS_CAL2, &orgnlfscal2);

  writeByte3 = orgnlfscal2 + 0x02;


  cc_tx_writeReg(FS_CAL2, writeByte3);

  /*3) Calibrate and wait for calibration to be done (radio back in IDLE state)*/
  cc_tx_cmd(SCAL);

  HAL_Delay(1);
  /*do {
	marcstate=SPI2ReadExtended(CC112X_MARCSTATE, 0x00);

  } while (marcstate != 0x41);*/

  /* 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with
  	   	   	   	   high VCDAC_START value
  */

  cc_tx_readReg(FS_CAL2, &Resultsvcdac_strtH0);
  cc_tx_readReg(FS_VCO4, &Resultsvcdac_strtH1);
  cc_tx_readReg(FS_CHP, &Resultsvcdac_strtH2);
  /* 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)*/

  writeByte3 = 0x00;

  cc_tx_writeReg(FS_VCO2, writeByte3);


  /*6) Set VCO cap-array to 0 (FS_VCO2 = 0x00)*/
  writeByte3 = orgnlfscal2;

  cc_tx_writeReg(FS_CAL2, writeByte3);

  /*  7) Calibrate and wait for calibration to be done
  (radio back in IDLE state)*/

  cc_tx_cmd(SCAL);

  HAL_Delay(1);

  cc_tx_readReg(FS_CAL2, &Resultsvcdac_strtM0);
  cc_tx_readReg(FS_VCO4, &Resultsvcdac_strtM1);
  cc_tx_readReg(FS_CHP, &Resultsvcdac_strtM2);

  /*9) Write back highest FS_VCO2 and corresponding FS_VCO
  and FS_CHP result*/

  if(Resultsvcdac_strtH0 > Resultsvcdac_strtM0) {

	writeByte3 = Resultsvcdac_strtH0;
        cc_tx_writeReg(FS_VCO2, writeByte3);
        writeByte3 = Resultsvcdac_strtH1;
        cc_tx_writeReg(FS_VCO4, writeByte3);
        writeByte3 = Resultsvcdac_strtH2;
        cc_tx_writeReg(FS_CHP, writeByte3);
  } else {
	writeByte3 = Resultsvcdac_strtM0;
	cc_tx_writeReg(FS_VCO2, writeByte3);
	writeByte3 = Resultsvcdac_strtM1;
	cc_tx_writeReg(FS_VCO4, writeByte3);
	writeByte3 = Resultsvcdac_strtM2;
	cc_tx_writeReg(FS_CHP, writeByte3);
  }
}

void cc_Tx_INIT() {
//cc_tx_writeReg(IOCFG3,0xB0);          //GPIO3 IO Pin Configuration
//cc_tx_writeReg(IOCFG2,0x06);          //GPIO2 IO Pin Configuration
//cc_tx_writeReg(IOCFG1,0xB0);          //GPIO1 IO Pin Configuration
//cc_tx_writeReg(IOCFG0,0x40);          //GPIO0 IO Pin Configuration
//cc_tx_writeReg(SYNC3,0x00);           //Sync Word Configuration [31:24]
//cc_tx_writeReg(SYNC2,0x00);           //Sync Word Configuration [23:16]
//cc_tx_writeReg(SYNC1,0x7A);           //Sync Word Configuration [15:8]
//cc_tx_writeReg(SYNC0,0x0E);           //Sync Word Configuration [7:0]
//cc_tx_writeReg(SYNC_CFG1,0x0B);       //Sync Word Detection Configuration Reg. 1
//cc_tx_writeReg(SYNC_CFG0,0x0B);       //Sync Word Length Configuration Reg. 0
//cc_tx_writeReg(DEVIATION_M,0x06);     //Frequency Deviation Configuration
//cc_tx_writeReg(MODCFG_DEV_E,0x03);    //Modulation Format and Frequency Deviation Configur..
//cc_tx_writeReg(DCFILT_CFG,0x1C);      //Digital DC Removal Configuration
//cc_tx_writeReg(PREAMBLE_CFG1,0x22);   //Preamble Length Configuration Reg. 1
//cc_tx_writeReg(PREAMBLE_CFG0,0x2A);   //Preamble Detection Configuration Reg. 0
//cc_tx_writeReg(FREQ_IF_CFG,0x40);     //RX Mixer Frequency Configuration
//cc_tx_writeReg(IQIC,0xC6);            //Digital Image Channel Compensation Configuration
//cc_tx_writeReg(CHAN_BW,0x08);         //Channel Filter Configuration
//cc_tx_writeReg(MDMCFG1,0x46);         //General Modem Parameter Configuration Reg. 1
//cc_tx_writeReg(MDMCFG0,0x05);         //General Modem Parameter Configuration Reg. 0
//cc_tx_writeReg(SYMBOL_RATE2,0x73);    //Symbol Rate Configuration Exponent and Mantissa [1..
//cc_tx_writeReg(SYMBOL_RATE1,0xA9);    //Symbol Rate Configuration Mantissa [15:8]
//cc_tx_writeReg(SYMBOL_RATE0,0x2A);    //Symbol Rate Configuration Mantissa [7:0]
//cc_tx_writeReg(AGC_REF,0x20);         //AGC Reference Level Configuration
//cc_tx_writeReg(AGC_CS_THR,0x19);      //Carrier Sense Threshold Configuration
//cc_tx_writeReg(AGC_GAIN_ADJUST,0x00); //RSSI Offset Configuration
//cc_tx_writeReg(AGC_CFG3,0x91);        //Automatic Gain Control Configuration Reg. 3
//cc_tx_writeReg(AGC_CFG2,0x20);        //Automatic Gain Control Configuration Reg. 2
//cc_tx_writeReg(AGC_CFG1,0xA9);        //Automatic Gain Control Configuration Reg. 1
//cc_tx_writeReg(AGC_CFG0,0xC3);        //Automatic Gain Control Configuration Reg. 0
//cc_tx_writeReg(FIFO_CFG,0x78);        //FIFO Configuration
//cc_tx_writeReg(DEV_ADDR,0x00);        //Device Address Configuration
//cc_tx_writeReg(SETTLING_CFG,0x03);    //Frequency Synthesizer Calibration and Settling Con..
//cc_tx_writeReg(FS_CFG,0x14);          //Frequency Synthesizer Configuration
//cc_tx_writeReg(WOR_CFG1,0x08);        //eWOR Configuration Reg. 1
//cc_tx_writeReg(WOR_CFG0,0x21);        //eWOR Configuration Reg. 0
//cc_tx_writeReg(WOR_EVENT0_MSB,0x00);  //Event 0 Configuration MSB
//cc_tx_writeReg(WOR_EVENT0_LSB,0x00);  //Event 0 Configuration LSB
//cc_tx_writeReg(PKT_CFG2,0x04);        //Packet Configuration Reg. 2
//cc_tx_writeReg(PKT_CFG1,0x05);        //Packet Configuration Reg. 1
//cc_tx_writeReg(PKT_CFG0,0x28);        //Packet Configuration Reg. 0
//cc_tx_writeReg(RFEND_CFG1,0x0F);      //RFEND Configuration Reg. 1
//cc_tx_writeReg(RFEND_CFG0,0x00);      //RFEND Configuration Reg. 0
//cc_tx_writeReg(PA_CFG2,0x45);         //Power Amplifier Configuration Reg. 2
//cc_tx_writeReg(PA_CFG1,0x56);         //Power Amplifier Configuration Reg. 1
//cc_tx_writeReg(PA_CFG0,0x7D);         //Power Amplifier Configuration Reg. 0
//cc_tx_writeReg(PKT_LEN,0xFF);         //Packet Length Configuration
//cc_tx_writeReg(IF_MIX_CFG,0x00);      //IF Mix Configuration
//cc_tx_writeReg(FREQOFF_CFG,0x22);     //Frequency Offset Correction Configuration
//cc_tx_writeReg(TOC_CFG,0x0B);         //Timing Offset Correction Configuration
//cc_tx_writeReg(MARC_SPARE,0x00);      //MARC Spare
//cc_tx_writeReg(ECG_CFG,0x00);         //External Clock Frequency Configuration
//cc_tx_writeReg(CFM_DATA_CFG,0x00);    //Custom frequency modulation enable
//cc_tx_writeReg(EXT_CTRL,0x01);        //External Control Configuration
//cc_tx_writeReg(RCCAL_FINE,0x00);      //RC Oscillator Calibration Fine
//cc_tx_writeReg(RCCAL_COARSE,0x00);    //RC Oscillator Calibration Coarse
//cc_tx_writeReg(RCCAL_OFFSET,0x00);    //RC Oscillator Calibration Clock Offset
//cc_tx_writeReg(FREQOFF1,0x00);        //Frequency Offset MSB
//cc_tx_writeReg(FREQOFF0,0x00);        //Frequency Offset LSB
//cc_tx_writeReg(FREQ2,0x6C);           //Frequency Configuration [23:16]
//cc_tx_writeReg(FREQ1,0xF3);           //Frequency Configuration [15:8]
//cc_tx_writeReg(FREQ0,0x54);           //Frequency Configuration [7:0]
//cc_tx_writeReg(IF_ADC2,0x02);         //Analog to Digital Converter Configuration Reg. 2
//cc_tx_writeReg(IF_ADC1,0xA6);         //Analog to Digital Converter Configuration Reg. 1
//cc_tx_writeReg(IF_ADC0,0x04);         //Analog to Digital Converter Configuration Reg. 0
//cc_tx_writeReg(FS_DIG1,0x00);         //Frequency Synthesizer Digital Reg. 1
//cc_tx_writeReg(FS_DIG0,0x5F);         //Frequency Synthesizer Digital Reg. 0
//cc_tx_writeReg(FS_CAL3,0x00);         //Frequency Synthesizer Calibration Reg. 3
//cc_tx_writeReg(FS_CAL2,0x20);         //Frequency Synthesizer Calibration Reg. 2
//cc_tx_writeReg(FS_CAL1,0x40);         //Frequency Synthesizer Calibration Reg. 1
//cc_tx_writeReg(FS_CAL0,0x0E);         //Frequency Synthesizer Calibration Reg. 0
//cc_tx_writeReg(FS_CHP,0x28);          //Frequency Synthesizer Charge Pump Configuration
//cc_tx_writeReg(FS_DIVTWO,0x03);       //Frequency Synthesizer Divide by 2
//cc_tx_writeReg(FS_DSM1,0x00);         //FS Digital Synthesizer Module Configuration Reg. 1
//cc_tx_writeReg(FS_DSM0,0x33);         //FS Digital Synthesizer Module Configuration Reg. 0
//cc_tx_writeReg(FS_DVC1,0xFF);         //Frequency Synthesizer Divider Chain Configuration ..
//cc_tx_writeReg(FS_DVC0,0x17);         //Frequency Synthesizer Divider Chain Configuration ..
//cc_tx_writeReg(FS_LBI,0x00);          //Frequency Synthesizer Local Bias Configuration
//cc_tx_writeReg(FS_PFD,0x50);          //Frequency Synthesizer Phase Frequency Detector Con..
//cc_tx_writeReg(FS_PRE,0x6E);          //Frequency Synthesizer Prescaler Configuration
//cc_tx_writeReg(FS_REG_DIV_CML,0x14);  //Frequency Synthesizer Divider Regulator Configurat..
//cc_tx_writeReg(FS_SPARE,0xAC);        //Frequency Synthesizer Spare
//cc_tx_writeReg(FS_VCO4,0x13);         //FS Voltage Controlled Oscillator Configuration Reg..
//cc_tx_writeReg(FS_VCO3,0x00);         //FS Voltage Controlled Oscillator Configuration Reg..
//cc_tx_writeReg(FS_VCO2,0x00);         //FS Voltage Controlled Oscillator Configuration Reg..
//cc_tx_writeReg(FS_VCO1,0xAC);         //FS Voltage Controlled Oscillator Configuration Reg..
//cc_tx_writeReg(FS_VCO0,0xB4);         //FS Voltage Controlled Oscillator Configuration Reg..
//cc_tx_writeReg(GBIAS6,0x00);          //Global Bias Configuration Reg. 6
//cc_tx_writeReg(GBIAS5,0x02);          //Global Bias Configuration Reg. 5
//cc_tx_writeReg(GBIAS4,0x00);          //Global Bias Configuration Reg. 4
//cc_tx_writeReg(GBIAS3,0x00);          //Global Bias Configuration Reg. 3
//cc_tx_writeReg(GBIAS2,0x10);          //Global Bias Configuration Reg. 2
//cc_tx_writeReg(GBIAS1,0x00);          //Global Bias Configuration Reg. 1
//cc_tx_writeReg(GBIAS0,0x00);          //Global Bias Configuration Reg. 0
//cc_tx_writeReg(IFAMP,0x01);           //Intermediate Frequency Amplifier Configuration
//cc_tx_writeReg(LNA,0x01);             //Low Noise Amplifier Configuration
//cc_tx_writeReg(RXMIX,0x01);           //RX Mixer Configuration
//cc_tx_writeReg(XOSC5,0x0E);           //Crystal Oscillator Configuration Reg. 5
//cc_tx_writeReg(XOSC4,0xA0);           //Crystal Oscillator Configuration Reg. 4
//cc_tx_writeReg(XOSC3,0x03);           //Crystal Oscillator Configuration Reg. 3
//cc_tx_writeReg(XOSC2,0x04);           //Crystal Oscillator Configuration Reg. 2
//cc_tx_writeReg(XOSC1,0x03);           //Crystal Oscillator Configuration Reg. 1
//cc_tx_writeReg(XOSC0,0x00);           //Crystal Oscillator Configuration Reg. 0
//cc_tx_writeReg(ANALOG_SPARE,0x00);    //Analog Spare
//cc_tx_writeReg(PA_CFG3,0x00);         //Power Amplifier Configuration Reg. 3
//cc_tx_writeReg(WOR_TIME1,0x00);       //eWOR Timer Counter Value MSB
//cc_tx_writeReg(WOR_TIME0,0x8A);       //eWOR Timer Counter Value LSB
//cc_tx_writeReg(WOR_CAPTURE1,0x00);    //eWOR Timer Capture Value MSB
//cc_tx_writeReg(WOR_CAPTURE0,0x00);    //eWOR Timer Capture Value LSB
//cc_tx_writeReg(BIST,0x00);            //MARC Built-In Self-Test
//cc_tx_writeReg(DCFILTOFFSET_I1,0xF8); //DC Filter Offset I MSB
//cc_tx_writeReg(DCFILTOFFSET_I0,0x39); //DC Filter Offset I LSB
//cc_tx_writeReg(DCFILTOFFSET_Q1,0x0E); //DC Filter Offset Q MSB
//cc_tx_writeReg(DCFILTOFFSET_Q0,0x9B); //DC Filter Offset Q LSB
//cc_tx_writeReg(IQIE_I1,0xEF);         //IQ Imbalance Value I MSB
//cc_tx_writeReg(IQIE_I0,0xDE);         //IQ Imbalance Value I LSB
//cc_tx_writeReg(IQIE_Q1,0x02);         //IQ Imbalance Value Q MSB
//cc_tx_writeReg(IQIE_Q0,0x2F);         //IQ Imbalance Value Q LSB
//cc_tx_writeReg(RSSI1,0xF6);           //Received Signal Strength Indicator Reg. 1
//cc_tx_writeReg(RSSI0,0x33);           //Received Signal Strength Indicator Reg.0
//cc_tx_writeReg(MARCSTATE,0x41);       //MARC State
//cc_tx_writeReg(LQI_VAL,0x80);         //Link Quality Indicator Value
//cc_tx_writeReg(PQT_SYNC_ERR,0xFF);    //Preamble and Sync Word Error
//cc_tx_writeReg(DEM_STATUS,0x01);      //Demodulator Status
//cc_tx_writeReg(FREQOFF_EST1,0xFF);    //Frequency Offset Estimate MSB
//cc_tx_writeReg(FREQOFF_EST0,0xE7);    //Frequency Offset Estimate LSB
//cc_tx_writeReg(AGC_GAIN3,0x27);       //Automatic Gain Control Reg. 3
//cc_tx_writeReg(AGC_GAIN2,0xD1);       //Automatic Gain Control Reg. 2
//cc_tx_writeReg(AGC_GAIN1,0x13);       //Automatic Gain Control Reg. 1
//cc_tx_writeReg(AGC_GAIN0,0x3F);       //Automatic Gain Control Reg. 0
//cc_tx_writeReg(CFM_RX_DATA_OUT,0x00); //Custom Frequency Modulation RX Data
//cc_tx_writeReg(CFM_TX_DATA_IN,0x00);  //Custom Frequency Modulation TX Data
//cc_tx_writeReg(ASK_SOFT_RX_DATA,0x30);//ASK Soft Decision Output
//cc_tx_writeReg(RNDGEN,0x7F);          //Random Number Generator Value
//cc_tx_writeReg(MAGN2,0x00);           //Signal Magnitude after CORDIC [16]
//cc_tx_writeReg(MAGN1,0x00);           //Signal Magnitude after CORDIC [15:8]
//cc_tx_writeReg(MAGN0,0x85);           //Signal Magnitude after CORDIC [7:0]
//cc_tx_writeReg(ANG1,0x01);            //Signal Angular after CORDIC [9:8]
//cc_tx_writeReg(ANG0,0xAC);            //Signal Angular after CORDIC [7:0]
//cc_tx_writeReg(CHFILT_I2,0x0F);       //Channel Filter Data Real Part [18:16]
//cc_tx_writeReg(CHFILT_I1,0xFF);       //Channel Filter Data Real Part [15:8]
//cc_tx_writeReg(CHFILT_I0,0x50);       //Channel Filter Data Real Part [7:0]
//cc_tx_writeReg(CHFILT_Q2,0x00);       //Channel Filter Data Imaginary Part [18:16]
//cc_tx_writeReg(CHFILT_Q1,0x00);       //Channel Filter Data Imaginary Part [15:8]
//cc_tx_writeReg(CHFILT_Q0,0x2D);       //Channel Filter Data Imaginary Part [7:0]
//cc_tx_writeReg(GPIO_STATUS,0x00);     //General Purpose Input/Output Status
//cc_tx_writeReg(FSCAL_CTRL,0x01);      //Frequency Synthesizer Calibration Control
//cc_tx_writeReg(PHASE_ADJUST,0x00);    //Frequency Synthesizer Phase Adjust
//cc_tx_writeReg(PARTNUMBER,0x48);      //Part Number
//cc_tx_writeReg(PARTVERSION,0x21);     //Part Revision
//cc_tx_writeReg(SERIAL_STATUS,0x10);   //Serial Status
//cc_tx_writeReg(MODEM_STATUS1,0x11);   //Modem Status Reg. 1
//cc_tx_writeReg(MODEM_STATUS0,0x00);   //Modem Status Reg. 0
//cc_tx_writeReg(MARC_STATUS1,0x00);    //MARC Status Reg. 1
//cc_tx_writeReg(MARC_STATUS0,0x00);    //MARC Status Reg. 0
//cc_tx_writeReg(PA_IFAMP_TEST,0x00);   //Power Amplifier Intermediate Frequency Amplifier T..
//cc_tx_writeReg(FSRF_TEST,0x00);       //Frequency Synthesizer Test
//cc_tx_writeReg(PRE_TEST,0x00);        //Frequency Synthesizer Prescaler Test
//cc_tx_writeReg(PRE_OVR,0x00);         //Frequency Synthesizer Prescaler Override
//cc_tx_writeReg(ADC_TEST,0x00);        //Analog to Digital Converter Test
//cc_tx_writeReg(DVC_TEST,0x0B);        //Digital Divider Chain Test
//cc_tx_writeReg(ATEST,0x40);           //Analog Test
//cc_tx_writeReg(ATEST_LVDS,0x00);      //Analog Test LVDS
//cc_tx_writeReg(ATEST_MODE,0x00);      //Analog Test Mode
//cc_tx_writeReg(XOSC_TEST1,0x3C);      //Crystal Oscillator Test Reg. 1
//cc_tx_writeReg(XOSC_TEST0,0x00);      //Crystal Oscillator Test Reg. 0
//cc_tx_writeReg(RXFIRST,0x00);         //RX FIFO Pointer First Entry
//cc_tx_writeReg(TXFIRST,0x00);         //TX FIFO Pointer First Entry
//cc_tx_writeReg(RXLAST,0x00);          //RX FIFO Pointer Last Entry
//cc_tx_writeReg(TXLAST,0x00);          //TX FIFO Pointer Last Entry
//cc_tx_writeReg(NUM_TXBYTES,0x00);     //TX FIFO Status
//cc_tx_writeReg(NUM_RXBYTES,0x00);     //RX FIFO Status
//cc_tx_writeReg(FIFO_NUM_TXBYTES,0x0F);//TX FIFO Status
//cc_tx_writeReg(FIFO_NUM_RXBYTES,0x00);//RX FIFO Status
//  
//
//  
//    cc_tx_writeReg(IOCFG3,0x06);          //GPIO3 IO Pin Configuration
//    cc_tx_writeReg(IOCFG2,0x07);          //GPIO2 IO Pin Configuration
//    cc_tx_writeReg(IOCFG1,0x30);          //GPIO1 IO Pin Configuration
//    cc_tx_writeReg(IOCFG0,0x3C);          //GPIO0 IO Pin Configuration
//    //cc_tx_writeReg(SYNC3,0x93);           //Sync Word Configuration [31:24]
//    //cc_tx_writeReg(SYNC2,0x0B);           //Sync Word Configuration [23:16]
//    //cc_tx_writeReg(SYNC1,0x51);           //Sync Word Configuration [15:8]
//    //cc_tx_writeReg(SYNC0,0xDE);           //Sync Word Configuration [7:0]
//    //cc_tx_writeReg(SYNC_CFG1,0x0A);       //Sync Word Detection Configuration Reg. 1
//    //cc_tx_writeReg(SYNC_CFG0,0x17);       //Sync Word Length Configuration Reg. 0
//cc_tx_writeReg(SYNC3,0x00);           //Sync Word Configuration [31:24]
//cc_tx_writeReg(SYNC2,0x00);           //Sync Word Configuration [23:16]
//cc_tx_writeReg(SYNC1,0x7A);           //Sync Word Configuration [15:8]
//cc_tx_writeReg(SYNC0,0x0E);           //Sync Word Configuration [7:0]
//cc_tx_writeReg(SYNC_CFG1,0x0B);       //Sync Word Detection Configuration Reg. 1
//cc_tx_writeReg(SYNC_CFG0,0x0B);       //Sync Word Length Configuration Reg. 0
//    cc_tx_writeReg(DEVIATION_M,0x06);     //Frequency Deviation Configuration
//    cc_tx_writeReg(MODCFG_DEV_E,0x03);    //Modulation Format and Frequency Deviation Configur..
//    cc_tx_writeReg(DCFILT_CFG,0x4C);      //Digital DC Removal Configuration
//    //cc_tx_writeReg(PREAMBLE_CFG1,0x14);   //Preamble Length Configuration Reg. 1
//    //cc_tx_writeReg(PREAMBLE_CFG0,0x2A);   //Preamble Detection Configuration Reg. 0
//cc_tx_writeReg(PREAMBLE_CFG1,0x22);   //Preamble Length Configuration Reg. 1
//cc_tx_writeReg(PREAMBLE_CFG0,0x2A); 
//    cc_tx_writeReg(FREQ_IF_CFG,0x40);     //RX Mixer Frequency Configuration
//    cc_tx_writeReg(IQIC,0xC4);            //Digital Image Channel Compensation Configuration
//    cc_tx_writeReg(CHAN_BW,0x08);         //Channel Filter Configuration
//    cc_tx_writeReg(MDMCFG1,0x46);         //General Modem Parameter Configuration Reg. 1
//    cc_tx_writeReg(MDMCFG0,0x0D);         //General Modem Parameter Configuration Reg. 0
//    cc_tx_writeReg(SYMBOL_RATE2,0x73);    //Symbol Rate Configuration Exponent and Mantissa [1..
//    cc_tx_writeReg(SYMBOL_RATE1,0xA9);    //Symbol Rate Configuration Mantissa [15:8]
//    cc_tx_writeReg(SYMBOL_RATE0,0x2A);    //Symbol Rate Configuration Mantissa [7:0]
//    cc_tx_writeReg(AGC_REF,0x36);         //AGC Reference Level Configuration
//    cc_tx_writeReg(AGC_CS_THR,0x00);      //Carrier Sense Threshold Configuration
//    cc_tx_writeReg(AGC_GAIN_ADJUST,0x00); //RSSI Offset Configuration
//    cc_tx_writeReg(AGC_CFG3,0x91);        //Automatic Gain Control Configuration Reg. 3
//    cc_tx_writeReg(AGC_CFG2,0x20);        //Automatic Gain Control Configuration Reg. 2
//    cc_tx_writeReg(AGC_CFG1,0xAA);        //Automatic Gain Control Configuration Reg. 1
//    cc_tx_writeReg(AGC_CFG0,0xC3);        //Automatic Gain Control Configuration Reg. 0
//    cc_tx_writeReg(FIFO_CFG,0x80);        //FIFO Configuration
//    cc_tx_writeReg(DEV_ADDR,0x00);        //Device Address Configuration
//    cc_tx_writeReg(SETTLING_CFG,0x0B);    //Frequency Synthesizer Calibration and Settling Con..
//    cc_tx_writeReg(FS_CFG,0x04);          //Frequency Synthesizer Configuration
//    cc_tx_writeReg(WOR_CFG1,0x08);        //eWOR Configuration Reg. 1
//    cc_tx_writeReg(WOR_CFG0,0x21);        //eWOR Configuration Reg. 0
//    cc_tx_writeReg(WOR_EVENT0_MSB,0x00);  //Event 0 Configuration MSB
//    cc_tx_writeReg(WOR_EVENT0_LSB,0x00);  //Event 0 Configuration LSB
//    //cc_tx_writeReg(PKT_CFG2,0x04);        //Packet Configuration Reg. 2
//    //cc_tx_writeReg(PKT_CFG1,0x05);        //Packet Configuration Reg. 1
//    //cc_tx_writeReg(PKT_CFG0,0x20);        //Packet Configuration Reg. 0
//    
//cc_tx_writeReg(PKT_CFG2,0x04);        //Packet Configuration Reg. 2
//cc_tx_writeReg(PKT_CFG1,0x05);        //Packet Configuration Reg. 1
////cc_tx_writeReg(PKT_CFG0,0x28);        //Packet Configuration Reg. 0
//cc_tx_writeReg(PKT_CFG0,0x00);        //Packet Configuration Reg. 0
//
//    cc_tx_writeReg(RFEND_CFG1,0x0F);      //RFEND Configuration Reg. 1
//    cc_tx_writeReg(RFEND_CFG0,0x00);      //RFEND Configuration Reg. 0
//    cc_tx_writeReg(PA_CFG2,0x45);         //Power Amplifier Configuration Reg. 2
//    cc_tx_writeReg(PA_CFG1,0x56);         //Power Amplifier Configuration Reg. 1
//    cc_tx_writeReg(PA_CFG0,0x7D);         //Power Amplifier Configuration Reg. 0
//    cc_tx_writeReg(PKT_LEN,0xFF);         //Packet Length Configuration
//    cc_tx_writeReg(IF_MIX_CFG,0x04);      //IF Mix Configuration
//    cc_tx_writeReg(FREQOFF_CFG,0x20);     //Frequency Offset Correction Configuration
//    cc_tx_writeReg(TOC_CFG,0x0B);         //Timing Offset Correction Configuration
//    cc_tx_writeReg(MARC_SPARE,0x00);      //MARC Spare
//    cc_tx_writeReg(ECG_CFG,0x00);         //External Clock Frequency Configuration
//    cc_tx_writeReg(CFM_DATA_CFG,0x00);    //Custom frequency modulation enable
//    cc_tx_writeReg(EXT_CTRL,0x01);        //External Control Configuration
//    cc_tx_writeReg(RCCAL_FINE,0x00);      //RC Oscillator Calibration Fine
//    cc_tx_writeReg(RCCAL_COARSE,0x00);    //RC Oscillator Calibration Coarse
//    cc_tx_writeReg(RCCAL_OFFSET,0x00);    //RC Oscillator Calibration Clock Offset
//    cc_tx_writeReg(FREQOFF1,0x00);        //Frequency Offset MSB
//    cc_tx_writeReg(FREQOFF0,0x00);        //Frequency Offset LSB
//    cc_tx_writeReg(FREQ2,0x6C);           //Frequency Configuration [23:16]
//    cc_tx_writeReg(FREQ1,0xF3);           //Frequency Configuration [15:8]
//    cc_tx_writeReg(FREQ0,0x34);           //Frequency Configuration [7:0]
//    cc_tx_writeReg(IF_ADC2,0x02);         //Analog to Digital Converter Configuration Reg. 2
//    cc_tx_writeReg(IF_ADC1,0xA6);         //Analog to Digital Converter Configuration Reg. 1
//    cc_tx_writeReg(IF_ADC0,0x04);         //Analog to Digital Converter Configuration Reg. 0
//    cc_tx_writeReg(FS_DIG1,0x00);         //Frequency Synthesizer Digital Reg. 1
//    cc_tx_writeReg(FS_DIG0,0x5F);         //Frequency Synthesizer Digital Reg. 0
//    cc_tx_writeReg(FS_CAL3,0x00);         //Frequency Synthesizer Calibration Reg. 3
//    cc_tx_writeReg(FS_CAL2,0x20);         //Frequency Synthesizer Calibration Reg. 2
//    cc_tx_writeReg(FS_CAL1,0x40);         //Frequency Synthesizer Calibration Reg. 1
//    cc_tx_writeReg(FS_CAL0,0x0E);         //Frequency Synthesizer Calibration Reg. 0
//    cc_tx_writeReg(FS_CHP,0x28);          //Frequency Synthesizer Charge Pump Configuration
//    cc_tx_writeReg(FS_DIVTWO,0x03);       //Frequency Synthesizer Divide by 2
//    cc_tx_writeReg(FS_DSM1,0x00);         //FS Digital Synthesizer Module Configuration Reg. 1
//    cc_tx_writeReg(FS_DSM0,0x33);         //FS Digital Synthesizer Module Configuration Reg. 0
//    cc_tx_writeReg(FS_DVC1,0xFF);         //Frequency Synthesizer Divider Chain Configuration ..
//    cc_tx_writeReg(FS_DVC0,0x17);         //Frequency Synthesizer Divider Chain Configuration ..
//    cc_tx_writeReg(FS_LBI,0x00);          //Frequency Synthesizer Local Bias Configuration
//    cc_tx_writeReg(FS_PFD,0x50);          //Frequency Synthesizer Phase Frequency Detector Con..
//    cc_tx_writeReg(FS_PRE,0x6E);          //Frequency Synthesizer Prescaler Configuration
//    cc_tx_writeReg(FS_REG_DIV_CML,0x14);  //Frequency Synthesizer Divider Regulator Configurat..
//    cc_tx_writeReg(FS_SPARE,0xAC);        //Frequency Synthesizer Spare
//    cc_tx_writeReg(FS_VCO4,0x14);         //FS Voltage Controlled Oscillator Configuration Reg..
//    cc_tx_writeReg(FS_VCO3,0x00);         //FS Voltage Controlled Oscillator Configuration Reg..
//    cc_tx_writeReg(FS_VCO2,0x00);         //FS Voltage Controlled Oscillator Configuration Reg..
//    cc_tx_writeReg(FS_VCO1,0x00);         //FS Voltage Controlled Oscillator Configuration Reg..
//    cc_tx_writeReg(FS_VCO0,0xB4);         //FS Voltage Controlled Oscillator Configuration Reg..
//    cc_tx_writeReg(GBIAS6,0x00);          //Global Bias Configuration Reg. 6
//    cc_tx_writeReg(GBIAS5,0x02);          //Global Bias Configuration Reg. 5
//    cc_tx_writeReg(GBIAS4,0x00);          //Global Bias Configuration Reg. 4
//    cc_tx_writeReg(GBIAS3,0x00);          //Global Bias Configuration Reg. 3
//    cc_tx_writeReg(GBIAS2,0x10);          //Global Bias Configuration Reg. 2
//    cc_tx_writeReg(GBIAS1,0x00);          //Global Bias Configuration Reg. 1
//    cc_tx_writeReg(GBIAS0,0x00);          //Global Bias Configuration Reg. 0
//    cc_tx_writeReg(IFAMP,0x01);           //Intermediate Frequency Amplifier Configuration
//    cc_tx_writeReg(LNA,0x01);             //Low Noise Amplifier Configuration
//    cc_tx_writeReg(RXMIX,0x01);           //RX Mixer Configuration
//    cc_tx_writeReg(XOSC5,0x0E);           //Crystal Oscillator Configuration Reg. 5
//    cc_tx_writeReg(XOSC4,0xA0);           //Crystal Oscillator Configuration Reg. 4
//    cc_tx_writeReg(XOSC3,0x03);           //Crystal Oscillator Configuration Reg. 3
//    cc_tx_writeReg(XOSC2,0x04);           //Crystal Oscillator Configuration Reg. 2
//    cc_tx_writeReg(XOSC1,0x03);           //Crystal Oscillator Configuration Reg. 1
//    cc_tx_writeReg(XOSC0,0x00);           //Crystal Oscillator Configuration Reg. 0
//    cc_tx_writeReg(ANALOG_SPARE,0x00);    //Analog Spare
//    cc_tx_writeReg(PA_CFG3,0x00);         //Power Amplifier Configuration Reg. 3
//    cc_tx_writeReg(WOR_TIME1,0x00);       //eWOR Timer Counter Value MSB
//    cc_tx_writeReg(WOR_TIME0,0x00);       //eWOR Timer Counter Value LSB
//    cc_tx_writeReg(WOR_CAPTURE1,0x00);    //eWOR Timer Capture Value MSB
//    cc_tx_writeReg(WOR_CAPTURE0,0x00);    //eWOR Timer Capture Value LSB
//    cc_tx_writeReg(BIST,0x00);            //MARC Built-In Self-Test
//    cc_tx_writeReg(DCFILTOFFSET_I1,0x00); //DC Filter Offset I MSB
//    cc_tx_writeReg(DCFILTOFFSET_I0,0x00); //DC Filter Offset I LSB
//    cc_tx_writeReg(DCFILTOFFSET_Q1,0x00); //DC Filter Offset Q MSB
//    cc_tx_writeReg(DCFILTOFFSET_Q0,0x00); //DC Filter Offset Q LSB
//    cc_tx_writeReg(IQIE_I1,0x00);         //IQ Imbalance Value I MSB
//    cc_tx_writeReg(IQIE_I0,0x00);         //IQ Imbalance Value I LSB
//    cc_tx_writeReg(IQIE_Q1,0x00);         //IQ Imbalance Value Q MSB
//    cc_tx_writeReg(IQIE_Q0,0x00);         //IQ Imbalance Value Q LSB
//    cc_tx_writeReg(RSSI1,0x80);           //Received Signal Strength Indicator Reg. 1
//    cc_tx_writeReg(RSSI0,0x00);           //Received Signal Strength Indicator Reg.0
//    cc_tx_writeReg(MARCSTATE,0x41);       //MARC State
//    cc_tx_writeReg(LQI_VAL,0x00);         //Link Quality Indicator Value
//    cc_tx_writeReg(PQT_SYNC_ERR,0xFF);    //Preamble and Sync Word Error
//    cc_tx_writeReg(DEM_STATUS,0x00);      //Demodulator Status
//    cc_tx_writeReg(FREQOFF_EST1,0x00);    //Frequency Offset Estimate MSB
//    cc_tx_writeReg(FREQOFF_EST0,0x00);    //Frequency Offset Estimate LSB
//    cc_tx_writeReg(AGC_GAIN3,0x00);       //Automatic Gain Control Reg. 3
//    cc_tx_writeReg(AGC_GAIN2,0xD1);       //Automatic Gain Control Reg. 2
//    cc_tx_writeReg(AGC_GAIN1,0x00);       //Automatic Gain Control Reg. 1
//    cc_tx_writeReg(AGC_GAIN0,0x3F);       //Automatic Gain Control Reg. 0
//    cc_tx_writeReg(CFM_RX_DATA_OUT,0x00); //Custom Frequency Modulation RX Data
//    cc_tx_writeReg(CFM_TX_DATA_IN,0x00);  //Custom Frequency Modulation TX Data
//    cc_tx_writeReg(ASK_SOFT_RX_DATA,0x30);//ASK Soft Decision Output
//    cc_tx_writeReg(RNDGEN,0x7F);          //Random Number Generator Value
//    cc_tx_writeReg(MAGN2,0x00);           //Signal Magnitude after CORDIC [16]
//    cc_tx_writeReg(MAGN1,0x00);           //Signal Magnitude after CORDIC [15:8]
//    cc_tx_writeReg(MAGN0,0x00);           //Signal Magnitude after CORDIC [7:0]
//    cc_tx_writeReg(ANG1,0x00);            //Signal Angular after CORDIC [9:8]
//    cc_tx_writeReg(ANG0,0x00);            //Signal Angular after CORDIC [7:0]
//    cc_tx_writeReg(CHFILT_I2,0x08);       //Channel Filter Data Real Part [18:16]
//    cc_tx_writeReg(CHFILT_I1,0x00);       //Channel Filter Data Real Part [15:8]
//    cc_tx_writeReg(CHFILT_I0,0x00);       //Channel Filter Data Real Part [7:0]
//    cc_tx_writeReg(CHFILT_Q2,0x00);       //Channel Filter Data Imaginary Part [18:16]
//    cc_tx_writeReg(CHFILT_Q1,0x00);       //Channel Filter Data Imaginary Part [15:8]
//    cc_tx_writeReg(CHFILT_Q0,0x00);       //Channel Filter Data Imaginary Part [7:0]
//    cc_tx_writeReg(GPIO_STATUS,0x00);     //General Purpose Input/Output Status
//    cc_tx_writeReg(FSCAL_CTRL,0x01);      //Frequency Synthesizer Calibration Control
//    cc_tx_writeReg(PHASE_ADJUST,0x00);    //Frequency Synthesizer Phase Adjust
//    cc_tx_writeReg(PARTNUMBER,0x00);      //Part Number
//    cc_tx_writeReg(PARTVERSION,0x00);     //Part Revision
//    cc_tx_writeReg(SERIAL_STATUS,0x00);   //Serial Status
//    cc_tx_writeReg(MODEM_STATUS1,0x01);   //Modem Status Reg. 1
//    cc_tx_writeReg(MODEM_STATUS0,0x00);   //Modem Status Reg. 0
//    cc_tx_writeReg(MARC_STATUS1,0x00);    //MARC Status Reg. 1
//    cc_tx_writeReg(MARC_STATUS0,0x00);    //MARC Status Reg. 0
//    cc_tx_writeReg(PA_IFAMP_TEST,0x00);   //Power Amplifier Intermediate Frequency Amplifier T..
//    cc_tx_writeReg(FSRF_TEST,0x00);       //Frequency Synthesizer Test
//    cc_tx_writeReg(PRE_TEST,0x00);        //Frequency Synthesizer Prescaler Test
//    cc_tx_writeReg(PRE_OVR,0x00);         //Frequency Synthesizer Prescaler Override
//    cc_tx_writeReg(ADC_TEST,0x00);        //Analog to Digital Converter Test
//    cc_tx_writeReg(DVC_TEST,0x0B);        //Digital Divider Chain Test
//    cc_tx_writeReg(ATEST,0x40);           //Analog Test
//    cc_tx_writeReg(ATEST_LVDS,0x00);      //Analog Test LVDS
//    cc_tx_writeReg(ATEST_MODE,0x00);      //Analog Test Mode
//    cc_tx_writeReg(XOSC_TEST1,0x3C);      //Crystal Oscillator Test Reg. 1
//    cc_tx_writeReg(XOSC_TEST0,0x00);      //Crystal Oscillator Test Reg. 0
//    cc_tx_writeReg(RXFIRST,0x00);         //RX FIFO Pointer First Entry
//    cc_tx_writeReg(TXFIRST,0x00);         //TX FIFO Pointer First Entry
//    cc_tx_writeReg(RXLAST,0x00);          //RX FIFO Pointer Last Entry
//    cc_tx_writeReg(TXLAST,0x00);          //TX FIFO Pointer Last Entry
//    cc_tx_writeReg(NUM_TXBYTES,0x00);     //TX FIFO Status
//    cc_tx_writeReg(NUM_RXBYTES,0x00);     //RX FIFO Status
//    cc_tx_writeReg(FIFO_NUM_TXBYTES,0x0F);//TX FIFO Status
//    cc_tx_writeReg(FIFO_NUM_RXBYTES,0x00);//RX FIFO Status
// Packet length = 42 
// TX power = 2 
// Packet bit length = 0 
// Deviation = 3.997803 
// Address config = No address check 
// Symbol rate = 9.6 
// Performance mode = High Performance 
// Modulation format = 2-FSK 
// Manchester enable = false 
// Device address = 0 
// Whitening = false 
// Carrier frequency = 435.802002 
// Bit rate = 9.6 
// RX filter BW = 25.000000 
// PA ramping = false 
// Packet length mode = Variable 
//
// Rf settings for CC1120
//
cc_tx_writeReg(IOCFG3,0xB0);          //GPIO3 IO Pin Configuration
cc_tx_writeReg(IOCFG2,0x06);          //GPIO2 IO Pin Configuration
cc_tx_writeReg(IOCFG1,0xB0);          //GPIO1 IO Pin Configuration
cc_tx_writeReg(IOCFG0,0x40);          //GPIO0 IO Pin Configuration
cc_tx_writeReg(SYNC3,0x93);           //Sync Word Configuration [31:24]
cc_tx_writeReg(SYNC2,0x0B);           //Sync Word Configuration [23:16]
cc_tx_writeReg(SYNC1,0x51);           //Sync Word Configuration [15:8]
cc_tx_writeReg(SYNC0,0xDE);           //Sync Word Configuration [7:0]
cc_tx_writeReg(SYNC_CFG1,0x0B);       //Sync Word Detection Configuration Reg. 1
cc_tx_writeReg(SYNC_CFG0,0x0B);       //Sync Word Length Configuration Reg. 0
cc_tx_writeReg(DEVIATION_M,0x06);     //Frequency Deviation Configuration
cc_tx_writeReg(MODCFG_DEV_E,0x03);    //Modulation Format and Frequency Deviation Configur..
cc_tx_writeReg(DCFILT_CFG,0x1C);      //Digital DC Removal Configuration
cc_tx_writeReg(PREAMBLE_CFG1,0x18);   //Preamble Length Configuration Reg. 1
cc_tx_writeReg(PREAMBLE_CFG0,0x2A);   //Preamble Detection Configuration Reg. 0
cc_tx_writeReg(FREQ_IF_CFG,0x40);     //RX Mixer Frequency Configuration
cc_tx_writeReg(IQIC,0xC6);            //Digital Image Channel Compensation Configuration
cc_tx_writeReg(CHAN_BW,0x08);         //Channel Filter Configuration
cc_tx_writeReg(MDMCFG1,0x46);         //General Modem Parameter Configuration Reg. 1
cc_tx_writeReg(MDMCFG0,0x05);         //General Modem Parameter Configuration Reg. 0
cc_tx_writeReg(SYMBOL_RATE2,0x73);    //Symbol Rate Configuration Exponent and Mantissa [1..
cc_tx_writeReg(SYMBOL_RATE1,0xA9);    //Symbol Rate Configuration Mantissa [15:8]
cc_tx_writeReg(SYMBOL_RATE0,0x2A);    //Symbol Rate Configuration Mantissa [7:0]
cc_tx_writeReg(AGC_REF,0x20);         //AGC Reference Level Configuration
cc_tx_writeReg(AGC_CS_THR,0x19);      //Carrier Sense Threshold Configuration
cc_tx_writeReg(AGC_GAIN_ADJUST,0x00); //RSSI Offset Configuration
cc_tx_writeReg(AGC_CFG3,0x91);        //Automatic Gain Control Configuration Reg. 3
cc_tx_writeReg(AGC_CFG2,0x20);        //Automatic Gain Control Configuration Reg. 2
cc_tx_writeReg(AGC_CFG1,0xA9);        //Automatic Gain Control Configuration Reg. 1
cc_tx_writeReg(AGC_CFG0,0xC3);        //Automatic Gain Control Configuration Reg. 0
cc_tx_writeReg(FIFO_CFG,0x78);        //FIFO Configuration
cc_tx_writeReg(DEV_ADDR,0x00);        //Device Address Configuration
cc_tx_writeReg(SETTLING_CFG,0x0B);    //Frequency Synthesizer Calibration and Settling Con..
cc_tx_writeReg(FS_CFG,0x14);          //Frequency Synthesizer Configuration
cc_tx_writeReg(WOR_CFG1,0x08);        //eWOR Configuration Reg. 1
cc_tx_writeReg(WOR_CFG0,0x21);        //eWOR Configuration Reg. 0
cc_tx_writeReg(WOR_EVENT0_MSB,0x00);  //Event 0 Configuration MSB
cc_tx_writeReg(WOR_EVENT0_LSB,0x00);  //Event 0 Configuration LSB
cc_tx_writeReg(PKT_CFG2,0x04);        //Packet Configuration Reg. 2
cc_tx_writeReg(PKT_CFG1,0x05);        //Packet Configuration Reg. 1
//cc_tx_writeReg(PKT_CFG0,0x28);        //Packet Configuration Reg. 0
cc_tx_writeReg(PKT_CFG0,0x20);        //Packet Configuration Reg. 0

cc_tx_writeReg(RFEND_CFG1,0x0F);      //RFEND Configuration Reg. 1
cc_tx_writeReg(RFEND_CFG0,0x00);      //RFEND Configuration Reg. 0
cc_tx_writeReg(PA_CFG2,0x3F);         //Power Amplifier Configuration Reg. 2
cc_tx_writeReg(PA_CFG1,0x56);         //Power Amplifier Configuration Reg. 1
cc_tx_writeReg(PA_CFG0,0x7D);         //Power Amplifier Configuration Reg. 0
//cc_tx_writeReg(PKT_LEN,0x2A);         //Packet Length Configuration
cc_tx_writeReg(PKT_LEN,0xFF);         //Packet Length Configuration

cc_tx_writeReg(IF_MIX_CFG,0x00);      //IF Mix Configuration
cc_tx_writeReg(FREQOFF_CFG,0x22);     //Frequency Offset Correction Configuration
cc_tx_writeReg(TOC_CFG,0x0B);         //Timing Offset Correction Configuration
cc_tx_writeReg(MARC_SPARE,0x00);      //MARC Spare
cc_tx_writeReg(ECG_CFG,0x00);         //External Clock Frequency Configuration
cc_tx_writeReg(CFM_DATA_CFG,0x00);    //Custom frequency modulation enable
cc_tx_writeReg(EXT_CTRL,0x01);        //External Control Configuration
cc_tx_writeReg(RCCAL_FINE,0x00);      //RC Oscillator Calibration Fine
cc_tx_writeReg(RCCAL_COARSE,0x00);    //RC Oscillator Calibration Coarse
cc_tx_writeReg(RCCAL_OFFSET,0x00);    //RC Oscillator Calibration Clock Offset
cc_tx_writeReg(FREQOFF1,0x00);        //Frequency Offset MSB
cc_tx_writeReg(FREQOFF0,0x00);        //Frequency Offset LSB
cc_tx_writeReg(FREQ2,0x6C);           //Frequency Configuration [23:16]
cc_tx_writeReg(FREQ1,0xF3);           //Frequency Configuration [15:8]
cc_tx_writeReg(FREQ0,0x54);           //Frequency Configuration [7:0]
cc_tx_writeReg(IF_ADC2,0x02);         //Analog to Digital Converter Configuration Reg. 2
cc_tx_writeReg(IF_ADC1,0xA6);         //Analog to Digital Converter Configuration Reg. 1
cc_tx_writeReg(IF_ADC0,0x04);         //Analog to Digital Converter Configuration Reg. 0
cc_tx_writeReg(FS_DIG1,0x00);         //Frequency Synthesizer Digital Reg. 1
cc_tx_writeReg(FS_DIG0,0x5F);         //Frequency Synthesizer Digital Reg. 0
cc_tx_writeReg(FS_CAL3,0x00);         //Frequency Synthesizer Calibration Reg. 3
cc_tx_writeReg(FS_CAL2,0x20);         //Frequency Synthesizer Calibration Reg. 2
cc_tx_writeReg(FS_CAL1,0x40);         //Frequency Synthesizer Calibration Reg. 1
cc_tx_writeReg(FS_CAL0,0x0E);         //Frequency Synthesizer Calibration Reg. 0
cc_tx_writeReg(FS_CHP,0x28);          //Frequency Synthesizer Charge Pump Configuration
cc_tx_writeReg(FS_DIVTWO,0x03);       //Frequency Synthesizer Divide by 2
cc_tx_writeReg(FS_DSM1,0x00);         //FS Digital Synthesizer Module Configuration Reg. 1
cc_tx_writeReg(FS_DSM0,0x33);         //FS Digital Synthesizer Module Configuration Reg. 0
cc_tx_writeReg(FS_DVC1,0xFF);         //Frequency Synthesizer Divider Chain Configuration ..
cc_tx_writeReg(FS_DVC0,0x17);         //Frequency Synthesizer Divider Chain Configuration ..
cc_tx_writeReg(FS_LBI,0x00);          //Frequency Synthesizer Local Bias Configuration
cc_tx_writeReg(FS_PFD,0x50);          //Frequency Synthesizer Phase Frequency Detector Con..
cc_tx_writeReg(FS_PRE,0x6E);          //Frequency Synthesizer Prescaler Configuration
cc_tx_writeReg(FS_REG_DIV_CML,0x14);  //Frequency Synthesizer Divider Regulator Configurat..
cc_tx_writeReg(FS_SPARE,0xAC);        //Frequency Synthesizer Spare
cc_tx_writeReg(FS_VCO4,0x13);         //FS Voltage Controlled Oscillator Configuration Reg..
cc_tx_writeReg(FS_VCO3,0x00);         //FS Voltage Controlled Oscillator Configuration Reg..
cc_tx_writeReg(FS_VCO2,0x00);         //FS Voltage Controlled Oscillator Configuration Reg..
cc_tx_writeReg(FS_VCO1,0xAC);         //FS Voltage Controlled Oscillator Configuration Reg..
cc_tx_writeReg(FS_VCO0,0xB4);         //FS Voltage Controlled Oscillator Configuration Reg..
cc_tx_writeReg(GBIAS6,0x00);          //Global Bias Configuration Reg. 6
cc_tx_writeReg(GBIAS5,0x02);          //Global Bias Configuration Reg. 5
cc_tx_writeReg(GBIAS4,0x00);          //Global Bias Configuration Reg. 4
cc_tx_writeReg(GBIAS3,0x00);          //Global Bias Configuration Reg. 3
cc_tx_writeReg(GBIAS2,0x10);          //Global Bias Configuration Reg. 2
cc_tx_writeReg(GBIAS1,0x00);          //Global Bias Configuration Reg. 1
cc_tx_writeReg(GBIAS0,0x00);          //Global Bias Configuration Reg. 0
cc_tx_writeReg(IFAMP,0x01);           //Intermediate Frequency Amplifier Configuration
cc_tx_writeReg(LNA,0x01);             //Low Noise Amplifier Configuration
cc_tx_writeReg(RXMIX,0x01);           //RX Mixer Configuration
cc_tx_writeReg(XOSC5,0x0E);           //Crystal Oscillator Configuration Reg. 5
cc_tx_writeReg(XOSC4,0xA0);           //Crystal Oscillator Configuration Reg. 4
cc_tx_writeReg(XOSC3,0x03);           //Crystal Oscillator Configuration Reg. 3
cc_tx_writeReg(XOSC2,0x04);           //Crystal Oscillator Configuration Reg. 2
cc_tx_writeReg(XOSC1,0x03);           //Crystal Oscillator Configuration Reg. 1
cc_tx_writeReg(XOSC0,0x00);           //Crystal Oscillator Configuration Reg. 0
cc_tx_writeReg(ANALOG_SPARE,0x00);    //Analog Spare
cc_tx_writeReg(PA_CFG3,0x00);         //Power Amplifier Configuration Reg. 3
cc_tx_writeReg(WOR_TIME1,0x00);       //eWOR Timer Counter Value MSB
cc_tx_writeReg(WOR_TIME0,0x00);       //eWOR Timer Counter Value LSB
cc_tx_writeReg(WOR_CAPTURE1,0x00);    //eWOR Timer Capture Value MSB
cc_tx_writeReg(WOR_CAPTURE0,0x00);    //eWOR Timer Capture Value LSB
cc_tx_writeReg(BIST,0x00);            //MARC Built-In Self-Test
cc_tx_writeReg(DCFILTOFFSET_I1,0xF8); //DC Filter Offset I MSB
cc_tx_writeReg(DCFILTOFFSET_I0,0x39); //DC Filter Offset I LSB
cc_tx_writeReg(DCFILTOFFSET_Q1,0x0E); //DC Filter Offset Q MSB
cc_tx_writeReg(DCFILTOFFSET_Q0,0x9B); //DC Filter Offset Q LSB
cc_tx_writeReg(IQIE_I1,0xEF);         //IQ Imbalance Value I MSB
cc_tx_writeReg(IQIE_I0,0xDE);         //IQ Imbalance Value I LSB
cc_tx_writeReg(IQIE_Q1,0x02);         //IQ Imbalance Value Q MSB
cc_tx_writeReg(IQIE_Q0,0x2F);         //IQ Imbalance Value Q LSB
cc_tx_writeReg(RSSI1,0x80);           //Received Signal Strength Indicator Reg. 1
cc_tx_writeReg(RSSI0,0x00);           //Received Signal Strength Indicator Reg.0
cc_tx_writeReg(MARCSTATE,0x41);       //MARC State
cc_tx_writeReg(LQI_VAL,0x00);         //Link Quality Indicator Value
cc_tx_writeReg(PQT_SYNC_ERR,0xFF);    //Preamble and Sync Word Error
cc_tx_writeReg(DEM_STATUS,0x00);      //Demodulator Status
cc_tx_writeReg(FREQOFF_EST1,0x00);    //Frequency Offset Estimate MSB
cc_tx_writeReg(FREQOFF_EST0,0x00);    //Frequency Offset Estimate LSB
cc_tx_writeReg(AGC_GAIN3,0x00);       //Automatic Gain Control Reg. 3
cc_tx_writeReg(AGC_GAIN2,0xD1);       //Automatic Gain Control Reg. 2
cc_tx_writeReg(AGC_GAIN1,0x13);       //Automatic Gain Control Reg. 1
cc_tx_writeReg(AGC_GAIN0,0x3F);       //Automatic Gain Control Reg. 0
cc_tx_writeReg(CFM_RX_DATA_OUT,0x00); //Custom Frequency Modulation RX Data
cc_tx_writeReg(CFM_TX_DATA_IN,0x00);  //Custom Frequency Modulation TX Data
cc_tx_writeReg(ASK_SOFT_RX_DATA,0x30);//ASK Soft Decision Output
cc_tx_writeReg(RNDGEN,0x7F);          //Random Number Generator Value
cc_tx_writeReg(MAGN2,0x00);           //Signal Magnitude after CORDIC [16]
cc_tx_writeReg(MAGN1,0x00);           //Signal Magnitude after CORDIC [15:8]
cc_tx_writeReg(MAGN0,0x00);           //Signal Magnitude after CORDIC [7:0]
cc_tx_writeReg(ANG1,0x00);            //Signal Angular after CORDIC [9:8]
cc_tx_writeReg(ANG0,0x00);            //Signal Angular after CORDIC [7:0]
cc_tx_writeReg(CHFILT_I2,0x08);       //Channel Filter Data Real Part [18:16]
cc_tx_writeReg(CHFILT_I1,0x00);       //Channel Filter Data Real Part [15:8]
cc_tx_writeReg(CHFILT_I0,0x00);       //Channel Filter Data Real Part [7:0]
cc_tx_writeReg(CHFILT_Q2,0x00);       //Channel Filter Data Imaginary Part [18:16]
cc_tx_writeReg(CHFILT_Q1,0x00);       //Channel Filter Data Imaginary Part [15:8]
cc_tx_writeReg(CHFILT_Q0,0x00);       //Channel Filter Data Imaginary Part [7:0]
cc_tx_writeReg(GPIO_STATUS,0x00);     //General Purpose Input/Output Status
cc_tx_writeReg(FSCAL_CTRL,0x01);      //Frequency Synthesizer Calibration Control
cc_tx_writeReg(PHASE_ADJUST,0x00);    //Frequency Synthesizer Phase Adjust
cc_tx_writeReg(PARTNUMBER,0x48);      //Part Number
cc_tx_writeReg(PARTVERSION,0x21);     //Part Revision
cc_tx_writeReg(SERIAL_STATUS,0x00);   //Serial Status
cc_tx_writeReg(MODEM_STATUS1,0x10);   //Modem Status Reg. 1
cc_tx_writeReg(MODEM_STATUS0,0x00);   //Modem Status Reg. 0
cc_tx_writeReg(MARC_STATUS1,0x00);    //MARC Status Reg. 1
cc_tx_writeReg(MARC_STATUS0,0x00);    //MARC Status Reg. 0
cc_tx_writeReg(PA_IFAMP_TEST,0x00);   //Power Amplifier Intermediate Frequency Amplifier T..
cc_tx_writeReg(FSRF_TEST,0x00);       //Frequency Synthesizer Test
cc_tx_writeReg(PRE_TEST,0x00);        //Frequency Synthesizer Prescaler Test
cc_tx_writeReg(PRE_OVR,0x00);         //Frequency Synthesizer Prescaler Override
cc_tx_writeReg(ADC_TEST,0x00);        //Analog to Digital Converter Test
cc_tx_writeReg(DVC_TEST,0x0B);        //Digital Divider Chain Test
cc_tx_writeReg(ATEST,0x40);           //Analog Test
cc_tx_writeReg(ATEST_LVDS,0x00);      //Analog Test LVDS
cc_tx_writeReg(ATEST_MODE,0x00);      //Analog Test Mode
cc_tx_writeReg(XOSC_TEST1,0x3C);      //Crystal Oscillator Test Reg. 1
cc_tx_writeReg(XOSC_TEST0,0x00);      //Crystal Oscillator Test Reg. 0
cc_tx_writeReg(RXFIRST,0x00);         //RX FIFO Pointer First Entry
cc_tx_writeReg(TXFIRST,0x00);         //TX FIFO Pointer First Entry
cc_tx_writeReg(RXLAST,0x00);          //RX FIFO Pointer Last Entry
cc_tx_writeReg(TXLAST,0x00);          //TX FIFO Pointer Last Entry
cc_tx_writeReg(NUM_TXBYTES,0x00);     //TX FIFO Status
cc_tx_writeReg(NUM_RXBYTES,0x00);     //RX FIFO Status
cc_tx_writeReg(FIFO_NUM_TXBYTES,0x0F);//TX FIFO Status
cc_tx_writeReg(FIFO_NUM_RXBYTES,0x00);//RX FIFO Status

// Carrier frequency = 435.802002 
// Modulation format = 2-FSK 
// Performance mode = High Performance 
// Bit rate = 9.6 
// Deviation = 3.997803 
// PA ramping = false 
// Symbol rate = 9.6 
// Packet bit length = 2 
// Packet length mode = Variable 
// Address config = No address check 
// Device address = 0 
// Whitening = false 
// Manchester enable = false 
// RX filter BW = 25.000000 
// TX power = 12 
// Packet length = 255 
//
// Rf settings for CC1120
//
//cc_tx_writeReg(IOCFG3,0xB0);          //GPIO3 IO Pin Configuration
//cc_tx_writeReg(IOCFG2,0x06);          //GPIO2 IO Pin Configuration
//cc_tx_writeReg(IOCFG1,0xB0);          //GPIO1 IO Pin Configuration
//cc_tx_writeReg(IOCFG0,0x40);          //GPIO0 IO Pin Configuration
//cc_tx_writeReg(SYNC3,0x00);           //Sync Word Configuration [31:24]
//cc_tx_writeReg(SYNC2,0x00);           //Sync Word Configuration [23:16]
//cc_tx_writeReg(SYNC1,0x7A);           //Sync Word Configuration [15:8]
//cc_tx_writeReg(SYNC0,0x0E);           //Sync Word Configuration [7:0]
//cc_tx_writeReg(SYNC_CFG1,0x0B);       //Sync Word Detection Configuration Reg. 1
//cc_tx_writeReg(SYNC_CFG0,0x0B);       //Sync Word Length Configuration Reg. 0
//cc_tx_writeReg(DEVIATION_M,0x06);     //Frequency Deviation Configuration
//cc_tx_writeReg(MODCFG_DEV_E,0x03);    //Modulation Format and Frequency Deviation Configur..
//cc_tx_writeReg(DCFILT_CFG,0x1C);      //Digital DC Removal Configuration
//cc_tx_writeReg(PREAMBLE_CFG1,0x22);   //Preamble Length Configuration Reg. 1
//cc_tx_writeReg(PREAMBLE_CFG0,0x2A);   //Preamble Detection Configuration Reg. 0
//cc_tx_writeReg(FREQ_IF_CFG,0x40);     //RX Mixer Frequency Configuration
//cc_tx_writeReg(IQIC,0xC6);            //Digital Image Channel Compensation Configuration
//cc_tx_writeReg(CHAN_BW,0x08);         //Channel Filter Configuration
//cc_tx_writeReg(MDMCFG1,0x46);         //General Modem Parameter Configuration Reg. 1
//cc_tx_writeReg(MDMCFG0,0x05);         //General Modem Parameter Configuration Reg. 0
//cc_tx_writeReg(SYMBOL_RATE2,0x73);    //Symbol Rate Configuration Exponent and Mantissa [1..
//cc_tx_writeReg(SYMBOL_RATE1,0xA9);    //Symbol Rate Configuration Mantissa [15:8]
//cc_tx_writeReg(SYMBOL_RATE0,0x2A);    //Symbol Rate Configuration Mantissa [7:0]
//cc_tx_writeReg(AGC_REF,0x20);         //AGC Reference Level Configuration
//cc_tx_writeReg(AGC_CS_THR,0x19);      //Carrier Sense Threshold Configuration
//cc_tx_writeReg(AGC_GAIN_ADJUST,0x00); //RSSI Offset Configuration
//cc_tx_writeReg(AGC_CFG3,0x91);        //Automatic Gain Control Configuration Reg. 3
//cc_tx_writeReg(AGC_CFG2,0x20);        //Automatic Gain Control Configuration Reg. 2
//cc_tx_writeReg(AGC_CFG1,0xA9);        //Automatic Gain Control Configuration Reg. 1
//cc_tx_writeReg(AGC_CFG0,0xC3);        //Automatic Gain Control Configuration Reg. 0
//cc_tx_writeReg(FIFO_CFG,0x78);        //FIFO Configuration
//cc_tx_writeReg(DEV_ADDR,0x00);        //Device Address Configuration
//cc_tx_writeReg(SETTLING_CFG,0x03);    //Frequency Synthesizer Calibration and Settling Con..
//cc_tx_writeReg(FS_CFG,0x14);          //Frequency Synthesizer Configuration
//cc_tx_writeReg(WOR_CFG1,0x08);        //eWOR Configuration Reg. 1
//cc_tx_writeReg(WOR_CFG0,0x21);        //eWOR Configuration Reg. 0
//cc_tx_writeReg(WOR_EVENT0_MSB,0x00);  //Event 0 Configuration MSB
//cc_tx_writeReg(WOR_EVENT0_LSB,0x00);  //Event 0 Configuration LSB
//cc_tx_writeReg(PKT_CFG2,0x04);        //Packet Configuration Reg. 2
//cc_tx_writeReg(PKT_CFG1,0x05);        //Packet Configuration Reg. 1
//cc_tx_writeReg(PKT_CFG0,0x28);        //Packet Configuration Reg. 0
//cc_tx_writeReg(RFEND_CFG1,0x0F);      //RFEND Configuration Reg. 1
//cc_tx_writeReg(RFEND_CFG0,0x00);      //RFEND Configuration Reg. 0
//cc_tx_writeReg(PA_CFG2,0x39);         //Power Amplifier Configuration Reg. 2
//cc_tx_writeReg(PA_CFG1,0x56);         //Power Amplifier Configuration Reg. 1
//cc_tx_writeReg(PA_CFG0,0x7D);         //Power Amplifier Configuration Reg. 0
//cc_tx_writeReg(PKT_LEN,0xFF);         //Packet Length Configuration
//cc_tx_writeReg(IF_MIX_CFG,0x00);      //IF Mix Configuration
//cc_tx_writeReg(FREQOFF_CFG,0x22);     //Frequency Offset Correction Configuration
//cc_tx_writeReg(TOC_CFG,0x0B);         //Timing Offset Correction Configuration
//cc_tx_writeReg(MARC_SPARE,0x00);      //MARC Spare
//cc_tx_writeReg(ECG_CFG,0x00);         //External Clock Frequency Configuration
//cc_tx_writeReg(CFM_DATA_CFG,0x00);    //Custom frequency modulation enable
//cc_tx_writeReg(EXT_CTRL,0x01);        //External Control Configuration
//cc_tx_writeReg(RCCAL_FINE,0x00);      //RC Oscillator Calibration Fine
//cc_tx_writeReg(RCCAL_COARSE,0x00);    //RC Oscillator Calibration Coarse
//cc_tx_writeReg(RCCAL_OFFSET,0x00);    //RC Oscillator Calibration Clock Offset
//cc_tx_writeReg(FREQOFF1,0x00);        //Frequency Offset MSB
//cc_tx_writeReg(FREQOFF0,0x00);        //Frequency Offset LSB
//cc_tx_writeReg(FREQ2,0x6C);           //Frequency Configuration [23:16]
//cc_tx_writeReg(FREQ1,0xF3);           //Frequency Configuration [15:8]
//cc_tx_writeReg(FREQ0,0x54);           //Frequency Configuration [7:0]
//cc_tx_writeReg(IF_ADC2,0x02);         //Analog to Digital Converter Configuration Reg. 2
//cc_tx_writeReg(IF_ADC1,0xA6);         //Analog to Digital Converter Configuration Reg. 1
//cc_tx_writeReg(IF_ADC0,0x04);         //Analog to Digital Converter Configuration Reg. 0
//cc_tx_writeReg(FS_DIG1,0x00);         //Frequency Synthesizer Digital Reg. 1
//cc_tx_writeReg(FS_DIG0,0x5F);         //Frequency Synthesizer Digital Reg. 0
//cc_tx_writeReg(FS_CAL3,0x00);         //Frequency Synthesizer Calibration Reg. 3
//cc_tx_writeReg(FS_CAL2,0x20);         //Frequency Synthesizer Calibration Reg. 2
//cc_tx_writeReg(FS_CAL1,0x40);         //Frequency Synthesizer Calibration Reg. 1
//cc_tx_writeReg(FS_CAL0,0x0E);         //Frequency Synthesizer Calibration Reg. 0
//cc_tx_writeReg(FS_CHP,0x28);          //Frequency Synthesizer Charge Pump Configuration
//cc_tx_writeReg(FS_DIVTWO,0x03);       //Frequency Synthesizer Divide by 2
//cc_tx_writeReg(FS_DSM1,0x00);         //FS Digital Synthesizer Module Configuration Reg. 1
//cc_tx_writeReg(FS_DSM0,0x33);         //FS Digital Synthesizer Module Configuration Reg. 0
//cc_tx_writeReg(FS_DVC1,0xFF);         //Frequency Synthesizer Divider Chain Configuration ..
//cc_tx_writeReg(FS_DVC0,0x17);         //Frequency Synthesizer Divider Chain Configuration ..
//cc_tx_writeReg(FS_LBI,0x00);          //Frequency Synthesizer Local Bias Configuration
//cc_tx_writeReg(FS_PFD,0x50);          //Frequency Synthesizer Phase Frequency Detector Con..
//cc_tx_writeReg(FS_PRE,0x6E);          //Frequency Synthesizer Prescaler Configuration
//cc_tx_writeReg(FS_REG_DIV_CML,0x14);  //Frequency Synthesizer Divider Regulator Configurat..
//cc_tx_writeReg(FS_SPARE,0xAC);        //Frequency Synthesizer Spare
//cc_tx_writeReg(FS_VCO4,0x13);         //FS Voltage Controlled Oscillator Configuration Reg..
//cc_tx_writeReg(FS_VCO3,0x00);         //FS Voltage Controlled Oscillator Configuration Reg..
//cc_tx_writeReg(FS_VCO2,0x00);         //FS Voltage Controlled Oscillator Configuration Reg..
//cc_tx_writeReg(FS_VCO1,0xAC);         //FS Voltage Controlled Oscillator Configuration Reg..
//cc_tx_writeReg(FS_VCO0,0xB4);         //FS Voltage Controlled Oscillator Configuration Reg..
//cc_tx_writeReg(GBIAS6,0x00);          //Global Bias Configuration Reg. 6
//cc_tx_writeReg(GBIAS5,0x02);          //Global Bias Configuration Reg. 5
//cc_tx_writeReg(GBIAS4,0x00);          //Global Bias Configuration Reg. 4
//cc_tx_writeReg(GBIAS3,0x00);          //Global Bias Configuration Reg. 3
//cc_tx_writeReg(GBIAS2,0x10);          //Global Bias Configuration Reg. 2
//cc_tx_writeReg(GBIAS1,0x00);          //Global Bias Configuration Reg. 1
//cc_tx_writeReg(GBIAS0,0x00);          //Global Bias Configuration Reg. 0
//cc_tx_writeReg(IFAMP,0x01);           //Intermediate Frequency Amplifier Configuration
//cc_tx_writeReg(LNA,0x01);             //Low Noise Amplifier Configuration
//cc_tx_writeReg(RXMIX,0x01);           //RX Mixer Configuration
//cc_tx_writeReg(XOSC5,0x0E);           //Crystal Oscillator Configuration Reg. 5
//cc_tx_writeReg(XOSC4,0xA0);           //Crystal Oscillator Configuration Reg. 4
//cc_tx_writeReg(XOSC3,0x03);           //Crystal Oscillator Configuration Reg. 3
//cc_tx_writeReg(XOSC2,0x04);           //Crystal Oscillator Configuration Reg. 2
//cc_tx_writeReg(XOSC1,0x03);           //Crystal Oscillator Configuration Reg. 1
//cc_tx_writeReg(XOSC0,0x00);           //Crystal Oscillator Configuration Reg. 0
//cc_tx_writeReg(ANALOG_SPARE,0x00);    //Analog Spare
//cc_tx_writeReg(PA_CFG3,0x00);         //Power Amplifier Configuration Reg. 3
//cc_tx_writeReg(WOR_TIME1,0x00);       //eWOR Timer Counter Value MSB
//cc_tx_writeReg(WOR_TIME0,0x00);       //eWOR Timer Counter Value LSB
//cc_tx_writeReg(WOR_CAPTURE1,0x00);    //eWOR Timer Capture Value MSB
//cc_tx_writeReg(WOR_CAPTURE0,0x00);    //eWOR Timer Capture Value LSB
//cc_tx_writeReg(BIST,0x00);            //MARC Built-In Self-Test
//cc_tx_writeReg(DCFILTOFFSET_I1,0xF8); //DC Filter Offset I MSB
//cc_tx_writeReg(DCFILTOFFSET_I0,0x39); //DC Filter Offset I LSB
//cc_tx_writeReg(DCFILTOFFSET_Q1,0x0E); //DC Filter Offset Q MSB
//cc_tx_writeReg(DCFILTOFFSET_Q0,0x9B); //DC Filter Offset Q LSB
//cc_tx_writeReg(IQIE_I1,0xEF);         //IQ Imbalance Value I MSB
//cc_tx_writeReg(IQIE_I0,0xDE);         //IQ Imbalance Value I LSB
//cc_tx_writeReg(IQIE_Q1,0x02);         //IQ Imbalance Value Q MSB
//cc_tx_writeReg(IQIE_Q0,0x2F);         //IQ Imbalance Value Q LSB
//cc_tx_writeReg(RSSI1,0x80);           //Received Signal Strength Indicator Reg. 1
//cc_tx_writeReg(RSSI0,0x00);           //Received Signal Strength Indicator Reg.0
//cc_tx_writeReg(MARCSTATE,0x41);       //MARC State
//cc_tx_writeReg(LQI_VAL,0x00);         //Link Quality Indicator Value
//cc_tx_writeReg(PQT_SYNC_ERR,0xFF);    //Preamble and Sync Word Error
//cc_tx_writeReg(DEM_STATUS,0x00);      //Demodulator Status
//cc_tx_writeReg(FREQOFF_EST1,0x00);    //Frequency Offset Estimate MSB
//cc_tx_writeReg(FREQOFF_EST0,0x00);    //Frequency Offset Estimate LSB
//cc_tx_writeReg(AGC_GAIN3,0x00);       //Automatic Gain Control Reg. 3
//cc_tx_writeReg(AGC_GAIN2,0xD1);       //Automatic Gain Control Reg. 2
//cc_tx_writeReg(AGC_GAIN1,0x13);       //Automatic Gain Control Reg. 1
//cc_tx_writeReg(AGC_GAIN0,0x3F);       //Automatic Gain Control Reg. 0
//cc_tx_writeReg(CFM_RX_DATA_OUT,0x00); //Custom Frequency Modulation RX Data
//cc_tx_writeReg(CFM_TX_DATA_IN,0x00);  //Custom Frequency Modulation TX Data
//cc_tx_writeReg(ASK_SOFT_RX_DATA,0x30);//ASK Soft Decision Output
//cc_tx_writeReg(RNDGEN,0x7F);          //Random Number Generator Value
//cc_tx_writeReg(MAGN2,0x00);           //Signal Magnitude after CORDIC [16]
//cc_tx_writeReg(MAGN1,0x00);           //Signal Magnitude after CORDIC [15:8]
//cc_tx_writeReg(MAGN0,0x00);           //Signal Magnitude after CORDIC [7:0]
//cc_tx_writeReg(ANG1,0x00);            //Signal Angular after CORDIC [9:8]
//cc_tx_writeReg(ANG0,0x00);            //Signal Angular after CORDIC [7:0]
//cc_tx_writeReg(CHFILT_I2,0x08);       //Channel Filter Data Real Part [18:16]
//cc_tx_writeReg(CHFILT_I1,0x00);       //Channel Filter Data Real Part [15:8]
//cc_tx_writeReg(CHFILT_I0,0x00);       //Channel Filter Data Real Part [7:0]
//cc_tx_writeReg(CHFILT_Q2,0x00);       //Channel Filter Data Imaginary Part [18:16]
//cc_tx_writeReg(CHFILT_Q1,0x00);       //Channel Filter Data Imaginary Part [15:8]
//cc_tx_writeReg(CHFILT_Q0,0x00);       //Channel Filter Data Imaginary Part [7:0]
//cc_tx_writeReg(GPIO_STATUS,0x00);     //General Purpose Input/Output Status
//cc_tx_writeReg(FSCAL_CTRL,0x01);      //Frequency Synthesizer Calibration Control
//cc_tx_writeReg(PHASE_ADJUST,0x00);    //Frequency Synthesizer Phase Adjust
//cc_tx_writeReg(PARTNUMBER,0x48);      //Part Number
//cc_tx_writeReg(PARTVERSION,0x21);     //Part Revision
//cc_tx_writeReg(SERIAL_STATUS,0x10);   //Serial Status
//cc_tx_writeReg(MODEM_STATUS1,0x10);   //Modem Status Reg. 1
//cc_tx_writeReg(MODEM_STATUS0,0x00);   //Modem Status Reg. 0
//cc_tx_writeReg(MARC_STATUS1,0x00);    //MARC Status Reg. 1
//cc_tx_writeReg(MARC_STATUS0,0x00);    //MARC Status Reg. 0
//cc_tx_writeReg(PA_IFAMP_TEST,0x00);   //Power Amplifier Intermediate Frequency Amplifier T..
//cc_tx_writeReg(FSRF_TEST,0x00);       //Frequency Synthesizer Test
//cc_tx_writeReg(PRE_TEST,0x00);        //Frequency Synthesizer Prescaler Test
//cc_tx_writeReg(PRE_OVR,0x00);         //Frequency Synthesizer Prescaler Override
//cc_tx_writeReg(ADC_TEST,0x00);        //Analog to Digital Converter Test
//cc_tx_writeReg(DVC_TEST,0x0B);        //Digital Divider Chain Test
//cc_tx_writeReg(ATEST,0x40);           //Analog Test
//cc_tx_writeReg(ATEST_LVDS,0x00);      //Analog Test LVDS
//cc_tx_writeReg(ATEST_MODE,0x00);      //Analog Test Mode
//cc_tx_writeReg(XOSC_TEST1,0x3C);      //Crystal Oscillator Test Reg. 1
//cc_tx_writeReg(XOSC_TEST0,0x00);      //Crystal Oscillator Test Reg. 0
//cc_tx_writeReg(RXFIRST,0x00);         //RX FIFO Pointer First Entry
//cc_tx_writeReg(TXFIRST,0x00);         //TX FIFO Pointer First Entry
//cc_tx_writeReg(RXLAST,0x00);          //RX FIFO Pointer Last Entry
//cc_tx_writeReg(TXLAST,0x00);          //TX FIFO Pointer Last Entry
//cc_tx_writeReg(NUM_TXBYTES,0x00);     //TX FIFO Status
//cc_tx_writeReg(NUM_RXBYTES,0x00);     //RX FIFO Status
//cc_tx_writeReg(FIFO_NUM_TXBYTES,0x0F);//TX FIFO Status
//cc_tx_writeReg(FIFO_NUM_RXBYTES,0x00);//RX FIFO Status

}
