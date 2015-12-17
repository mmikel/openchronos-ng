// system
#include <openchronos.h>
#include <SMARTRF_CC430.h>

// driver
#include <drivers/display.h>
#include <drivers/battery.h>
#include <drivers/ports.h>
#include <drivers/timer.h>
#include <drivers/rf1a.h>
#include <drivers/buzzer.h>

//***************************************CC1101 define**************************************************//
// CC1101 CONFIG REGSITER

#define CC1101_IOCFG2       0x00        // GDO2 output pin configuration
#define CC1101_IOCFG1       0x01        // GDO1 output pin configuration
#define CC1101_IOCFG0       0x02        // GDO0 output pin configuration
#define CC1101_FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define CC1101_SYNC1        0x04        // Sync word, high INT8U
#define CC1101_SYNC0        0x05        // Sync word, low INT8U
#define CC1101_PKTLEN       0x06        // Packet length
#define CC1101_PKTCTRL1     0x07        // Packet automation control
#define CC1101_PKTCTRL0     0x08        // Packet automation control
#define CC1101_ADDR         0x09        // Device address
#define CC1101_CHANNR       0x0A        // Channel number
#define CC1101_FSCTRL1      0x0B        // Frequency synthesizer control
#define CC1101_FSCTRL0      0x0C        // Frequency synthesizer control
#define CC1101_FREQ2        0x0D        // Frequency control word, high INT8U
#define CC1101_FREQ1        0x0E        // Frequency control word, middle INT8U
#define CC1101_FREQ0        0x0F        // Frequency control word, low INT8U
#define CC1101_MDMCFG4      0x10        // Modem configuration
#define CC1101_MDMCFG3      0x11        // Modem configuration
#define CC1101_MDMCFG2      0x12        // Modem configuration
#define CC1101_MDMCFG1      0x13        // Modem configuration
#define CC1101_MDMCFG0      0x14        // Modem configuration
#define CC1101_DEVIATN      0x15        // Modem deviation setting
#define CC1101_MCSM2        0x16        // Main Radio Control State Machine configuration
#define CC1101_MCSM1        0x17        // Main Radio Control State Machine configuration
#define CC1101_MCSM0        0x18        // Main Radio Control State Machine configuration
#define CC1101_FOCCFG       0x19        // Frequency Offset Compensation configuration
#define CC1101_BSCFG        0x1A        // Bit Synchronization configuration
#define CC1101_AGCCTRL2     0x1B        // AGC control
#define CC1101_AGCCTRL1     0x1C        // AGC control
#define CC1101_AGCCTRL0     0x1D        // AGC control
#define CC1101_WOREVT1      0x1E        // High INT8U Event 0 timeout
#define CC1101_WOREVT0      0x1F        // Low INT8U Event 0 timeout
#define CC1101_WORCTRL      0x20        // Wake On Radio control
#define CC1101_FREND1       0x21        // Front end RX configuration
#define CC1101_FREND0       0x22        // Front end TX configuration
#define CC1101_FSCAL3       0x23        // Frequency synthesizer calibration
#define CC1101_FSCAL2       0x24        // Frequency synthesizer calibration
#define CC1101_FSCAL1       0x25        // Frequency synthesizer calibration
#define CC1101_FSCAL0       0x26        // Frequency synthesizer calibration
#define CC1101_RCCTRL1      0x27        // RC oscillator configuration
#define CC1101_RCCTRL0      0x28        // RC oscillator configuration
#define CC1101_FSTEST       0x29        // Frequency synthesizer calibration control
#define CC1101_PTEST        0x2A        // Production test
#define CC1101_AGCTEST      0x2B        // AGC test
#define CC1101_TEST2        0x2C        // Various test settings
#define CC1101_TEST1        0x2D        // Various test settings
#define CC1101_TEST0        0x2E        // Various test settings

//CC1101 Strobe commands
#define CC1101_SRES         0x30        // Reset chip.
#define CC1101_SFSTXON      0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                        // If in RX/TX: Go to a wait state where only the synthesizer is
                                        // running (for quick RX / TX turnaround).
#define CC1101_SXOFF        0x32        // Turn off crystal oscillator.
#define CC1101_SCAL         0x33        // Calibrate frequency synthesizer and turn it off
                                        // (enables quick start).
#define CC1101_SRX          0x34        // Enable RX. Perform calibration first if coming from IDLE and
                                        // MCSM0.FS_AUTOCAL=1.
#define CC1101_STX          0x35        // In IDLE state: Enable TX. Perform calibration first if
                                        // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                        // Only go to TX if channel is clear.
#define CC1101_SIDLE        0x36        // Exit RX / TX, turn off frequency synthesizer and exit
                                        // Wake-On-Radio mode if applicable.
#define CC1101_SAFC         0x37        // Perform AFC adjustment of the frequency synthesizer
#define CC1101_SWOR         0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define CC1101_SPWD         0x39        // Enter power down mode when CSn goes high.
#define CC1101_SFRX         0x3A        // Flush the RX FIFO buffer.
#define CC1101_SFTX         0x3B        // Flush the TX FIFO buffer.
#define CC1101_SWORRST      0x3C        // Reset real time clock.
#define CC1101_SNOP         0x3D        // No operation. May be used to pad strobe commands to two
                                        // INT8Us for simpler software.
//CC1101 STATUS REGSITER
#define CC1101_PARTNUM      0x30
#define CC1101_VERSION      0x31
#define CC1101_FREQEST      0x32
#define CC1101_LQI          0x33
#define CC1101_RSSI         0x34
#define CC1101_MARCSTATE    0x35
#define CC1101_WORTIME1     0x36
#define CC1101_WORTIME0     0x37
#define CC1101_PKTSTATUS    0x38
#define CC1101_VCO_VC_DAC   0x39
#define CC1101_TXBYTES      0x3A
#define CC1101_RXBYTES      0x3B

//CC1101 PATABLE,TXFIFO,RXFIFO
#define CC1101_PATABLE      0x3E
#define CC1101_TXFIFO       0x3F
#define CC1101_RXFIFO       0x3F

#define WRITE_BURST         0x40	// write burst
#define READ_SINGLE         0x80	// read single
#define READ_BURST          0xC0	// read burst
#define BYTES_IN_RXFIFO     0x7F  	// byte number in RXfifo

// *************************************************************************************************
// @fn          GDOx_ISR
// @brief       GDO0/2 ISR to detect received packet.
// @param       none
// @return      none
// *************************************************************************************************
//pfs

#ifdef __GNUC__
#include <legacymsp430.h>
interrupt (CC1101_VECTOR) radio_ISR(void)
#else
#pragma vector=CC1101_VECTOR
__interrupt void radio_ISR(void)
#endif
{
// handle code here
}

#define RXTX_BUFFER_SIZE 61

uint8_t PaTable[8] = {0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60};

unsigned char rxtx_buffer[RXTX_BUFFER_SIZE];

void WriteBurstPATable(unsigned char *buffer, uint8_t count) {
    volatile uint8_t i = 0; 
    uint16_t int_state;

    ENTER_CRITICAL_SECTION(int_state);
    while (!(RF1AIFCTL1 & RFINSTRIFG));
    RF1AINSTRW = 0x7E00 + buffer[i];          // PA Table burst write   

    for (i = 1; i < count; i++) {
        RF1ADINB = buffer[i];                   // Send data
        while (!(RFDINIFG & RF1AIFCTL1));       // Wait for TX to finish
    } 
    i = RF1ADOUTB;                            // Reset RFDOUTIFG flag which contains status byte

    while( !(RF1AIFCTL1 & RFINSTRIFG));
    RF1AINSTRB = RF_SNOP;                     // reset PA Table pointer

    EXIT_CRITICAL_SECTION(int_state); 
}

static void WriteRfSettings(void) {
    WriteSingleReg(CC1101_FSCTRL1,  SMARTRF_SETTING_FSCTRL1);
    WriteSingleReg(CC1101_FSCTRL0,  SMARTRF_SETTING_FSCTRL0);
    WriteSingleReg(CC1101_FREQ2,    SMARTRF_SETTING_FREQ2);
    WriteSingleReg(CC1101_FREQ1,    SMARTRF_SETTING_FREQ1);
    WriteSingleReg(CC1101_FREQ0,    SMARTRF_SETTING_FREQ0);
    WriteSingleReg(CC1101_MDMCFG4,  SMARTRF_SETTING_MDMCFG4);
    WriteSingleReg(CC1101_MDMCFG3,  SMARTRF_SETTING_MDMCFG3);
    WriteSingleReg(CC1101_MDMCFG2,  SMARTRF_SETTING_MDMCFG2);
    WriteSingleReg(CC1101_MDMCFG1,  SMARTRF_SETTING_MDMCFG1);
    WriteSingleReg(CC1101_MDMCFG0,  SMARTRF_SETTING_MDMCFG0);
    WriteSingleReg(CC1101_CHANNR,   SMARTRF_SETTING_CHANNR);
    WriteSingleReg(CC1101_DEVIATN,  SMARTRF_SETTING_DEVIATN);
    WriteSingleReg(CC1101_FREND1,   SMARTRF_SETTING_FREND1);
    WriteSingleReg(CC1101_FREND0,   SMARTRF_SETTING_FREND0);
    WriteSingleReg(CC1101_MCSM0 ,   SMARTRF_SETTING_MCSM0);
    WriteSingleReg(CC1101_FOCCFG,   SMARTRF_SETTING_FOCCFG);
    WriteSingleReg(CC1101_BSCFG,    SMARTRF_SETTING_BSCFG);
    WriteSingleReg(CC1101_AGCCTRL2, SMARTRF_SETTING_AGCCTRL2);
    WriteSingleReg(CC1101_AGCCTRL1, SMARTRF_SETTING_AGCCTRL1);
    WriteSingleReg(CC1101_AGCCTRL0, SMARTRF_SETTING_AGCCTRL0);
    WriteSingleReg(CC1101_FSCAL3,   SMARTRF_SETTING_FSCAL3);
    WriteSingleReg(CC1101_FSCAL2,   SMARTRF_SETTING_FSCAL2);
    WriteSingleReg(CC1101_FSCAL1,   SMARTRF_SETTING_FSCAL1);
    WriteSingleReg(CC1101_FSCAL0,   SMARTRF_SETTING_FSCAL0);
    WriteSingleReg(CC1101_FSTEST,   SMARTRF_SETTING_FSTEST);
    WriteSingleReg(CC1101_TEST2,    SMARTRF_SETTING_TEST2);
    WriteSingleReg(CC1101_TEST1,    SMARTRF_SETTING_TEST1);
    WriteSingleReg(CC1101_TEST0,    SMARTRF_SETTING_TEST0);

    WriteSingleReg(CC1101_IOCFG2,   SMARTRF_SETTING_IOCFG2); 	//serial clock.synchronous to the data in synchronous serial mode
    WriteSingleReg(CC1101_IOCFG0,   SMARTRF_SETTING_IOCFG0D);  	//asserts when sync word has been sent/received, and de-asserts at the end of the packet 
    WriteSingleReg(CC1101_PKTCTRL1, SMARTRF_SETTING_PKTCTRL1);		//two status bytes will be appended to the payload of the packet,including RSSI LQI and CRC OK
											//No address check
    WriteSingleReg(CC1101_PKTCTRL0, SMARTRF_SETTING_PKTCTRL0);		//whitening off;CRC Enable\A3\BBvariable length packets, packet length configured by the first byte after sync word
    WriteSingleReg(CC1101_ADDR,     SMARTRF_SETTING_ADDR);		//address used for packet filtration.
    WriteSingleReg(CC1101_PKTLEN,   SMARTRF_SETTING_PKTLEN); 	//61 bytes max length
}

static void cc1101_init(void) {
    PMMCTL0_H = 0xA5;
    PMMCTL0_L |= PMMHPMRE_L; 
    PMMCTL0_H = 0x00; 

    WriteRfSettings();
    WriteBurstPATable(&PaTable[0], 8);
}

static void rpc_activate(void) {
    display_chars(0, LCD_SEG_L2_5_0, "RPC  ", SEG_SET);

    display_symbol(0, LCD_ICON_BEEPER1, SEG_ON);
    display_symbol(0, LCD_ICON_BEEPER2, SEG_OFF);
    display_symbol(0, LCD_ICON_BEEPER3, SEG_OFF);
}

static void rpc_deactivate(void) {
    display_symbol(0, LCD_ICON_BEEPER1, SEG_OFF);
    display_symbol(0, LCD_ICON_BEEPER2, SEG_OFF);
    display_symbol(0, LCD_ICON_BEEPER3, SEG_OFF);
}

static void rpc_up_btn(void) {
    display_symbol(0, LCD_ICON_BEEPER2, SEG_ON);
    display_symbol(0, LCD_ICON_BEEPER3, SEG_ON);

    open_radio();
    ResetRadioCore();
    cc1101_init();

    RF1AIES |= BIT9;
    RF1AIFG &= ~BIT9;                         // Clear pending interrupts
    RF1AIE &= ~BIT9;                          // Disable TX end-of-packet interrupt

    uint8_t i;

    for (i = 0; i < RXTX_BUFFER_SIZE; i++) {
        rxtx_buffer[i] = i;
    }

    WriteBurstReg(RF_TXFIFOWR, rxtx_buffer, RXTX_BUFFER_SIZE);

    Strobe(RF_STX);
    timer0_delay(42, LPM1_bits);
    while ((Strobe(RF_SNOP)&0xF0)!=0);
    Strobe(CC1101_SFTX);                      //flush TXfifo

    radio_reset();
    radio_powerdown();

    note alarm[2] = {0x1902, 0x000F};
    buzzer_play(alarm);
    display_symbol(0, LCD_ICON_BEEPER2, SEG_OFF);
    display_symbol(0, LCD_ICON_BEEPER3, SEG_OFF);
}

static void rpc_down_btn(void) {
    display_symbol(0, LCD_ICON_BEEPER2, SEG_ON);
    display_symbol(0, LCD_ICON_BEEPER3, SEG_ON);

    open_radio();
    ResetRadioCore();
    cc1101_init();

    RF1AIES |= BIT9;                          
    RF1AIFG &= ~BIT9;                         // Clear pending interrupts
    RF1AIE &= ~BIT9;                          // Disable TX end-of-packet interrupt
    
    uint8_t i;
    Strobe(CC1101_SRX);    
    
    uint8_t size;
    uint8_t status[2];

    size = ReadSingleReg(CC1101_RXFIFO);
    ReadBurstReg(CC1101_RXFIFO, rxtx_buffer, size);
    ReadBurstReg(CC1101_RXFIFO, status, 2);
    Strobe(CC1101_SFRX);

    rxtx_buffer[5] = 0x0;
    
    display_chars(0, LCD_SEG_L2_5_0, rxtx_buffer, SEG_SET);

}

static void rpc_num_btn(void) {
    
}

static void rpc_lstar_btn(void) {
    
}

static void rpc_lnum_btn(void) {
    
}

static void rpc_updown_btn(void) {
    
}

void mod_rpc_init(void) {
    menu_add_entry("RPC  ", &rpc_up_btn, &rpc_down_btn, &rpc_num_btn, &rpc_lstar_btn, &rpc_lnum_btn, &rpc_updown_btn, &rpc_activate, &rpc_deactivate);
}

void mod_rpc_test(void) {
    while (1) {
        rpc_up_btn();
	__delay_cycles(65535);
    }
}