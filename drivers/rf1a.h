// *************************************************************************************************
//
// Actual revision: $Revision: $
// Revision label:  $Name: $
// Revision state:  $State: $
//
// *************************************************************************************************
// Radio core access functions. Taken from TI reference code for CC430.
// *************************************************************************************************

// *************************************************************************************************
// Prototype section
// Define section

#define st(x)      			do { x } while (__LINE__ == -1)
#define ENTER_CRITICAL_SECTION(x)  	st( x = __read_status_register(); __dint(); )
#define EXIT_CRITICAL_SECTION(x)	__write_status_register(x)

unsigned char Strobe(unsigned char strobe);
unsigned char ReadSingleReg(unsigned char addr);
void WriteSingleReg(unsigned char addr, unsigned char value);
void ReadBurstReg(unsigned char addr, unsigned char *buffer, unsigned char count);
void WriteBurstReg(unsigned char addr, unsigned char *buffer, unsigned char count);
void ResetRadioCore(void);
void WritePATable(unsigned char value);
void WaitForXT2(void);

void radio_reset(void);
void radio_powerdown(void);
void radio_sxoff(void);
void radio_idle(void);
void open_radio(void);
void close_radio(void);
void pmm_set_high_current_mode(void);
void pmm_set_low_current_mode(void);
