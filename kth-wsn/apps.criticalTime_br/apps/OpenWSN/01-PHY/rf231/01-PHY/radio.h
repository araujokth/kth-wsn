#ifndef __RADIO_H
#define __RADIO_H

// Radio Configuration (should be moved to platform files)
#  define AT_IRQ_PIN BIT6
#  define AT_SLP_TR_POUT P4OUT
#  define AT_SLP_TR_PDIR P4DIR
#  define AT_SLP_TR_PIN BIT7
#  define AT_SPI_POUT P4OUT
#  define AT_SPI_PDIR P4DIR
#  define AT_SPI_PIN BIT0
//#  define AT_TEST_PIN BIT6

uint8_t at_bytes_left; // current # of chars remaining to TX
uint8_t* at_tx_byte_str;           // pointer to remaining chars to tx
uint8_t* at_rx_byte_str;           // pointer to remaining chars to rx

uint8_t at_tx_buf[2];    // transmit buffer for register control
uint8_t at_rx_buf[2];    // receive buffer for register control

#define AT_MAX_PACKET_LEN 127
uint8_t at_sendbuffer[AT_MAX_PACKET_LEN + 4];
uint8_t at_recvbuffer[AT_MAX_PACKET_LEN + 4];

uint8_t at_current_channel;

#define AT_STATE_OFF 0x00
#define AT_STATE_TX_READY 0x01
#define AT_STATE_TXING 0x02
#define AT_STATE_RX_READY 0x03
#define AT_STATE_RX_WAITING 0x10
#define AT_STATE_RXING_CMD 0x11
#define AT_STATE_RXING_LEN 0x12
#define AT_STATE_RXING_STR 0x13
#define AT_STATE_RXING_LQI 0x14
//uint8_t at_state; //I'm doing my own variable inside P.nc

#define AT_SLP_TR_CLR AT_SLP_TR_POUT &= ~AT_SLP_TR_PIN
#define AT_SLP_TR_SET AT_SLP_TR_POUT |= AT_SLP_TR_PIN
#define AT_TX_START { AT_SLP_TR_SET; AT_SLP_TR_CLR; }

#define AT_CLR_IRQ at_get_reg(RG_IRQ_STATUS);          // read interrupt

//#define AT_FOUR_SIX_ON AT_SPI_POUT &= ~AT_TEST_PIN
//#define AT_FOUR_SIX_OFF AT_SPI_POUT |= AT_TEST_PIN

#define AT_SPI_ON AT_SPI_POUT &= ~AT_SPI_PIN//; atomic AT_FOUR_SIX_ON
#define AT_SPI_OFF AT_SPI_POUT |= AT_SPI_PIN//; atomic AT_FOUR_SIX_OFF

//#define TOGGLE_P11_ON P1OUT &= ~BIT1
//#define TOGGLE_P11_OFF P1OUT |= BIT1

#define AT_READ_REG(reg) (0x80 | reg)
#define AT_WRITE_REG(reg) (0xC0 | reg)

#define AT_FRAME_TX 0x60
#define AT_FRAME_RX 0x20

#define AT_BUSY (~AT_SPI_POUT & AT_SPI_PIN)
#define AT_REG_VAL at_rx_buf[1]
#define AT_WAIT while(AT_BUSY)
#define AT_UNTIL_STATUS(status)  while((at_get_reg(RG_TRX_STATUS) & 0x1F) != status)

//state change to sleep/wake up
#define AT_SLEEP AT_SLP_TR_SET
#define AT_WAKE AT_SLP_TR_CLR //repetition but makes things clearer in the radio drivers.

/*
int at_send(char* txstr, char* rxstr, int len);
void at_set_reg(char reg, char val);
char at_get_reg(char reg);

void at_init(void);
void at_config(void);
int at_useclock(void);
int at_test(void);

char at_txmode(char chan);
char at_rxmode(char chan);
void at_read (char** bytes_handle, char** len_handle);

void isr_at_irq(void);
void isr_at_rx(void);
*/
#endif
