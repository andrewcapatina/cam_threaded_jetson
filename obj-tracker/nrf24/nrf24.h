// Need to set GPIO pins based on board used.
#define GPIO_CE 200 
#define GPIO_IRQ 168
#define GPIO_EDGE_FALL "falling"
#define GPIO_EDGE_RISE "rising"
#define GPIO_LVL_HIGH 0x01
#define GPIO_LVL_LOW 0x00
#define GPIO_DIR_INPUT 0x00
#define GPIO_DIR_OUTPUT 0x01

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define POLL_TIMEOUT (10 * 1000) // 2 seconds
#define MAX_BUF 64

#define NUM_PAYLOAD_BYTES 30

/* Instruction Mnemonics */
#define R_REGISTER    0x00 /* last 4 bits will indicate reg. address */
#define W_REGISTER    0x20 /* last 4 bits will indicate reg. address */
#define REGISTER_MASK 0x1F
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_TX_PAYLOAD_NO_ACK 0xB0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define ACTIVATE      0x50 
#define R_RX_PL_WID   0x60
#define NOP           0xFF

/* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define RPD         0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE     0x1D

/* Bit Mnemonics */

/* configuration register */
#define MASK_RX_DR  0x40
#define MASK_TX_DS  0x20
#define MASK_MAX_RT 0x10
#define EN_CRC      0x08
#define CRCO        0x04
#define PWR_UP      0x02
#define PRIM_RX     0x01

/* enable auto acknowledgment */
#define ENAA_P5     0x20
#define ENAA_P4     0x10
#define ENAA_P3     0x08
#define ENAA_P2     0x04
#define ENAA_P1     0x02
#define ENAA_P0     0x01

/* enable rx addresses */
#define ERX_P5      0x20
#define ERX_P4      0x10
#define ERX_P3      0x08
#define ERX_P2      0x04
#define ERX_P1      0x02
#define ERX_P0      0x01  

/* general status register */
#define RX_DR       0x40
#define TX_DS       0x20
#define MAX_RT      0x10
#define RX_P_NO	    1
#define TX_FULL     0x01

/* fifo status */
#define TX_REUSE    0x40
#define FIFO_FULL   0x20
#define TX_EMPTY    0x10
#define RX_FULL     0x2
#define RX_EMPTY    0x1

/* dynamic length */
#define DPL_P0      0x01
#define DPL_P1      0x02
#define DPL_P2      0x04
#define DPL_P3      0x08
#define DPL_P4      0x10
#define DPL_P5      0x20

/* Payload widths */
#define RX_WIDTH    32 	// In bytes

/**
 *	Function to bring a selected 
 *	GPIO into userspace.
 * */
int gpio_export(unsigned int gpio);

/**
 *	Function to bring a GPIO back into 
 *	the kernel space settings.
 * */
int gpio_unexport(unsigned int gpio);

/**
 *	Function to set GPIO direction.
 *
 *	out_flag = 1: set as output.
 *	out_flag = 0: set as input.
 *
 * */
int gpio_set_dir(unsigned int gpio, unsigned int out_flag);

/**
 *	Returns file descriptor for gpio number 
 *	value.
 * */
int gpio_get_value_fd(unsigned int gpio);

/**
 *	Set value of a GPIO pin.
 *
 * */
int gpio_set_value(unsigned int gpio, unsigned int value);

int gpio_get_active_edge(unsigned int gpio, unsigned int * value);

int gpio_set_active_edge(unsigned int gpio);

/**
 *	Gets value of a GPIO pin.
 * */
int gpio_get_value(unsigned int gpio, unsigned int * value);

int gpio_set_value(unsigned int gpio, char * value);

/**
 *	Set the GPIO active edge.
 *	
 * */
int gpio_set_edge(unsigned int gpio, char *edge);

/**
 *	Open the GPIO file for reading its 
 *	current value.
 *
 * */
int gpio_fd_open(unsigned int gpio);

/**
 * Close the file descriptor.
 * */
int gpio_fd_close(int fd);

int spi_init(const char * dev, int mode, int bits, int speed, int lsb_setting);

int spi_send_msg(int spi_dev_fd, char addr, char * data, int len);

int spi_read_msg(int spi_dev_fd, char addr, char * status, char * copy_to, int len);

void nrf_print_all_registers(int spi_dev_fd);

/**
  *	Function to write a byte to memory mapped register
  * of the transceiver.
  *
  *	spi_dev_fd: spi file descriptor.
  **/
void nrf_write_reg_byte(int spi_dev_fd, char reg, char value);

/**
  *	Function to write multi byte value to memory mapped register
  * of the transceiver.
  *
  *	spi_dev_fd: spi file descriptor.
  **/
void nrf_write_reg_multi_byte(int spi_dev_fd, char reg, char * value, int len);

/**
  *	Function to write command byte to memory mapped register
  * of the transceiver.
  *
  *	spi_dev_fd: spi file descriptor.
  **/
void nrf_write_command_byte(int spi_dev_fd, char cmd, char value);

/**
  *	Function to write multi byte command to memory mapped register
  * of the transceiver.
  *
  *	spi_dev_fd: spi file descriptor.
  **/
void nrf_write_command_multi_byte(int spi_dev_fd, char cmd, char * value, int len);

/**
 *	Initializes nrf registers 
 *	for transmit mode.
 *
 * 	spi_dev_fd: file descriptor for spi device.
 * */
void nrf_tx_init(int spi_dev_fd);

/**
 *	Initializes memory mapped registers for receive mode.
 * 	spi_dev_fd: file descriptor for spi device.
 * */
void nrf_rx_init(int spi_dev_fd);

/**
 *	Sets the rx address of the transceiver.
 *	For pipe > 0, only one byte needed to set 
 *	the address. First 4 bytes copy of data pipe
 *	0.
 *
 * 	spi_dev_fd: file descriptor for spi device.
 * */
int nrf_set_rx_address(int spi_dev_fd, char * addr, int pipe);

/**
 *	Sets the tx address of the transceiver.
 *
 * 	spi_dev_fd: file descriptor for spi device.
 * */
int nrf_set_tx_address(int spi_dev_fd, char * addr);

/**
  *	Function to check which data pipe is available 
  * to read.
  *
  *	spi_dev_fd: spi file descriptor.
  * pipe: available data pipe.
  **/
bool nrf_rx_pipe_available(int spi_dev_fd, int * pipe);

/**
 *	Shutdown the transceiver.
 * 	spi_dev_fd: file descriptor for spi device.
 * */
void nrf_shutdown(int spi_dev_fd);

int nrf_tx_new_payload(int spi_dev_fd, char * payload, int len);

/**
 *
 *
 * 	if 0 is returned: send pending.
 *	If 1 is returned: TX_DS set.
 *	If 2 is returned: MAX_RT set.
 * 	spi_dev_fd: file descriptor for spi device.
 * */
int nrf_tx_pending_send(int spi_dev_fd);

/**
 *	Reads the payload when data pipe
 *	is available.
 *
 * 	spi_dev_fd: file descriptor for spi device.
 * */
int nrf_rx_read_payload(int spi_dev_fd, char * payload, int * pipe, int * bytes);

/**
 *	Function to load a payload and send a packet.
 *
 *
 *	spi_dev_fd: file descriptor for spi device.
 * */
int nrf_tx_send_packet(int spi_dev_fd, char * payload, int len);

/**
  *	Function to send a file to receiver.
  *
  *	spi_dev_fd: spi file descriptor.
  * 	fp: file descriptor/pointer.
  *	file_type: send image =0, video = 1.
  **/
void send_file(int spi_dev_fd, FILE * fp, int file_type);

/**
  *	Function to send a file to receiver.
  *
  *	spi_dev_fd: spi file descriptor.
  * file_type: 0 = jpeg, 1 = video.
  **/
int receive_file(int spi_dev_fd, int file_type);

/**
  *	Put transceiver into receive mode.
  *
  *	spi_dev_fd: spi file descriptor.
  **/
int nrf_rx_mode(int spi_dev_fd);

/**
  *	Put transceiver into transmit mode.
  *
  *	spi_dev_fd: spi file descriptor.
  **/
int nrf_tx_mode(int spi_dev_fd);

