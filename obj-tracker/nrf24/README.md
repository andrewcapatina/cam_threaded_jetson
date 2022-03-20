
Example to put the receiver into transmit mode:

```
int main() 
{
	unsigned int val;
	char rising[7] = GPIO_EDGE_RISE;

	const char dev[32] = "/dev/spidev0.0";
	uint8_t mode = SPI_MODE_0;
	uint8_t bits = 8;
	uint32_t speed = 1000000;
	int lsb_setting = 0;
	int spi_dev_fd;
	
	// Setting (chip enable)
	gpio_export(GPIO_CE);
	gpio_set_dir(GPIO_CE, GPIO_DIR_OUTPUT);	// set gpio 4 as output.

	// Setting (IRQ)
	gpio_export(GPIO_IRQ);
	gpio_set_dir(GPIO_IRQ, GPIO_DIR_INPUT);
	gpio_set_edge(GPIO_IRQ, rising);
	gpio_set_active_edge(GPIO_IRQ);	// set to active low

	if(errno)
		return 0;

	spi_dev_fd = spi_init(dev, mode, bits, speed, lsb_setting);

	char addr;
	char shutdown_msg[2];
	char snd_msg[3];
	
	shutdown_msg[0] = 0x00;
	addr = W_REGISTER | CONFIG;
	spi_send_msg(spi_dev_fd, addr, shutdown_msg, 1);

	usleep(10000);


	char msg_addr[5];
	msg_addr[0] = 'R';	
	msg_addr[1] = 'x';	
	msg_addr[2] = 'A';	
	msg_addr[3] = 'A';	
	msg_addr[4] = 'A';	

	nrf_set_rx_address(spi_dev_fd, msg_addr, 0);

	nrf_set_tx_address(spi_dev_fd, msg_addr);


	int rtn;

	nrf_tx_init(spi_dev_fd); 
	nrf_print_all_registers(spi_dev_fd);

	
	char word[31];
	//strcpy(word, "fox jumped over the blue fence");
	
	FILE * fp = fopen("img.jpg", "r");
	send_files(spi_dev_fd, fp);
	fclose(fp);

	nrf_print_all_registers(spi_dev_fd);

	nrf_shutdown(spi_dev_fd);

	// Close the SPI device file for reading and 
	// writing.
	close(spi_dev_fd);

	// Bring back default settings for GPIO. 
	gpio_unexport(GPIO_CE);
	gpio_unexport(GPIO_IRQ);
	// echo 4 > /sys/class/gpio/unexport

	return 0;
}
```
Example to put the transceiver into receiver mode:

```
int main() 
{
	unsigned int val;
	char rising[7] = GPIO_EDGE_RISE;

	const char dev[32] = "/dev/spidev0.0";
	uint8_t mode = SPI_MODE_0;
	uint8_t bits = 8;
	uint32_t speed = 1000000;
	int lsb_setting = 0;
	int spi_dev_fd;
	
	// Setting (chip enable)
	gpio_export(GPIO_CE);
	gpio_set_dir(GPIO_CE, GPIO_DIR_OUTPUT);	// set gpio 4 as output.

	// Setting (IRQ)
	gpio_export(GPIO_IRQ);
	gpio_set_dir(GPIO_IRQ, GPIO_DIR_INPUT);
	gpio_set_edge(GPIO_IRQ, rising);
	gpio_set_active_edge(GPIO_IRQ);	// set to active low

	if(errno)
		return 0;

	spi_dev_fd = spi_init(dev, mode, bits, speed, lsb_setting);

	char addr;
	char shutdown_msg[2];
	char snd_msg[3];
	
	shutdown_msg[0] = 0x00;
	addr = W_REGISTER | CONFIG;
	spi_send_msg(spi_dev_fd, addr, shutdown_msg, 1);

	usleep(10000);


	char msg_addr[5];
	msg_addr[0] = 'R';	
	msg_addr[1] = 'x';	
	msg_addr[2] = 'A';	
	msg_addr[3] = 'A';	
	msg_addr[4] = 'A';	

	nrf_set_rx_address(spi_dev_fd, msg_addr, 0);

	msg_addr[0] = 0xe7;	
	msg_addr[1] = 0xe7;	
	msg_addr[2] = 0xe7;	
	msg_addr[3] = 0xe7;	
	msg_addr[4] = 0xe7;	
	nrf_set_tx_address(spi_dev_fd, msg_addr);

	//msg_addr[4] = 'B';
	//nrf_set_rx_address(spi_dev_fd, msg_addr, 1);

	//msg_addr[0] = 'C';
	//spi_send_msg(spi_dev_fd, W_REGISTER | RX_ADDR_P2, msg_addr, 1);

	int rtn;

	nrf_rx_init(spi_dev_fd); 


	nrf_print_all_registers(spi_dev_fd);


	
	char payload[64];
	int pipe, bytes; 

	printf("1\n");
	rtn = nrf_rx_read_payload(spi_dev_fd, payload, &pipe, &bytes);

	if(strcmp(payload, "SENDING_IMAGE") == 0)
	{
		printf("2 \n");
		receive_file(spi_dev_fd, 0);

	}

	nrf_print_all_registers(spi_dev_fd);

	usleep(100000);

	nrf_shutdown(spi_dev_fd);

	// Close the SPI device file for reading and 
	// writing.
	close(spi_dev_fd);

	// Bring back default settings for GPIO. 
	gpio_unexport(GPIO_CE);
	gpio_unexport(GPIO_IRQ);
	// echo 4 > /sys/class/gpio/unexport

	return 0;
}
```
	
