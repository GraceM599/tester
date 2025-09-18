//
// Created by mihir on 9/18/2025.
//

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <cstddef>
#include <cstdint>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <cmath>
#include <linux/spi/spidev.h>

void initSPI() {

    uint8_t mode = SPI_MODE_3;

    int bits_per_word = 8;
    int speed = 100000;

    f_dev = open("/dev/spidev0.0", O_RDWR);
    if (f_dev < 0) {
        std::cout<< "ERR: Failed to open SPI communication on /dev/spidev0.0";
    }


    //config SPI parameters
    if(ioctl(f_dev, SPI_IOC_WR_MODE, &mode) < 0){
        close(f_dev);
    }
    if (ioctl(f_dev, SPI_IOC_RD_MODE, &mode) < 0) {
        close(f_dev);
    }
    /* Set bits per word*/
    if (ioctl(f_dev, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0) {
        close(f_dev);
    }
    if (ioctl(f_dev, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word) < 0) {
        close(f_dev);
    }

    /* Set SPI speed*/
    if (ioctl(f_dev, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        close(f_dev);
    }
    if (ioctl(f_dev, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
        close(f_dev);
    }
}

int readRegister(uint8_t register_add){
    struct spi_ioc_transfer xfer[1] = {0};

    // Write message for register address
    uint8_t reg_addr = register_add | 0x80;
    uint8_t data[2];
    data[0] = reg_addr;
    data[1] = 0x00;
    xfer[0].tx_buf = (__u64)data;      // output buffer
    xfer[0].rx_buf = (__u64)data;      // input buffer
    xfer[0].len = (__u32)sizeof(data);  // length of data to read

    int retv = ioctl(f_dev, SPI_IOC_MESSAGE(1), &xfer);
    if (retv < 0)
    {
        std::cout << "error in spi_read_reg8(): ioctl(SPI_IOC_MESSAGE(2))" << std::endl;
    }
    return data[1];
}

int writeRegister(uint8_t register_addr, uint8_t value) {
    struct spi_ioc_transfer xfer[1] = {0};

    // Write message for register address
    uint8_t data[2];
    data[0] = register_addr;
    data[1] = value;
    xfer[0].tx_buf = (__u64)data;      // output buffer
    xfer[0].rx_buf = (__u64)data;      // input buffer
    xfer[0].len = (__u32)sizeof(data); // length of data to write

    int retv = ioctl(f_dev, SPI_IOC_MESSAGE(1), &xfer);
    if (retv < 0)
    {
        std::cout << "error in spi_write_reg8(): ioctl(SPI_IOC_MESSAGE(2)) return" <<std::endl;
    }
    return retv;
}

int main() {

    initSPI();
    uint8_t whoami = readRegister(0x0F);
    std::cout << static_cast<int>(whoami) << std::endl;
    return 1;
}