#include <sys/types.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <linux/spi/spidev.h>
#include <cstdint>

int f_dev;

void initSPI() {
    uint8_t mode = SPI_MODE_0;
    int bits_per_word = 8;
    int speed = 1000000;

    f_dev = open("/dev/spidev0.0", O_RDWR);
    if (f_dev < 0) {
        std::cout << "ERR: Failed to open SPI communication on /dev/spidev0.0" << std::endl;
        return;
    }

    if(ioctl(f_dev, SPI_IOC_WR_MODE, &mode) < 0){
        std::cout << "Failed to set SPI mode" << std::endl;
        close(f_dev);
        return;
    }
    if (ioctl(f_dev, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0) {
        std::cout << "Failed to set bits per word" << std::endl;
        close(f_dev);
        return;
    }
    if (ioctl(f_dev, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        std::cout << "Failed to set SPI speed" << std::endl;
        close(f_dev);
        return;
    }
    
    std::cout << "SPI initialized successfully" << std::endl;
}

int readRegister(uint8_t register_add) {
    struct spi_ioc_transfer xfer[1] = {0};

    uint8_t reg_addr = register_add | 0x80;  // Set read bit
    uint8_t data[2];
    data[0] = reg_addr;
    data[1] = 0x00;
    
    xfer[0].tx_buf = (__u64)data;
    xfer[0].rx_buf = (__u64)data;
    xfer[0].len = (__u32)sizeof(data);

    int retv = ioctl(f_dev, SPI_IOC_MESSAGE(1), &xfer);
    if (retv < 0) {
        std::cout << "SPI transfer failed" << std::endl;
        return -1;
    }
    
    return data[1];
}

int main() {
    std::cout << "=== LSM6DS3 SPI Communication Test ===" << std::endl;
    
    initSPI();
    
    // Test different SPI modes if WHO_AM_I fails
    uint8_t modes[] = {SPI_MODE_0, SPI_MODE_1, SPI_MODE_2, SPI_MODE_3};
    const char* mode_names[] = {"MODE_0", "MODE_1", "MODE_2", "MODE_3"};
    
    for (int i = 0; i < 4; i++) {
        std::cout << "\nTesting SPI " << mode_names[i] << "..." << std::endl;
        
        // Reinitialize with different mode
        close(f_dev);
        f_dev = open("/dev/spidev0.0", O_RDWR);
        uint8_t mode = modes[i];
        int bits_per_word = 8;
        int speed = 1000000;
        
        ioctl(f_dev, SPI_IOC_WR_MODE, &mode);
        ioctl(f_dev, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);
        ioctl(f_dev, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
        
        // Try to read WHO_AM_I
        int id = readRegister(0x0F);
        std::cout << "WHO_AM_I: 0x" << std::hex << id << std::dec;
        
        if (id == 0x69) {
            std::cout << " ✓ SUCCESS! LSM6DS3 detected with " << mode_names[i] << std::endl;
            break;
        } else {
            std::cout << " ✗ Failed" << std::endl;
        }
    }
    
    close(f_dev);
    return 0;
}
