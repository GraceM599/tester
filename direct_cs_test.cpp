#include <sys/types.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <linux/spi/spidev.h>
#include <cstdint>

int f_dev;

void testWithManualCS() {
    std::cout << "=== Manual CS Control Test ===" << std::endl;
    std::cout << "This test bypasses automatic CS control" << std::endl;
    std::cout << "Try connecting CS directly to GND for this test" << std::endl;
    
    f_dev = open("/dev/spidev0.0", O_RDWR);
    if (f_dev < 0) {
        std::cout << "Failed to open SPI" << std::endl;
        return;
    }
    
    // Configure SPI
    uint8_t mode = SPI_MODE_0;
    int bits_per_word = 8;
    int speed = 500000;
    
    ioctl(f_dev, SPI_IOC_WR_MODE, &mode);
    ioctl(f_dev, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);
    ioctl(f_dev, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    
    // Try single-byte transfers (some sensors prefer this)
    std::cout << "\n--- Single Byte Transfers ---" << std::endl;
    
    struct spi_ioc_transfer xfer[2];
    memset(xfer, 0, sizeof(xfer));
    
    // First transfer: send register address
    uint8_t reg_addr = 0x8F;  // WHO_AM_I with read bit
    uint8_t dummy = 0x00;
    uint8_t response = 0x00;
    
    xfer[0].tx_buf = (__u64)&reg_addr;
    xfer[0].rx_buf = (__u64)&dummy;
    xfer[0].len = 1;
    
    // Second transfer: read response
    xfer[1].tx_buf = (__u64)&dummy;
    xfer[1].rx_buf = (__u64)&response;
    xfer[1].len = 1;
    
    int ret = ioctl(f_dev, SPI_IOC_MESSAGE(2), xfer);
    if (ret < 0) {
        std::cout << "Single byte transfer failed" << std::endl;
    } else {
        std::cout << "WHO_AM_I response: 0x" << std::hex << (int)response << std::dec << std::endl;
    }
    
    // Try with longer delay between bytes
    std::cout << "\n--- With Inter-byte Delay ---" << std::endl;
    usleep(1000);  // 1ms delay
    
    ret = ioctl(f_dev, SPI_IOC_MESSAGE(2), xfer);
    if (ret >= 0) {
        std::cout << "WHO_AM_I with delay: 0x" << std::hex << (int)response << std::dec << std::endl;
    }
    
    close(f_dev);
}

int main() {
    testWithManualCS();
    
    std::cout << "\n=== Instructions for Manual CS Test ===" << std::endl;
    std::cout << "1. Disconnect CS wire from Pi pin" << std::endl;
    std::cout << "2. Connect sensor CS directly to GND" << std::endl;
    std::cout << "3. Run this test again" << std::endl;
    std::cout << "4. If this works, the issue is CS timing/control" << std::endl;
    
    return 0;
}
