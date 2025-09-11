#include <sys/types.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstdint>

int main() {
    std::cout << "=== Test Without CS Control ===" << std::endl;
    std::cout << "INSTRUCTIONS:" << std::endl;
    std::cout << "1. Disconnect CS wire from Pi Pin 24" << std::endl;
    std::cout << "2. Connect sensor CS directly to GND" << std::endl;
    std::cout << "3. Press Enter when ready..." << std::endl;
    std::cin.get();
    
    int f_dev = open("/dev/spidev0.0", O_RDWR);
    if (f_dev < 0) {
        std::cout << "Failed to open SPI device" << std::endl;
        return -1;
    }
    
    // Configure SPI
    uint8_t mode = SPI_MODE_0;
    int bits_per_word = 8;
    int speed = 500000;
    
    ioctl(f_dev, SPI_IOC_WR_MODE, &mode);
    ioctl(f_dev, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);
    ioctl(f_dev, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    
    std::cout << "\nTesting WHO_AM_I with CS tied to GND..." << std::endl;
    
    // Try WHO_AM_I multiple times
    for (int i = 0; i < 5; i++) {
        struct spi_ioc_transfer xfer = {0};
        uint8_t tx_data[2] = {0x8F, 0x00};  // WHO_AM_I with read bit
        uint8_t rx_data[2] = {0x00, 0x00};
        
        xfer.tx_buf = (__u64)tx_data;
        xfer.rx_buf = (__u64)rx_data;
        xfer.len = 2;
        
        int ret = ioctl(f_dev, SPI_IOC_MESSAGE(1), &xfer);
        if (ret < 0) {
            std::cout << "Attempt " << (i+1) << ": Transfer failed" << std::endl;
        } else {
            std::cout << "Attempt " << (i+1) << ": WHO_AM_I = 0x" 
                      << std::hex << (int)rx_data[1] << std::dec << std::endl;
        }
        
        usleep(100000);  // 100ms delay
    }
    
    close(f_dev);
    
    std::cout << "\nIf this shows 0x6A/0x69/0x6C, then CS timing was the issue!" << std::endl;
    std::cout << "If still 0x00, then it's likely sensor power or hardware issue." << std::endl;
    
    return 0;
}
