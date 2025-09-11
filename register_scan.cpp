#include <sys/types.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstdint>

int f_dev;

uint8_t readRegister(uint8_t reg) {
    struct spi_ioc_transfer xfer = {0};
    uint8_t tx_data[2] = {(uint8_t)(reg | 0x80), 0x00};
    uint8_t rx_data[2] = {0x00, 0x00};
    
    xfer.tx_buf = (__u64)tx_data;
    xfer.rx_buf = (__u64)rx_data;
    xfer.len = 2;
    
    int ret = ioctl(f_dev, SPI_IOC_MESSAGE(1), &xfer);
    if (ret < 0) return 0xFF;  // Error indicator
    
    return rx_data[1];
}

int main() {
    std::cout << "=== Register Scan Test ===" << std::endl;
    
    f_dev = open("/dev/spidev0.0", O_RDWR);
    if (f_dev < 0) {
        std::cout << "Failed to open SPI device" << std::endl;
        return -1;
    }
    
    uint8_t mode = SPI_MODE_0;
    int bits_per_word = 8;
    int speed = 500000;
    
    ioctl(f_dev, SPI_IOC_WR_MODE, &mode);
    ioctl(f_dev, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);
    ioctl(f_dev, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    
    std::cout << "Scanning common WHO_AM_I register addresses..." << std::endl;
    
    // Common WHO_AM_I register addresses for different sensors
    uint8_t who_am_i_regs[] = {0x0F, 0x75, 0x00, 0x01};
    const char* reg_names[] = {"0x0F (LSM6DS3)", "0x75 (MPU)", "0x00 (alt)", "0x01 (alt)"};
    
    for (int i = 0; i < 4; i++) {
        uint8_t value = readRegister(who_am_i_regs[i]);
        std::cout << "Register " << reg_names[i] << ": 0x" << std::hex << (int)value << std::dec;
        
        if (value != 0x00 && value != 0xFF) {
            std::cout << " ← NON-ZERO VALUE!";
        }
        std::cout << std::endl;
    }
    
    std::cout << "\nScanning first 16 registers for any activity..." << std::endl;
    bool found_activity = false;
    
    for (uint8_t reg = 0x00; reg <= 0x0F; reg++) {
        uint8_t value = readRegister(reg);
        if (value != 0x00 && value != 0xFF) {
            std::cout << "Register 0x" << std::hex << (int)reg << ": 0x" << (int)value 
                      << std::dec << " ← Active!" << std::endl;
            found_activity = true;
        }
    }
    
    if (!found_activity) {
        std::cout << "❌ No register activity detected - likely hardware issue" << std::endl;
        std::cout << "\nPossible causes:" << std::endl;
        std::cout << "1. Sensor not powered (check 3.3V at sensor)" << std::endl;
        std::cout << "2. Sensor damaged" << std::endl;
        std::cout << "3. Wrong sensor type" << std::endl;
        std::cout << "4. MISO not connected" << std::endl;
    } else {
        std::cout << "✅ Found register activity - sensor is responding!" << std::endl;
    }
    
    close(f_dev);
    return 0;
}
