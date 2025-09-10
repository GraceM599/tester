#include <sys/types.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <linux/spi/spidev.h>
#include <cstdint>

int f_dev;

void initSPI(uint8_t mode, int speed) {
    f_dev = open("/dev/spidev0.0", O_RDWR);
    if (f_dev < 0) {
        std::cout << "Failed to open SPI device" << std::endl;
        return;
    }
    
    int bits_per_word = 8;
    ioctl(f_dev, SPI_IOC_WR_MODE, &mode);
    ioctl(f_dev, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);
    ioctl(f_dev, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
}

void probeRegister(uint8_t reg_addr, const char* reg_name) {
    struct spi_ioc_transfer xfer = {0};
    
    // Try reading the register
    uint8_t tx_data[2] = {(uint8_t)(reg_addr | 0x80), 0x00};  // Set read bit
    uint8_t rx_data[2] = {0x00, 0x00};
    
    xfer.tx_buf = (__u64)tx_data;
    xfer.rx_buf = (__u64)rx_data;
    xfer.len = 2;
    
    int ret = ioctl(f_dev, SPI_IOC_MESSAGE(1), &xfer);
    if (ret < 0) {
        std::cout << reg_name << " (0x" << std::hex << (int)reg_addr << "): Transfer failed" << std::dec << std::endl;
    } else {
        std::cout << reg_name << " (0x" << std::hex << (int)reg_addr << "): 0x" << (int)rx_data[1] << std::dec << std::endl;
    }
}

void tryDifferentProtocols() {
    std::cout << "=== Advanced Sensor Probing ===" << std::endl;
    
    // Test different SPI modes and speeds
    uint8_t modes[] = {SPI_MODE_0, SPI_MODE_1, SPI_MODE_2, SPI_MODE_3};
    int speeds[] = {100000, 500000, 1000000};
    const char* mode_names[] = {"MODE_0", "MODE_1", "MODE_2", "MODE_3"};
    const char* speed_names[] = {"100kHz", "500kHz", "1MHz"};
    
    for (int m = 0; m < 4; m++) {
        for (int s = 0; s < 3; s++) {
            std::cout << "\n--- Testing " << mode_names[m] << " at " << speed_names[s] << " ---" << std::endl;
            
            close(f_dev);
            initSPI(modes[m], speeds[s]);
            
            // Add delay for sensor to stabilize
            usleep(10000);  // 10ms
            
            // Try WHO_AM_I
            probeRegister(0x0F, "WHO_AM_I");
            
            // Try other common registers
            probeRegister(0x10, "CTRL1_XL");
            probeRegister(0x11, "CTRL2_G");
            probeRegister(0x12, "CTRL3_C");
        }
    }
    
    close(f_dev);
}

void tryMultipleReads() {
    std::cout << "\n=== Multiple Read Attempts ===" << std::endl;
    
    initSPI(SPI_MODE_0, 500000);
    
    // Try reading WHO_AM_I multiple times with different delays
    for (int i = 0; i < 10; i++) {
        std::cout << "Attempt " << (i+1) << ": ";
        probeRegister(0x0F, "WHO_AM_I");
        usleep(100000);  // 100ms between attempts
    }
    
    close(f_dev);
}

void tryResetSequence() {
    std::cout << "\n=== Trying Sensor Reset ===" << std::endl;
    
    initSPI(SPI_MODE_0, 500000);
    
    // Try to write to reset register (if it exists)
    struct spi_ioc_transfer xfer = {0};
    
    // Try writing to CTRL3_C register (0x12) to perform software reset
    uint8_t reset_cmd[2] = {0x12, 0x01};  // CTRL3_C, SW_RESET bit
    
    xfer.tx_buf = (__u64)reset_cmd;
    xfer.rx_buf = (__u64)reset_cmd;
    xfer.len = 2;
    
    std::cout << "Attempting software reset..." << std::endl;
    int ret = ioctl(f_dev, SPI_IOC_MESSAGE(1), &xfer);
    if (ret < 0) {
        std::cout << "Reset command failed" << std::endl;
    } else {
        std::cout << "Reset command sent" << std::endl;
        
        // Wait for reset to complete
        usleep(100000);  // 100ms
        
        // Try reading WHO_AM_I after reset
        probeRegister(0x0F, "WHO_AM_I after reset");
    }
    
    close(f_dev);
}

void tryDifferentSensors() {
    std::cout << "\n=== Testing for Different Sensor Types ===" << std::endl;
    
    initSPI(SPI_MODE_0, 500000);
    
    // Common IMU sensors and their WHO_AM_I values
    struct {
        const char* name;
        uint8_t expected_value;
    } sensors[] = {
        {"LSM6DS3", 0x69},
        {"LSM6DS33", 0x69},
        {"LSM6DSO", 0x6C},
        {"LSM6DSL", 0x6A},
        {"LSM6DSM", 0x6A},
        {"MPU6000", 0x68},
        {"MPU6050", 0x68},
        {"MPU9250", 0x71},
        {"ICM20602", 0x12},
        {"ICM20948", 0xEA}
    };
    
    probeRegister(0x0F, "WHO_AM_I");
    
    // Try different WHO_AM_I register addresses
    probeRegister(0x75, "WHO_AM_I (MPU style)");
    probeRegister(0x00, "WHO_AM_I (alt addr)");
    
    close(f_dev);
}

void tryRawSPIPatterns() {
    std::cout << "\n=== Raw SPI Pattern Test ===" << std::endl;
    
    initSPI(SPI_MODE_0, 500000);
    
    struct spi_ioc_transfer xfer = {0};
    
    // Test 1: Send known pattern and see what comes back
    uint8_t test_patterns[][2] = {
        {0x8F, 0x00},  // WHO_AM_I read
        {0xFF, 0x00},  // All 1s
        {0x00, 0x00},  // All 0s
        {0xAA, 0x55},  // Alternating pattern
        {0x55, 0xAA}   // Inverse alternating
    };
    
    const char* pattern_names[] = {
        "WHO_AM_I read",
        "All 1s",
        "All 0s", 
        "0xAA, 0x55",
        "0x55, 0xAA"
    };
    
    for (int i = 0; i < 5; i++) {
        uint8_t tx_data[2] = {test_patterns[i][0], test_patterns[i][1]};
        uint8_t rx_data[2] = {0x00, 0x00};
        
        xfer.tx_buf = (__u64)tx_data;
        xfer.rx_buf = (__u64)rx_data;
        xfer.len = 2;
        
        int ret = ioctl(f_dev, SPI_IOC_MESSAGE(1), &xfer);
        if (ret >= 0) {
            std::cout << pattern_names[i] << " -> Response: 0x" << std::hex 
                     << (int)rx_data[0] << " 0x" << (int)rx_data[1] << std::dec << std::endl;
        }
        
        usleep(10000);  // 10ms delay between tests
    }
    
    close(f_dev);
}

int main() {
    std::cout << "=== Comprehensive Sensor Debug ===" << std::endl;
    
    // Test 1: Try all combinations of modes and speeds
    tryDifferentProtocols();
    
    // Test 2: Multiple read attempts
    tryMultipleReads();
    
    // Test 3: Try sensor reset
    tryResetSequence();
    
    // Test 4: Test for different sensor types
    tryDifferentSensors();
    
    // Test 5: Raw SPI pattern analysis
    tryRawSPIPatterns();
    
    std::cout << "\n=== Troubleshooting Suggestions ===" << std::endl;
    std::cout << "1. If all responses are 0x00: Sensor not powered or MISO not connected" << std::endl;
    std::cout << "2. If all responses are 0xFF: CS not working or sensor in wrong mode" << std::endl;
    std::cout << "3. If responses vary but no valid WHO_AM_I: Wrong sensor type or damaged" << std::endl;
    std::cout << "4. If consistent non-zero values: Check sensor datasheet for correct registers" << std::endl;
    
    return 0;
}
