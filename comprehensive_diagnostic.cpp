#include <sys/types.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <linux/spi/spidev.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <vector>

class SPITester {
private:
    int f_dev;
    
public:
    SPITester() : f_dev(-1) {}
    
    ~SPITester() {
        if (f_dev >= 0) close(f_dev);
    }
    
    bool initSPI(uint8_t mode, int speed) {
        if (f_dev >= 0) close(f_dev);
        
        f_dev = open("/dev/spidev0.0", O_RDWR);
        if (f_dev < 0) {
            return false;
        }
        
        int bits_per_word = 8;
        
        if (ioctl(f_dev, SPI_IOC_WR_MODE, &mode) < 0 ||
            ioctl(f_dev, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0 ||
            ioctl(f_dev, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
            close(f_dev);
            f_dev = -1;
            return false;
        }
        
        return true;
    }
    
    uint8_t readRegister(uint8_t reg_addr) {
        if (f_dev < 0) return 0xFF;
        
        struct spi_ioc_transfer xfer = {0};
        uint8_t tx_data[2] = {(uint8_t)(reg_addr | 0x80), 0x00};
        uint8_t rx_data[2] = {0x00, 0x00};
        
        xfer.tx_buf = (__u64)tx_data;
        xfer.rx_buf = (__u64)rx_data;
        xfer.len = 2;
        
        if (ioctl(f_dev, SPI_IOC_MESSAGE(1), &xfer) < 0) {
            return 0xFF;
        }
        
        return rx_data[1];
    }
    
    bool writeRegister(uint8_t reg_addr, uint8_t value) {
        if (f_dev < 0) return false;
        
        struct spi_ioc_transfer xfer = {0};
        uint8_t tx_data[2] = {reg_addr, value};
        uint8_t rx_data[2] = {0x00, 0x00};
        
        xfer.tx_buf = (__u64)tx_data;
        xfer.rx_buf = (__u64)rx_data;
        xfer.len = 2;
        
        return ioctl(f_dev, SPI_IOC_MESSAGE(1), &xfer) >= 0;
    }
};

class I2CTester {
private:
    int f_dev;
    
public:
    I2CTester() : f_dev(-1) {}
    
    ~I2CTester() {
        if (f_dev >= 0) close(f_dev);
    }
    
    bool initI2C(uint8_t device_addr) {
        if (f_dev >= 0) close(f_dev);
        
        f_dev = open("/dev/i2c-1", O_RDWR);
        if (f_dev < 0) {
            return false;
        }
        
        if (ioctl(f_dev, I2C_SLAVE, device_addr) < 0) {
            close(f_dev);
            f_dev = -1;
            return false;
        }
        
        return true;
    }
    
    uint8_t readRegister(uint8_t reg_addr) {
        if (f_dev < 0) return 0xFF;
        
        if (write(f_dev, &reg_addr, 1) != 1) {
            return 0xFF;
        }
        
        usleep(1000);  // Small delay
        
        uint8_t data;
        if (read(f_dev, &data, 1) != 1) {
            return 0xFF;
        }
        
        return data;
    }
};

void testSPIHardware() {
    std::cout << "\n=== SPI Hardware Test ===" << std::endl;
    
    SPITester spi;
    
    // Test different SPI modes and speeds
    uint8_t modes[] = {SPI_MODE_0, SPI_MODE_1, SPI_MODE_2, SPI_MODE_3};
    const char* mode_names[] = {"MODE_0", "MODE_1", "MODE_2", "MODE_3"};
    int speeds[] = {100000, 500000, 1000000};
    const char* speed_names[] = {"100kHz", "500kHz", "1MHz"};
    
    bool sensor_found = false;
    
    for (int m = 0; m < 4 && !sensor_found; m++) {
        for (int s = 0; s < 3 && !sensor_found; s++) {
            std::cout << "\nTesting " << mode_names[m] << " @ " << speed_names[s] << "... ";
            
            if (spi.initSPI(modes[m], speeds[s])) {
                usleep(10000);  // 10ms stabilization
                
                uint8_t who_am_i = spi.readRegister(0x0F);
                std::cout << "WHO_AM_I: 0x" << std::hex << (int)who_am_i << std::dec;
                
                if (who_am_i == 0x69) {
                    std::cout << " ✅ LSM6DS3 FOUND!";
                    sensor_found = true;
                } else if (who_am_i == 0x6C) {
                    std::cout << " ✅ LSM6DSO FOUND!";
                    sensor_found = true;
                } else if (who_am_i != 0x00 && who_am_i != 0xFF) {
                    std::cout << " ⚠️  Unknown sensor";
                } else {
                    std::cout << " ❌";
                }
                std::cout << std::endl;
                
            } else {
                std::cout << "❌ SPI init failed" << std::endl;
            }
        }
    }
    
    if (!sensor_found) {
        std::cout << "\n❌ No sensor detected via SPI" << std::endl;
    }
}

void testI2CHardware() {
    std::cout << "\n=== I2C Hardware Test ===" << std::endl;
    
    I2CTester i2c;
    uint8_t addresses[] = {0x6A, 0x6B};
    const char* addr_names[] = {"0x6A", "0x6B"};
    
    bool sensor_found = false;
    
    for (int i = 0; i < 2; i++) {
        std::cout << "Testing I2C address " << addr_names[i] << "... ";
        
        if (i2c.initI2C(addresses[i])) {
            uint8_t who_am_i = i2c.readRegister(0x0F);
            std::cout << "WHO_AM_I: 0x" << std::hex << (int)who_am_i << std::dec;
            
            if (who_am_i == 0x69) {
                std::cout << " ✅ LSM6DS3 FOUND!" << std::endl;
                sensor_found = true;
                break;
            } else if (who_am_i == 0x6C) {
                std::cout << " ✅ LSM6DSO FOUND!" << std::endl;
                sensor_found = true;
                break;
            } else if (who_am_i != 0x00 && who_am_i != 0xFF) {
                std::cout << " ⚠️  Unknown sensor" << std::endl;
            } else {
                std::cout << " ❌" << std::endl;
            }
        } else {
            std::cout << "❌ I2C init failed" << std::endl;
        }
    }
    
    if (!sensor_found) {
        std::cout << "❌ No sensor detected via I2C" << std::endl;
    }
}

void testRegisterReading() {
    std::cout << "\n=== Register Reading Test ===" << std::endl;
    
    SPITester spi;
    
    // Use the most common working configuration
    if (!spi.initSPI(SPI_MODE_0, 500000)) {
        std::cout << "❌ Failed to initialize SPI" << std::endl;
        return;
    }
    
    std::cout << "Testing various registers..." << std::endl;
    
    struct {
        uint8_t addr;
        const char* name;
        uint8_t expected_default;
    } registers[] = {
        {0x0F, "WHO_AM_I", 0x69},
        {0x10, "CTRL1_XL", 0x00},
        {0x11, "CTRL2_G", 0x00},
        {0x12, "CTRL3_C", 0x04},
        {0x13, "CTRL4_C", 0x00},
        {0x14, "CTRL5_C", 0x00},
        {0x15, "CTRL6_C", 0x00}
    };
    
    for (int i = 0; i < 7; i++) {
        uint8_t value = spi.readRegister(registers[i].addr);
        std::cout << registers[i].name << " (0x" << std::hex << (int)registers[i].addr 
                  << "): 0x" << (int)value << std::dec;
        
        if (registers[i].addr == 0x0F) {  // WHO_AM_I
            if (value == 0x69 || value == 0x6C) {
                std::cout << " ✅";
            } else {
                std::cout << " ❌ (expected 0x69 or 0x6C)";
            }
        }
        std::cout << std::endl;
    }
}

void performConfigurationTest() {
    std::cout << "\n=== Configuration Test ===" << std::endl;
    
    SPITester spi;
    
    if (!spi.initSPI(SPI_MODE_0, 500000)) {
        std::cout << "❌ Failed to initialize SPI" << std::endl;
        return;
    }
    
    // Test if we can write to configuration registers
    std::cout << "Testing register writes..." << std::endl;
    
    // Read original values
    uint8_t orig_ctrl1 = spi.readRegister(0x10);
    uint8_t orig_ctrl2 = spi.readRegister(0x11);
    
    std::cout << "Original CTRL1_XL: 0x" << std::hex << (int)orig_ctrl1 << std::dec << std::endl;
    std::cout << "Original CTRL2_G: 0x" << std::hex << (int)orig_ctrl2 << std::dec << std::endl;
    
    // Try to write test values
    if (spi.writeRegister(0x10, 0x60)) {  // 416 Hz accelerometer
        usleep(1000);
        uint8_t readback = spi.readRegister(0x10);
        std::cout << "CTRL1_XL write test: wrote 0x60, read 0x" << std::hex << (int)readback << std::dec;
        if (readback == 0x60) {
            std::cout << " ✅" << std::endl;
        } else {
            std::cout << " ❌" << std::endl;
        }
    } else {
        std::cout << "❌ Failed to write CTRL1_XL" << std::endl;
    }
    
    if (spi.writeRegister(0x11, 0x60)) {  // 416 Hz gyroscope
        usleep(1000);
        uint8_t readback = spi.readRegister(0x11);
        std::cout << "CTRL2_G write test: wrote 0x60, read 0x" << std::hex << (int)readback << std::dec;
        if (readback == 0x60) {
            std::cout << " ✅" << std::endl;
        } else {
            std::cout << " ❌" << std::endl;
        }
    } else {
        std::cout << "❌ Failed to write CTRL2_G" << std::endl;
    }
    
    // Restore original values
    spi.writeRegister(0x10, orig_ctrl1);
    spi.writeRegister(0x11, orig_ctrl2);
    
    std::cout << "Registers restored to original values" << std::endl;
}

int main() {
    std::cout << "=== COMPREHENSIVE SENSOR DIAGNOSTIC ===" << std::endl;
    std::cout << "This tool will test both SPI and I2C communication" << std::endl;
    std::cout << "to help identify exactly what's wrong with your setup." << std::endl;
    
    // Test 1: SPI Hardware
    testSPIHardware();
    
    // Test 2: I2C Hardware (alternative)
    testI2CHardware();
    
    // Test 3: Register Reading
    testRegisterReading();
    
    // Test 4: Configuration Test
    performConfigurationTest();
    
    std::cout << "\n=== DIAGNOSTIC SUMMARY ===" << std::endl;
    std::cout << "1. If no sensor found on either SPI or I2C:" << std::endl;
    std::cout << "   → Check power (3.3V), ground, and all connections" << std::endl;
    std::cout << "   → Verify sensor is not damaged" << std::endl;
    std::cout << "\n2. If I2C works but SPI doesn't:" << std::endl;
    std::cout << "   → Use I2C instead (simpler wiring, fewer issues)" << std::endl;
    std::cout << "   → Check CS (chip select) connection for SPI" << std::endl;
    std::cout << "\n3. If sensor found but configuration fails:" << std::endl;
    std::cout << "   → Check if sensor is in the correct mode" << std::endl;
    std::cout << "   → Try software reset first" << std::endl;
    std::cout << "\n4. For best results:" << std::endl;
    std::cout << "   → Use the working_i2c_test.cpp (most reliable)" << std::endl;
    std::cout << "   → Or use fixed_register_reader.cpp for SPI" << std::endl;
    
    return 0;
}
