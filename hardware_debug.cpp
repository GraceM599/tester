#include <sys/types.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <linux/spi/spidev.h>
#include <cstdint>

int f_dev;

void testSPIBasics() {
    std::cout << "=== Hardware Debug Test ===" << std::endl;
    
    // Test 1: Can we open SPI device?
    f_dev = open("/dev/spidev0.0", O_RDWR);
    if (f_dev < 0) {
        std::cout << "❌ FAILED: Cannot open /dev/spidev0.0" << std::endl;
        std::cout << "   Check: sudo raspi-config -> Interface Options -> SPI -> Enable" << std::endl;
        return;
    } else {
        std::cout << "✅ SUCCESS: SPI device opened" << std::endl;
    }
    
    // Test 2: Basic SPI configuration
    uint8_t mode = SPI_MODE_0;
    int bits_per_word = 8;
    int speed = 100000;  // Start with slower speed
    
    if (ioctl(f_dev, SPI_IOC_WR_MODE, &mode) < 0) {
        std::cout << "❌ FAILED: Cannot set SPI mode" << std::endl;
        close(f_dev);
        return;
    }
    
    if (ioctl(f_dev, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0) {
        std::cout << "❌ FAILED: Cannot set bits per word" << std::endl;
        close(f_dev);
        return;
    }
    
    if (ioctl(f_dev, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        std::cout << "❌ FAILED: Cannot set SPI speed" << std::endl;
        close(f_dev);
        return;
    }
    
    std::cout << "✅ SUCCESS: SPI configured (Mode 0, 100kHz)" << std::endl;
    
    // Test 3: Try simple SPI transfer (loopback test)
    uint8_t tx_data[2] = {0x55, 0xAA};  // Test pattern
    uint8_t rx_data[2] = {0x00, 0x00};
    
    struct spi_ioc_transfer xfer = {0};
    xfer.tx_buf = (__u64)tx_data;
    xfer.rx_buf = (__u64)rx_data;
    xfer.len = 2;
    
    int ret = ioctl(f_dev, SPI_IOC_MESSAGE(1), &xfer);
    if (ret < 0) {
        std::cout << "❌ FAILED: SPI transfer failed" << std::endl;
        close(f_dev);
        return;
    } else {
        std::cout << "✅ SUCCESS: SPI transfer completed" << std::endl;
        std::cout << "   Sent: 0x" << std::hex << (int)tx_data[0] << " 0x" << (int)tx_data[1] << std::dec << std::endl;
        std::cout << "   Received: 0x" << std::hex << (int)rx_data[0] << " 0x" << (int)rx_data[1] << std::dec << std::endl;
    }
    
    // Test 4: Try different speeds
    std::cout << "\n=== Testing Different SPI Speeds ===" << std::endl;
    int speeds[] = {100000, 500000, 1000000, 2000000};
    const char* speed_names[] = {"100kHz", "500kHz", "1MHz", "2MHz"};
    
    for (int i = 0; i < 4; i++) {
        if (ioctl(f_dev, SPI_IOC_WR_MAX_SPEED_HZ, &speeds[i]) < 0) {
            std::cout << "❌ " << speed_names[i] << ": Failed to set speed" << std::endl;
        } else {
            // Try a transfer
            ret = ioctl(f_dev, SPI_IOC_MESSAGE(1), &xfer);
            if (ret < 0) {
                std::cout << "❌ " << speed_names[i] << ": Transfer failed" << std::endl;
            } else {
                std::cout << "✅ " << speed_names[i] << ": Transfer OK" << std::endl;
            }
        }
    }
    
    // Test 5: Try WHO_AM_I with different approaches
    std::cout << "\n=== Testing LSM6DS3 Communication ===" << std::endl;
    
    // Reset to safe settings
    mode = SPI_MODE_0;
    speed = 500000;
    ioctl(f_dev, SPI_IOC_WR_MODE, &mode);
    ioctl(f_dev, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    
    // Method 1: Standard read
    uint8_t who_am_i_cmd[2] = {0x8F, 0x00};  // 0x0F | 0x80 for read
    uint8_t who_am_i_resp[2] = {0x00, 0x00};
    
    xfer.tx_buf = (__u64)who_am_i_cmd;
    xfer.rx_buf = (__u64)who_am_i_resp;
    xfer.len = 2;
    
    ret = ioctl(f_dev, SPI_IOC_MESSAGE(1), &xfer);
    if (ret < 0) {
        std::cout << "❌ WHO_AM_I transfer failed" << std::endl;
    } else {
        std::cout << "📡 WHO_AM_I response: 0x" << std::hex << (int)who_am_i_resp[1] << std::dec << std::endl;
        if (who_am_i_resp[1] == 0x69) {
            std::cout << "🎉 LSM6DS3 DETECTED!" << std::endl;
        } else if (who_am_i_resp[1] == 0x6C) {
            std::cout << "🎉 LSM6DSO DETECTED!" << std::endl;
        } else if (who_am_i_resp[1] == 0x00 || who_am_i_resp[1] == 0xFF) {
            std::cout << "⚠️  No response - check wiring!" << std::endl;
        } else {
            std::cout << "❓ Unknown sensor (expected 0x69 for LSM6DS3)" << std::endl;
        }
    }
    
    // Method 2: Try with different delay
    usleep(1000);  // 1ms delay
    ret = ioctl(f_dev, SPI_IOC_MESSAGE(1), &xfer);
    if (ret >= 0) {
        std::cout << "📡 WHO_AM_I with delay: 0x" << std::hex << (int)who_am_i_resp[1] << std::dec << std::endl;
    }
    
    close(f_dev);
}

int main() {
    testSPIBasics();
    
    std::cout << "\n=== Hardware Checklist ===" << std::endl;
    std::cout << "1. VCC connected to 3.3V (NOT 5V)" << std::endl;
    std::cout << "2. GND connected to ground" << std::endl;
    std::cout << "3. MOSI (Pi pin 19) → SDA/SDI on sensor" << std::endl;
    std::cout << "4. MISO (Pi pin 21) → SDO on sensor" << std::endl;
    std::cout << "5. SCLK (Pi pin 23) → SCL on sensor" << std::endl;
    std::cout << "6. CS (Pi pin 24/CE0) → CS on sensor" << std::endl;
    std::cout << "7. SPI enabled in raspi-config" << std::endl;
    
    std::cout << "\n=== Next Steps ===" << std::endl;
    std::cout << "If WHO_AM_I still returns 0x00 or 0xFF:" << std::endl;
    std::cout << "- Double-check all connections" << std::endl;
    std::cout << "- Try CS on pin 26 (CE1) instead of pin 24 (CE0)" << std::endl;
    std::cout << "- Check sensor power with multimeter (should be 3.3V)" << std::endl;
    std::cout << "- Try a different sensor if available" << std::endl;
    
    return 0;
}
