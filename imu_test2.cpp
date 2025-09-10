#include <sys/types.h>
#include <time.h>
#include <errno.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <linux/spi/spidev.h>
#include <cstdint>

// ========================
// L3M6DS3 IMU SPI PINS
// ========================
// SDA  - MOSI (Data into IMU)
// DO   - MISO (Data out from IMU)
// SCL  - SCLK (SPI Clock)
// CS   - CE0 (Chip Select for SPI0 on Raspberry Pi)
// VCC  - 3.3V
// GND  - Ground
//
// Raspberry Pi SPI0 GPIO pins (40-pin header):
// MOSI -> Pin 19
// MISO -> Pin 21
// SCLK -> Pin 23
// CE0  -> Pin 24
// 3.3V -> Pin 1 or 17
// GND  -> Pin 6, 9, or 14
//
// SPI Notes:
// - SPI Mode 3 (CPOL=1, CPHA=1) as per datasheet
// - 8 bits per word
// - Max speed 10 MHz (we use 1 MHz for stability)
// ========================

int f_dev;
double acc_lsb_to_g;
double gyro_lsb_to_degsec;

// Initialize SPI
void initSPI() {
    f_dev = open("/dev/spidev0.0", O_RDWR);
    if (f_dev < 0) {
        std::cerr << "ERR: Failed to open SPI device!" << std::endl;
        exit(1);
    }

    uint8_t mode = SPI_MODE_3;      // CPOL=1, CPHA=1
    uint8_t bits = 8;               // 8 bits per word
    uint32_t speed = 1000000;       // 1 MHz for testing

    // Configure SPI parameters
    if(ioctl(f_dev, SPI_IOC_WR_MODE, &mode) < 0) { close(f_dev); exit(1);}
    if(ioctl(f_dev, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) { close(f_dev); exit(1);}
    if(ioctl(f_dev, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) { close(f_dev); exit(1);}
}

// Read a single 8-bit register
uint8_t readRegister(uint8_t reg) {
    uint8_t tx[2] = { reg | 0x80, 0x00 }; // address byte (MSB=1 for read) + dummy
    uint8_t rx[2] = {0};

    struct spi_ioc_transfer tr = {0};
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = 2;
    tr.speed_hz = 1000000;
    tr.bits_per_word = 8;

    int ret = ioctl(f_dev, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        std::cerr << "SPI read failed!" << std::endl;
        return 0;
    }

    return rx[1]; // second byte contains actual register value
}

// Write a single 8-bit register
int writeRegister(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { reg & 0x7F, value }; // MSB=0 for write
    struct spi_ioc_transfer tr = {0};
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = 0;
    tr.len = 2;
    tr.speed_hz = 1000000;
    tr.bits_per_word = 8;

    int ret = ioctl(f_dev, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        std::cerr << "SPI write failed!" << std::endl;
    }
    return ret;
}

// ========================
// Accelerometer Configuration
// ========================
int setAccConfig(int config_num) {
    int status = 0;
    uint8_t ACCEL_CONFIG_ = 0x10;

    switch(config_num) {
        case 0: acc_lsb_to_g = 0.061; status = writeRegister(ACCEL_CONFIG_, 0x20); break; // ±2g
        case 1: acc_lsb_to_g = 0.122; status = writeRegister(ACCEL_CONFIG_, 0xAA); break; // ±4g
        case 2: acc_lsb_to_g = 0.244; status = writeRegister(ACCEL_CONFIG_, 0xAE); break; // ±8g
        case 3: acc_lsb_to_g = 0.488; status = writeRegister(ACCEL_CONFIG_, 0xA6); break; // ±16g
        default: status = -1; break;
    }

    return status;
}

// ========================
// Gyroscope Configuration
// ========================
int setGyroConfig(int config_num) {
    int status = 0;
    uint8_t GYRO_CONFIG_ = 0x11;

    switch(config_num) {
        case 0: gyro_lsb_to_degsec = 8.75; status = writeRegister(GYRO_CONFIG_, 0x20); break;   // ±250 dps
        case 1: gyro_lsb_to_degsec = 17.5; status = writeRegister(GYRO_CONFIG_, 0xA4); break;  // ±500 dps
        case 2: gyro_lsb_to_degsec = 35; status = writeRegister(GYRO_CONFIG_, 0xA8); break;    // ±1000 dps
        case 3: gyro_lsb_to_degsec = 70; status = writeRegister(GYRO_CONFIG_, 0xAC); break;    // ±2000 dps
        default: status = -1; break;
    }

    return status;
}

// ========================
// Fetch accelerometer and gyroscope data
// ========================
void fetchData() {
    int16_t X, Y, Z;

    // Read accelerometer registers (X, Y, Z)
    X = (int16_t)(readRegister(0x29) << 8 | readRegister(0x28));
    Y = (int16_t)(readRegister(0x2B) << 8 | readRegister(0x2A));
    Z = (int16_t)(readRegister(0x2D) << 8 | readRegister(0x2C));

    float accX = ((float)X) * acc_lsb_to_g / 1000;
    float accY = ((float)Y) * acc_lsb_to_g / 1000;
    float accZ = ((float)Z) * acc_lsb_to_g / 1000;

    // Read gyroscope registers (X, Y, Z)
    X = (int16_t)(readRegister(0x23) << 8 | readRegister(0x22));
    Y = (int16_t)(readRegister(0x25) << 8 | readRegister(0x24));
    Z = (int16_t)(readRegister(0x27) << 8 | readRegister(0x26));

    float gyroX = ((float)X) * gyro_lsb_to_degsec / 1000;
    float gyroY = ((float)Y) * gyro_lsb_to_degsec / 1000;
    float gyroZ = ((float)Z) * gyro_lsb_to_degsec / 1000;

    std::cout << "Accel: X=" << accX << "g, Y=" << accY << "g, Z=" << accZ << "g | "
              << "Gyro: X=" << gyroX << "dps, Y=" << gyroY << "dps, Z=" << gyroZ << "dps"
              << std::endl;
}

// ========================
// Main
// ========================
int main() {
    initSPI();

    // Read WHO_AM_I register (0x0F) to verify IMU is connected
    uint8_t id = readRegister(0x0F);
    std::cout << "WHO_AM_I: 0x" << std::hex << (int)id << std::endl;
    if (id != 0x69) {
        std::cerr << "IMU not detected!" << std::endl;
        return 1;
    }

    // Set accelerometer and gyroscope configuration
    if (setAccConfig(0) < 0) {
        std::cerr << "Error setting accelerometer config!" << std::endl;
        return 1;
    }
    if (setGyroConfig(0) < 0) {
        std::cerr << "Error setting gyroscope config!" << std::endl;
        return 1;
    }

    // Fetch and print data 100 times
    for(int i = 0; i < 100; i++) {
        fetchData();
        usleep(100000); // 100 ms delay
    }

    close(f_dev);
    return 0;
}
