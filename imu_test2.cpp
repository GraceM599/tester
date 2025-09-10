#include <sys/types.h>
#include <time.h>
#include <errno.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <string.h>
#include <cmath>
#include <linux/spi/spidev.h>
#include <cstdint>
//linux is on raspi

//L3M6DS3 pins
// Serial Data pin acts as SDI when SPI is used
// SCL (Serial CLick) pin for SPI clock
//CS (Chip Select) pin
//DO pin for serial data output

//PI pins
//MOSI (19, MOSI0)
//MISO (21, MISO0)
//SCLK (Serial CLock, 23, SCLK0)
//CS (Either 24 or 26)

//run if no workie
//int id = readRegister(0x0F);
//std::cout << "WHO_AM_I: " << std::hex << id << std::endl;

double acc_lsb_to_g;
int f_dev;
double gyro_lsb_to_degsec;

// SDA - MOSI  ; DO - MISO0  ; SCL - SCLK0  ; CS - CE0

void initSPI() {

    uint8_t mode = SPI_MODE_0;  // Changed from SPI_MODE_3 to SPI_MODE_0

    int bits_per_word = 8;
    int speed = 1000000;  // Increased speed from 100kHz to 1MHz

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

//read register function
int readRegister(uint8_t register_add){
    struct spi_ioc_transfer xfer[1] = {0};

    // Write message for register address
    uint8_t reg_addr = register_add | 0x80;  // Set read bit
    uint8_t data[2];
    data[0] = reg_addr;
    data[1] = 0x00;  // Changed from 0x05 to 0x00 - don't overwrite response
    xfer[0].tx_buf = (__u64)data;      // output buffer
    xfer[0].rx_buf = (__u64)data;      // input buffer
    xfer[0].len = (__u32)sizeof(data);  // length of data to read

    int retv = ioctl(f_dev, SPI_IOC_MESSAGE(1), &xfer);
    if (retv < 0)
    {
        std::cout << "error in spi_read_reg8(): ioctl(SPI_IOC_MESSAGE(1))" << std::endl;
        return -1;
    }
    // Debug output - remove after testing
    std::cout << "Read reg 0x" << std::hex << (int)register_add << ": 0x" << (int)data[1] << std::dec << std::endl;
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

int setAccConfig(int config_num){
    int status;
    uint8_t ACCEL_CONFIG_ = 0x10;  // LSM6DS3 accelerometer config register

    // First enable the accelerometer by writing to CTRL1_XL (0x10)
    switch(config_num){
        case 0: // range = +- 16 g (matching NASA working code)
            acc_lsb_to_g = 0.488;  // 16g range scaling
            std::cout << "made it to case 0" << std::endl;

            // 0x54 = 0101 0100 = exactly from working NASA I2C code (16g range, normal mode)
            status = writeRegister(ACCEL_CONFIG_, 0x54);
            std::cout << "Write status: " << status << std::endl;

            break;
        case 1: // range = +- 4 g
            acc_lsb_to_g = 0.122;
            status = writeRegister(ACCEL_CONFIG_, 0xAA);
            break;
        case 2: // range = +- 8 g
            acc_lsb_to_g = 0.244;
            status = writeRegister(ACCEL_CONFIG_, 0xAE);
            break;
        case 3: // range = +- 16 g
            acc_lsb_to_g = 0.488;
            status = writeRegister(ACCEL_CONFIG_, 0xA6);
            break;
        default: // error
            status = 1;
            break;
    }
    return status;
}

int setGyroConfig(int config_num){
    int status;

    uint8_t GYRO_CONFIG_ = 0x11;  // LSM6DS3 gyroscope config register CTRL2_G
    switch(config_num){
        case 0:  // range = +- 500 deg/s (matching NASA working code)
            gyro_lsb_to_degsec = 17.5;  // 500 dps range scaling
            // 0x5C = 0101 1100 = exactly from working NASA I2C code (500 dps range, normal mode)
            status = writeRegister(GYRO_CONFIG_, 0x5C);
            break;
        case 1:  // range = +- 500 deg/s
            gyro_lsb_to_degsec = 17.5;
            std::cout << "At gyro case 2" << std::endl;
            status = writeRegister(GYRO_CONFIG_, 0xA4);
            break;
        case 2: // range = +- 1000 deg/s
            gyro_lsb_to_degsec = 35;
            status = writeRegister(GYRO_CONFIG_, 0xA8);
            break;
        case 3: // range = +- 2000 deg/s
            gyro_lsb_to_degsec = 70;
            status = writeRegister(GYRO_CONFIG_, 0xAC);
            break;
        default:
            //put status back to =1
            status = 0;
            break;
    }
    return status;
}

void fetchData(){
    //temporarily changed x-> nx to avoid non defined errors
    int16_t X, Y, Z;
    X = readRegister(0x29) << 8 | readRegister(0x28);
    Y = readRegister(0x2B) << 8 | readRegister(0x2A);
    Z = readRegister(0x2D)  << 8 | readRegister(0x2C);
    std::cout << readRegister(0x29) << std::endl;
    float accX;
    float accY;
    float accZ;
    float gyroX;
    float gyroY;
    float gyroZ;

    double accXoffset = 0;
    double accYoffset = 0;
    double accZoffset = 0;
    // Use NASA working code conversion method (16g range)
    accX = (X / 32768.0f) * 16.0f - accXoffset;
    accY = (Y / 32768.0f) * 16.0f - accYoffset;
    accZ = (Z / 32768.0f) * 16.0f - accZoffset;

    X = readRegister(0x23) << 8 | readRegister(0x22);
    Y = readRegister(0x25) << 8 | readRegister(0x24);
    Z = readRegister(0x27) << 8 | readRegister(0x26);

    //correcct code replaced w/o offset
    /*
    gyroX = ((float)X) * gyro_lsb_to_degsec / 1000- gyroXoffset;
    gyroY = ((float)Y) * gyro_lsb_to_degsec / 1000 - gyroYoffset;
    gyroZ = ((float)Z) * gyro_lsb_to_degsec / 1000 - gyroZoffset;

    */
    // Use NASA working code conversion method (500 dps range)
    gyroX = (X / 32768.0f) * 500.0f;
    gyroY = (Y / 32768.0f) * 500.0f;
    gyroZ = (Z / 32768.0f) * 500.0f;

    //print data
    std::cout << "Accel: X=" << accX
                  << " g, Y=" << accY
                  << " g, Z=" << accZ << " g | "
                  << "Gyro: X=" << gyroX
                  << " dps, Y=" << gyroY
                  << " dps, Z=" << gyroZ << " dps"
                  << std::endl;
}

int main() {

    initSPI();
    
    // First, check if the sensor is responding
    std::cout << "Checking sensor communication..." << std::endl;
    int id = readRegister(0x0F);  // WHO_AM_I register
    std::cout << "WHO_AM_I: 0x" << std::hex << id << std::dec << std::endl;
    
    // Based on working NASA code - they expect 0x6A, not 0x69!
    if (id != 0x6A && id != 0x69 && id != 0x6C) {  // Try multiple valid WHO_AM_I values
        std::cout << "ERROR: Sensor not detected! Expected 0x6A/0x69/0x6C, got 0x" << std::hex << id << std::dec << std::endl;
        std::cout << "Check your wiring and SPI configuration." << std::endl;
        close(f_dev);
        return -1;
    }
    
    if (id == 0x6A) {
        std::cout << "🎉 LSM6DS3 sensor detected successfully (WHO_AM_I: 0x6A)!" << std::endl;
    } else if (id == 0x69) {
        std::cout << "🎉 LSM6DS3/LSM6DS33 sensor detected successfully (WHO_AM_I: 0x69)!" << std::endl;
    } else if (id == 0x6C) {
        std::cout << "🎉 LSM6DSO sensor detected successfully (WHO_AM_I: 0x6C)!" << std::endl;
    }
    
    // Configure accelerometer
    if (setAccConfig(0) != 0) {
        std::cout << "Error while setting accelerometer config" << std::endl;
        close(f_dev);
        return -1;
    }
    if (setGyroConfig(0) != 0) {
        std::cout << "Error while setting gyroscope config" << std::endl;
        close(f_dev);
        return -1;
    }
    
    std::cout << "Configuration complete. Starting data acquisition..." << std::endl;
    
    for (int i = 0; i<100; i++){
        fetchData();
        usleep(100000);  // 100ms delay
    }
    
    close(f_dev);
    return 0;
}