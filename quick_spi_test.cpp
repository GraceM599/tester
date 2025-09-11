#include <sys/types.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstdint>

int f_dev;

void testSPIMode(uint8_t mode, int speed, const char* mode_name) {
    f_dev = open("/dev/spidev0.0", O_RDWR);
    if (f_dev < 0) {
        std::cout << "Failed to open SPI device" << std::endl;
        return;
    }
    
    int bits_per_word = 8;
    ioctl(f_dev, SPI_IOC_WR_MODE, &mode);
    ioctl(f_dev, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);
    ioctl(f_dev, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    
    // Try WHO_AM_I
    struct spi_ioc_transfer xfer = {0};
    uint8_t tx_data[2] = {0x8F, 0x00};  // WHO_AM_I with read bit
    uint8_t rx_data[2] = {0x00, 0x00};
    
    xfer.tx_buf = (__u64)tx_data;
    xfer.rx_buf = (__u64)rx_data;
    xfer.len = 2;
    
    int ret = ioctl(f_dev, SPI_IOC_MESSAGE(1), &xfer);
    if (ret < 0) {
        std::cout << mode_name << ": Transfer failed" << std::endl;
    } else {
        std::cout << mode_name << " @ " << speed << "Hz: WHO_AM_I = 0x" 
                  << std::hex << (int)rx_data[1] << std::dec << std::endl;
    }
    
    close(f_dev);
    usleep(10000);  // 10ms delay between tests
}

int main() {
    std::cout << "=== Quick SPI Mode Test ===" << std::endl;
    std::cout << "Testing different SPI modes and speeds..." << std::endl;
    
    uint8_t modes[] = {SPI_MODE_0, SPI_MODE_1, SPI_MODE_2, SPI_MODE_3};
    const char* mode_names[] = {"MODE_0", "MODE_1", "MODE_2", "MODE_3"};
    int speeds[] = {100000, 500000, 1000000};
    
    for (int s = 0; s < 3; s++) {
        std::cout << "\n--- Speed: " << speeds[s] << "Hz ---" << std::endl;
        for (int m = 0; m < 4; m++) {
            testSPIMode(modes[m], speeds[s], mode_names[m]);
        }
    }
    
    std::cout << "\n=== Results Analysis ===" << std::endl;
    std::cout << "• If all show 0x00: Power or MISO connection issue" << std::endl;
    std::cout << "• If all show 0xFF: CS or general wiring issue" << std::endl;
    std::cout << "• If one shows 0x6A/0x69/0x6C: That's your working mode!" << std::endl;
    
    return 0;
}
