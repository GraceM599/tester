#include <sys/types.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstdint>

int main() {
    std::cout << "=== WIRING VERIFICATION GUIDE ===" << std::endl;
    std::cout << "\nStep-by-step wiring check for LSM6DS3 + Raspberry Pi:\n" << std::endl;
    
    std::cout << "📍 REQUIRED CONNECTIONS:" << std::endl;
    std::cout << "┌─────────────────┬──────────────────┬─────────────────┐" << std::endl;
    std::cout << "│ Raspberry Pi    │ Physical Pin     │ LSM6DS3 Sensor  │" << std::endl;
    std::cout << "├─────────────────┼──────────────────┼─────────────────┤" << std::endl;
    std::cout << "│ 3.3V Power      │ Pin 1 or Pin 17  │ VCC or VIN      │" << std::endl;
    std::cout << "│ Ground          │ Pin 6, 9, 14, 20│ GND             │" << std::endl;
    std::cout << "│ MOSI (SPI TX)   │ Pin 19 (GPIO 10) │ SDA or SDI      │" << std::endl;
    std::cout << "│ MISO (SPI RX)   │ Pin 21 (GPIO 9)  │ SDO             │" << std::endl;
    std::cout << "│ SCLK (SPI CLK)  │ Pin 23 (GPIO 11) │ SCL or SCLK     │" << std::endl;
    std::cout << "│ CS (Chip Select)│ Pin 24 (GPIO 8)  │ CS              │" << std::endl;
    std::cout << "└─────────────────┴──────────────────┴─────────────────┘" << std::endl;
    
    std::cout << "\n⚠️  CRITICAL CHECKS:" << std::endl;
    std::cout << "1. POWER: Sensor MUST use 3.3V, NOT 5V!" << std::endl;
    std::cout << "2. MISO: This is the most commonly missed connection!" << std::endl;
    std::cout << "3. CS: Must be connected (don't leave floating)" << std::endl;
    
    std::cout << "\n🔧 PHYSICAL VERIFICATION STEPS:" << std::endl;
    std::cout << "\n1. POWER CHECK (use multimeter):" << std::endl;
    std::cout << "   - Measure VCC to GND on sensor: should be 3.3V" << std::endl;
    std::cout << "   - If 0V: power not connected" << std::endl;
    std::cout << "   - If 5V: WRONG! Will damage sensor" << std::endl;
    
    std::cout << "\n2. CONTINUITY CHECK (use multimeter beep mode):" << std::endl;
    std::cout << "   - Pi Pin 1 (3.3V) ↔ Sensor VCC" << std::endl;
    std::cout << "   - Pi Pin 6 (GND)  ↔ Sensor GND" << std::endl;
    std::cout << "   - Pi Pin 19       ↔ Sensor SDA/SDI" << std::endl;
    std::cout << "   - Pi Pin 21       ↔ Sensor SDO" << std::endl;
    std::cout << "   - Pi Pin 23       ↔ Sensor SCL" << std::endl;
    std::cout << "   - Pi Pin 24       ↔ Sensor CS" << std::endl;
    
    std::cout << "\n3. BREADBOARD ISSUES:" << std::endl;
    std::cout << "   - Push all wires firmly into breadboard" << std::endl;
    std::cout << "   - Check for broken breadboard rows" << std::endl;
    std::cout << "   - Try different breadboard rows" << std::endl;
    std::cout << "   - Use solid core wire (not stranded)" << std::endl;
    
    std::cout << "\n4. SENSOR ORIENTATION:" << std::endl;
    std::cout << "   - Check sensor pin labels match your connections" << std::endl;
    std::cout << "   - Some breakout boards have pins in different order" << std::endl;
    
    std::cout << "\n🔍 COMMON WIRING MISTAKES:" << std::endl;
    std::cout << "❌ Using 5V instead of 3.3V (damages sensor)" << std::endl;
    std::cout << "❌ Forgetting MISO connection (can't read data)" << std::endl;
    std::cout << "❌ Swapping MOSI and MISO" << std::endl;
    std::cout << "❌ Wrong CS pin (using Pin 26 instead of Pin 24)" << std::endl;
    std::cout << "❌ Loose breadboard connections" << std::endl;
    std::cout << "❌ Wrong ground (using 5V ground instead of 3.3V ground)" << std::endl;
    
    std::cout << "\n📝 DEBUGGING CHECKLIST:" << std::endl;
    std::cout << "□ 1. Measure 3.3V between sensor VCC and GND" << std::endl;
    std::cout << "□ 2. Test continuity on all 6 connections" << std::endl;
    std::cout << "□ 3. Check Pi GPIO pins are not damaged" << std::endl;
    std::cout << "□ 4. Try different breadboard if using one" << std::endl;
    std::cout << "□ 5. Verify sensor is not damaged (try different sensor)" << std::endl;
    std::cout << "□ 6. Check sensor breakout board for shorts/damage" << std::endl;
    
    std::cout << "\n🎯 QUICK TEST PROCEDURE:" << std::endl;
    std::cout << "1. Disconnect all wires" << std::endl;
    std::cout << "2. Connect ONLY power (3.3V and GND)" << std::endl;
    std::cout << "3. Measure voltage at sensor - should be 3.3V" << std::endl;
    std::cout << "4. If voltage OK, reconnect one wire at a time" << std::endl;
    std::cout << "5. Test after each connection" << std::endl;
    
    std::cout << "\n🔄 ALTERNATIVE WIRING (if Pin 24 CS doesn't work):" << std::endl;
    std::cout << "Try CS on Pin 26 (GPIO 7) and use /dev/spidev0.1 in code" << std::endl;
    
    std::cout << "\n🆘 IF ALL ELSE FAILS:" << std::endl;
    std::cout << "- Try a different LSM6DS3 sensor" << std::endl;
    std::cout << "- Try a different Raspberry Pi" << std::endl;
    std::cout << "- Use I2C instead of SPI (different wiring)" << std::endl;
    
    return 0;
}
