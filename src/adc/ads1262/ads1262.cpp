////////////////////////////////////////////////////////////////////////////////////////////
//    Arduino library for the ADS1262 32-bit ADC breakout board from ProtoCentral
//    Copyright (c) 2020 ProtoCentral Electronics
//
//    This example measures raw capacitance across CHANNEL0 and Gnd and
//    prints on serial terminal
//    
//    this example gives differential voltage across the AN0 And AN1 in mV
//
//    This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//     THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//    NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//    IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//     WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//    SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//  
//     For information on how to use, visit https://github.com/Protocentral/ProtoCentral_ads1262
//
//////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include "adc/ads1262/ads1262.h"
#include <SPI.h>

#include "settings.h"

#define ADS1262_CS_PIN settings.Pin_ADS1220_CS
#define ADS1262_PWDN_PIN settings.Pin_ADS1262_PWDN
#define ADS1262_START_PIN settings.Pin_ADS1262_START

char *ads1262::ads1262_Read_Data() {
    static char SPI_Dummy_Buff[6];
    digitalWrite(ADS1262_CS_PIN, LOW);

    for (int i = 0; i < 6; ++i) {
        SPI_Dummy_Buff[i] = SPI.transfer(CONFIG_SPI_MASTER_DUMMY);
    }

    digitalWrite(ADS1262_CS_PIN, HIGH);

    return SPI_Dummy_Buff;
}

inline bool is_pow2(int n) {
    return (n & (n - 1)) == 0;
}

void ads1262::ads1262_Init() {
    // start the SPI library:
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    //CPOL = 0, CPHA = 1
    SPI.setDataMode(SPI_MODE1);
    // Selecting 1Mhz clock for SPI
    SPI.setClockDivider(SPI_CLOCK_DIV8); // DIV16

    ads1262_Reset();
    delay(100);

    ads1262_Hard_Stop();
    delay(350);

    bool intRef = true;
    bool vbias = false;
    bool pgaBypass = true;
    int pgaGain = 2;

    //assert(!pgaBypass or pgaGain == 1);
    assert(1 <= pgaGain and pgaGain <= 32);
    assert(is_pow2(pgaGain));


    ads1262_Reg_Write(POWER, (vbias << 1) | (intRef));
    ads1262_Reg_Write(INTERFACE, 0b000000000101);

    ads1262_Reg_Write(MODE0, 0x00);   // chop-mode, Run-Mode (continuous), Ref-Reversal
    ads1262_Reg_Write(MODE1, 0b000010000000);    // sensor bias, fir mode enabled
    ads1262_Reg_Write(MODE2,  (pgaBypass << 7) | (uint8_t(log2(pgaGain)) << 4) | 0b0100 /*sps*/);  // FIR mode: 20sps max
    ads1262_Reg_Write(MODE2,  (pgaBypass << 7) | (uint8_t(log2(pgaGain)) << 4) | 0b10000100 /*sps*/);
    //ads1262_Reg_Write(INPMUX, 0b1010);   // neg=AINCOM, pos=AIN0
    ads1262_Reg_Write(INPMUX, 0b00000001);   // neg=AINCOM, pos=AIN0 https://www.ti.com/lit/ds/symlink/ads1262.pdf?#page=94

    ads1262_Reg_Write(OFCAL0, 0x00);    // offset calibration
    ads1262_Reg_Write(OFCAL1, 0x00);    //Ch 1 enabled, gain 6, connected to electrode in
    ads1262_Reg_Write(OFCAL2, 0x00);    //Ch 1 enabled, gain 6, connected to electrode in

    ads1262_Reg_Write(FSCAL0, 0x00);    //Ch 1 enabled, gain 6, connected to electrode in
    ads1262_Reg_Write(FSCAL1, 0x00);    //Ch 1 enabled, gain 6, connected to electrode in
    ads1262_Reg_Write(FSCAL2, 0x40);    //Ch 1 enabled, gain 6, connected to electrode in

    ads1262_Reg_Write(IDACMUX, 0);  // sensor excitation current MUX (not releveant here)
    ads1262_Reg_Write(IDACMAG, 0);    //Ch 1 enabled, gain 6, connected to electrode in


    ads1262_Reg_Write(REFMUX, 0x00);    // internal reference (P)

    // test:
    ads1262_Reg_Write(TDACP, 0x00);    //Ch 1 enabled, gain 6, connected to electrode in
    ads1262_Reg_Write(TDACN, 0x00);    //Ch 1 enabled, gain 6, connected to electrode in
    delay(10);
    ads1262_Reg_Write(GPIOCON, 0x00);    //Ch 1 enabled, gain 6, connected to electrode in
    delay(10);
    ads1262_Reg_Write(GPIODIR, 0x00);    //Ch 1 enabled, gain 6, connected to electrode in
    delay(10);
    ads1262_Reg_Write(GPIODAT, 0x00);    //Ch 1 enabled, gain 6, connected to electrode in
    delay(10);
    /*
    ads1262_Reg_Write(ADC2CFG, 0x00);    //Ch 1 enabled, gain 6, connected to electrode in
    delay(10);
    ads1262_Reg_Write(ADC2MUX, 0x01);    //Ch 1 enabled, gain 6, connected to electrode in
    delay(10);
    ads1262_Reg_Write(ADC2OFC0, 0x00);    //Ch 1 enabled, gain 6, connected to electrode in
    delay(10);
    ads1262_Reg_Write(ADC2OFC1, 0x00);    //Ch 1 enabled, gain 6, connected to electrode in
    delay(10);
    ads1262_Reg_Write(ADC2FSC0, 0x00);    //Ch 1 enabled, gain 6, connected to electrode in
    delay(10);
    ads1262_Reg_Write(ADC2FSC1, 0x40);    //Ch 1 enabled, gain 6, connected to electrode in
    delay(10);
     */
    // ads1262_Start_Read_Data_Continuous();
    delay(10);
    ads1262_Enable_Start();
}

void ads1262::ads1262_Reset() {
    if (ADS1262_PWDN_PIN != 255) {
        ESP_LOGW("ads1262", "Reset by PWDN pin = %i", (int) ADS1262_PWDN_PIN);
        digitalWrite(ADS1262_PWDN_PIN, HIGH);

        delay(100);                    // Wait 100 mSec
        digitalWrite(ADS1262_PWDN_PIN, LOW);
        delay(100);
        digitalWrite(ADS1262_PWDN_PIN, HIGH);
        delay(100);
    } else {
        ESP_LOGW("ads1262", "Reset by command not implemented!");
        //ads1262_Reg_Write(POWER, 0x1 << 4);
    }
}

void ads1262::ads1262_Disable_Start() {
    digitalWrite(ADS1262_START_PIN, LOW);
    delay(20);
}

void ads1262::ads1262_Enable_Start() {
    digitalWrite(ADS1262_START_PIN, HIGH);
    delay(20);
}

void ads1262::ads1262_Hard_Stop(void) {
    digitalWrite(ADS1262_START_PIN, LOW);
    delay(100);
}


void ads1262::ads1262_Start_Data_Conv_Command(void) {
    ads1262_SPI_Command_Data(START);                    // Send 0x08 to the ADS1x9x
}

void ads1262::ads1262_Soft_Stop(void) {
    ads1262_SPI_Command_Data(STOP);                   // Send 0x0A to the ADS1x9x
}

void ads1262::ads1262_Start_Read_Data_Continuous(void) {
    //ads1262_SPI_Command_Data(RDATAC);					// Send 0x10 to the ADS1x9x
}

void ads1262::ads1262_Stop_Read_Data_Continuous(void) {
    //ads1262_SPI_Command_Data(SDATAC);					// Send 0x11 to the ADS1x9x
}

void ads1262::ads1262_SPI_Command_Data(unsigned char data_in) {
    byte data[1];
    //data[0] = data_in;
    digitalWrite(ADS1262_CS_PIN, LOW);
    delay(2);
    digitalWrite(ADS1262_CS_PIN, HIGH);
    delay(2);
    digitalWrite(ADS1262_CS_PIN, LOW);
    delay(2);
    SPI.transfer(data_in);
    delay(2);
    digitalWrite(ADS1262_CS_PIN, HIGH);
}

//Sends a write command to SCP1000
void ads1262::ads1262_Reg_Write(unsigned char READ_WRITE_ADDRESS, unsigned char DATA) {
    // now combine the register address and the command into one byte:
    byte dataToSend = READ_WRITE_ADDRESS | WREG;

    digitalWrite(ADS1262_CS_PIN, LOW);
    delay(2);
    digitalWrite(ADS1262_CS_PIN, HIGH);
    delay(2);
    // take the chip select low to select the device:
    digitalWrite(ADS1262_CS_PIN, LOW);
    delay(2);
    SPI.transfer(dataToSend); //Send register location
    SPI.transfer(0x00);        //number of register to wr
    SPI.transfer(DATA);        //Send value to record into register

    delay(2);
    // take the chip select high to de-select:
    digitalWrite(ADS1262_CS_PIN, HIGH);
}