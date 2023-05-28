#pragma once

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

class LCD {

    Adafruit_SSD1306 display;

public:
    LCD() {

    }


    bool init() {
        uint8_t addr = 0x3C;
        display = Adafruit_SSD1306(128, 32, &Wire, -1);
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() != 0)
            return false;
        if (!display.begin(SSD1306_SWITCHCAPVCC, addr)) {
            return false;
        }
        display.clearDisplay();

        display.drawCircle(16, 16, 15, WHITE);

        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.cp437(true);

        return true;
    }

    char _buffer[64];


    void updateValues(const Sample &s) {
        snprintf(_buffer, 64, "%6.4fV %6.4fA %6.4fW", s.u, s.i, s.p());
        display.print(_buffer);
    }

};