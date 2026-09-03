#include <Arduino.h>
#include <Wire.h>

#include <InfluxDbClient.h>
#include <map>
#include <vector>

#include "adc/adc_ads.h"
#include "adc/adc_esp.h"
#include "adc/ina226.h"
#include "adc/ina228.h"
#include "adc/ina228_mux.h"
#include "adc/ads1220.h"
#include "adc/ads1262.h"
#include "adc/ads131.h"

#ifdef WITH_LCD
#include "lcd.h"
#endif

#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_ota_ops.h"
#include <WiFi.h>
#include "USB.h"

#include "ble.h"
#include "platform.h"
#include "sampler_registry.h"
#include "console.h"
#include "telemetry.h"
#include "energy_counter.h"
#include "util.h"
#include "adc/tmp117.h"
#ifdef WITH_RELAY_MUX
#include "relay_mux.h"
#include "adc/relay_mux_adc.h"
#endif

InfluxDBClient client;

PowerSampler_ADS ads;
PowerSampler_ADS131 ads131;
PowerSampler_INA226 ina226;
PowerSampler_INA228 ina228_40{0x40};

/* Second INA228 on the XIAO harness, strapped to 0x43 (ADD0 -> SCL). The
 * derived storage slots do not work at this address -- 2 + 3 = 5 is TMP117
 * 0x48 and 8 + 3 = 11 is mux channel B -- so both are passed explicitly from
 * the free high range (see CALIB_SLOT_FIRST_HIGH in settings.h). It has no
 * ALERT pin (Pin_INA22x_ALERT4 = 255) and samples on the 50 ms register-read
 * fallback in hasData(). */
PowerSampler_INA228 ina228_43{0x43, /*calibSlot=*/19, /*resistorSlot=*/20};

#ifdef DUAL_INA228
/* The 0x41 part on the two-INA228 variant is a PLAIN INA228, not a mux backend --
 * there is no TMUX8612 on that board. Its derived slots (3 and 9) are exactly the
 * ones INA228MuxBackend would have used for channel A and its resistor/range, so
 * the two builds never both need them; a board reflashed between variants simply
 * re-reads its own calibration into the same slots. */
PowerSampler_INA228 ina228_41{0x41};
#endif

INA228MuxBackend muxBackend{0x41, settings.Pin_INA22x_ALERT2,
                            settings.Pin_Mux_S1, settings.Pin_Mux_S2, settings.Pin_Mux_Zero};
PowerSampler_MuxChannel mux_chA{muxBackend, INA228MuxBackend::Target::CH_A, 3};
PowerSampler_MuxChannel mux_chB{muxBackend, INA228MuxBackend::Target::CH_B, 11};

/* One per ADD0 strap (GND/V+/SDA/SCL).  All four are registered unconditionally
 * and SamplerRegistry::initAll() drops whichever do not answer -- init() reads
 * DEVICE_ID first and returns false on no response, so an absent part costs one
 * failed transaction at boot and logs "init failed".  That beats hardcoding the
 * populated addresses: rebuilding the firmware is not the right response to
 * moving a strap. */
PowerSampler_TMP117 tmp117{0x48};
PowerSampler_TMP117 tmp117_49{0x49};
PowerSampler_TMP117 tmp117_4a{0x4A};
PowerSampler_TMP117 tmp117_4b{0x4B};

#if defined(SHUNT_ADC_ONLY) && !defined(XIAO_NEST)
/* A separate controller lets another default-address part coexist with bus 0;
 * storage slot 9 is unused by every sampler registered in this build. */
/* NOT on the XIAO: GPIO8 is D9 = J2.4/DIN there, so Wire1 would drive the
 * ADS1262's MOSI, and GPIO3 is D2 = the relay mux REQ_B line. */
static constexpr uint8_t TMP117_I2C2_SDA = 3;
static constexpr uint8_t TMP117_I2C2_SCL = 8;
static constexpr uint8_t TMP117_I2C2_STORAGE_ID = 9;
PowerSampler_TMP117 tmp117_i2c2{0x48, 1, TMP117_I2C2_STORAGE_ID};
#endif

/* pwr-metering shunt-adc board (ADS1262) on J2. Pins are the wiring, not
 * settings.h, because settings.h has no SPI entries for this board.
 *
 * J2 is revB: 1:3V3-IN 2:GND 3:SCLK 4:DIN 5:DOUT/DRDY 6:nCS 7:NC 8:START.
 * There is no dedicated /DRDY -- it rides DOUT (J2.5), which is why the driver
 * holds nCS low forever (SBAS661C 9.4.5). J2.7 MUST be left unconnected: on a
 * revA harness that pin is the isolator's /DRDY OUTPUT, so driving it is
 * output-into-output contention. Check the board silk for the revB marker.
 */
#ifdef XIAO_NEST
/* XIAO ESP32-S3, J2.3..J2.8 wired straight to D10..D5 (2026-09-02).
 *      J2.3 SCLK -> D10 = GPIO9    J2.6 nCS   -> D7 = GPIO44
 *      J2.4 DIN  -> D9  = GPIO8    J2.7 NC    -> not connected
 *      J2.5 DOUT -> D8  = GPIO7    J2.8 START -> D5 = GPIO6
 *
 * GPIO9 is AUX_PIN (aux_switch.h). This board has no aux load switch, so the
 * env builds with -D AUX_PIN_PRESENT=0; without it auxBegin() would pinMode()
 * the SCLK pad to OUTPUT and auxArmDeepSleepHold() would gpio_hold_en() it,
 * putting a held pad in contention with the SPI peripheral's clock.
 *
 * GPIO44 (nCS) is U0RXD and GPIO43 (D6, unused) is U0TXD. Neither is a
 * strapping pin, and the console is USB-CDC, so UART0 is free -- but the ROM
 * boot log still bursts out of GPIO43 at 115200 on every reset. Do not put a
 * signal that must stay quiet at boot on D6. */
static constexpr int8_t SHUNTADC_SCLK = 9, SHUNTADC_DIN = 8, SHUNTADC_DOUT = 7,
                        SHUNTADC_NCS = 44, SHUNTADC_START = 6;
#else
/* WARNING: on the XIAO these five collide with the INA228 alerts and the mux
 * (GPIO4=ALERT2, 5=ALERT3, 6=Mux_S1, 7=Mux_S2). This wiring is the N16R8 clone
 * bring-up map; pick one or the other per board before enabling both. */
static constexpr int8_t SHUNTADC_SCLK = 4, SHUNTADC_DIN = 5, SHUNTADC_DOUT = 6,
                        SHUNTADC_NCS = 7, SHUNTADC_START = 16;
#endif

Ads1262ShuntAdc shuntAdc;
/* Shunt current on J3 (IIN, PAIR_CH0). uPair = PAIR_NONE means no voltage
 * channel: u carries the raw shunt volts and p is pinned to 0. Give it a uPair
 * (J5 = PAIR_CH2, the old wiring) once a divider is actually fitted there, and
 * set dividerRatio at the same time.
 *
 * NO LONGER PARKED: with shuntAdcDcct below also registered, the device scans
 * CH0 <-> CH1. Every pair switch restarts the conversion cycle (td(STDR)
 * doubled to ~104 ms by chop), so EACH channel updates at ~4.8 SPS -- not the
 * 19.15 SPS the mux-parked configuration delivered. Still well above the
 * ~2.5 SPS on-air target.
 *
 * Each published sample is no noisier: it is still ONE conversion at the same
 * gain and filter. What degrades is the uncertainty of any FIXED-DURATION
 * average, by ~2x -- four times fewer CH0 conversions arrive per second, and
 * averaging error falls as sqrt(N). Comment out the DCCT registration below and
 * this channel parks again automatically (the device parks whenever exactly one
 * pair is registered).
 *
 * shuntOhm 2 mOhm: sized for 14 A => 28 mV, 80% of the 35 mV input window note N8
 * derives from G=32, leaving headroom for transients. Nominal, NOT calibrated --
 * gain error is whatever the resistor's tolerance is until it is measured against
 * a reference.
 *
 * storageId 12 continues past the existing samplers. */
PowerSampler_ShuntAdc shuntAdcIn{shuntAdc,
                                 Ads1262ShuntAdc::PAIR_NONE, Ads1262ShuntAdc::PAIR_CH0,
                                 /* shuntOhm = 2 mOhm nominal; dividerRatio 0 = no voltage
                                  * channel, so u stays raw ADC volts. */
                                 0.002f, 0.0f,
                                 /* storageId is a namespace shared by EVERY registered
                                  * sampler -- it indexes the EEPROM calibration slot
                                  * (settings.h: 16 + id*8). Taken: 0 (ADS/ADS131),
                                  * 2 (INA228 0x40), 3/11 (mux A/B), 5/6 (TMP117 0x48/0x49,
                                  * see STORAGE_ID_BASE in tmp117.h), 15 (Dummy). 5 aliased
                                  * TMP117 and would have silently shared its calibration. */
                                 12,
                                 SHUNTADC_SCLK, SHUNTADC_DIN, SHUNTADC_DOUT,
                                 SHUNTADC_NCS, SHUNTADC_START,
                                 /* reportDieTemp: DCCT below shares this ADS1262 but
                                  * deliberately opts out, so this remains the one
                                  * channel carrying the single die sensor -- and it is the
                                  * x-axis for the offset drift the shunt sizing
                                  * depends on. T read nan before this was set. */
                                 true};

/* Offset-drift characterisation: the ADC's INTERNAL short with chop OFF,
 * streamed through the normal sampler path. MUTUALLY EXCLUSIVE with shuntAdcIn
 * -- it parks the ADC in a different configuration -- so exactly one of the two
 * samplers.add() lines below may be active. 16 conversions per point at sinc4
 * 10 SPS, so ~1.6 s per point plus a one-time 400 ms settle (was ~0.8 s on the
 * production FIR @ 20 SPS; see ZERO_MODE1_VALUE in adc/ads1262.h). */
/* storageId 13: the ID indexes the shared EEPROM calibration slot
 * (settings.h: 16 + id*8). 6 aliased tmp117_49 -- tmp117.h computes
 * STORAGE_ID_BASE(5) + (0x49 - ADDR_MIN(0x48)) = 6 -- so both samplers would have
 * silently overwritten each other's calibration. Taken: 0, 2, 3, 5, 6, 11, 12, 15. */
/* avgN = 4: at ZERO_DATA_RATE_CODE (10 SPS) this is 400 ms per published
 * sample, i.e. the 2.5 SPS target. avgN = 16 was 1.6 s -> 0.625 SPS, which is
 * exactly the rate measured on the bench. Averaging deeper here also duplicates
 * work EnergyCounter/MeanWindow already does downstream; what the device-level
 * average buys is a sigma estimate over conversions taken in ONE undisturbed
 * configuration, and 4 is enough for that. The mean's uncertainty goes from
 * sd/4 to sd/2 -- about 3 nV to 7 nV at the measured 13 nV per-conversion
 * sigma, against a budget of 350 nV (10 ppm of 35 mV). */
/* DCCT primary current on J4 (PAIR_CH1): a 500:N current transformer into a
 * 5 Ohm burden resistor across AIN2(+)/AIN3(-). V_burden = I_primary * N/100,
 * so the envelope (2..40 A primary, N = 1..6 turns) spans 20 mV .. 2.4 V. At
 * G=1 the PGA's own output swing (Equation 12) still limits each input pin to
 * roughly -2.2..+2.2 V on these +-2.5 V rails, which a ground-referenced 2.4 V
 * peak clears by ~0.2 V, so this pair runs GAIN 1 with the PGA BYPASSED:
 * rail-to-rail input, +-VREF = +-2.5 V digital full scale. 2.4 V is 96% of FS;
 * prefer a turns
 * count with I*N <= ~200 A-turns when the setup allows headroom. Over-range is
 * ALSO policed digitally (DIAG_PGA_RANGE): the PGA monitors watch the PGA
 * output, which a bypassed signal never passes through, so they cannot be relied
 * on here -- though they are demonstrably not dead in bypass either, a floating
 * J4 having raised PGAH continuously on 2026-08-25.
 * G=1 noise (~1 uV/conversion) is ~50 ppm of the worst-case 20 mV
 * signal -- the DCCT ratio error and the burden's tolerance/tempco dominate.
 * Burden dissipation reaches (40*6/500)^2 * 5 = 1.15 W: fit a >= 3 W low-tempco
 * part, it is the metrology element.
 *
 * TIE THE BURDEN COLD END TO BOARD GND -- not optional. Without a DC path to the
 * board's reference the pair floats out of the +-2.6 V absolute input window,
 * and a CH1 driven HARD out of range corrupts its NEIGHBOUR: on 2026-08-25 it
 * put a 7.2 mV offset on CH0 (3.6 A of phantom shunt current) and made the
 * internal die-temperature read return 1503 degC, because the mux now visits
 * both pairs and the front end had not recovered by the next conversion. A
 * merely FLOATING CH1 does not do this -- only a driven one.
 *
 * FIELD MAPPING (same convention as the other overloaded channels): u is the
 * raw burden voltage in VOLTS at the ADC input (dividerRatio 0 = unscaled,
 * doubles as a range readout), i is the PRIMARY current in AMPERES, p = 0.
 *
 * shuntOhm 0.01 is the N=1 effective transresistance (5 Ohm * 1/500). The
 * turns ratio is applied at runtime through the EXISTING per-sampler
 * calibration: after re-rigging, `calibrate DCCT I <1/N>` on the console
 * (0.5 for N=2 ... 0.1667 for N=6; EEPROM-persisted). i is only correct for
 * the rigged N -- and note a trim calibration lives in the SAME factor, so
 * changing N overwrites trim.
 *
 * storageId 10: taken are 0 (ADS/ADS131), 1 (INA226), 2 (INA228 0x40),
 * 3/11 (mux A/B), 5/6/7 (TMP117 0x48-0x4A), 16 (TMP117 0x4B), 9 (SHUNT_ADC_ONLY
 * TMP117), 12/13/14 (shunt/zero/health), 15 (Dummy), 19/20 (INA228 0x43
 * calib + resistor/range, assigned explicitly -- see ina228_43).
 *
 * Slot 8 is NOT free despite no getStorageId() returning it: PowerSampler_INA228
 * is a SECOND consumer of this same EEPROM namespace, storing resistor/range at
 * 8 + (addr - I2C_A0) (ina228.h:186) independently of its own storageId of 2.
 * Grepping getStorageId() alone therefore understates what is taken -- 10 was
 * checked against both. */
PowerSampler_ShuntAdc shuntAdcDcct{shuntAdc,
                                   Ads1262ShuntAdc::PAIR_NONE, Ads1262ShuntAdc::PAIR_CH1,
                                   0.01f, 0.0f,
                                   10,
                                   SHUNTADC_SCLK, SHUNTADC_DIN, SHUNTADC_DOUT,
                                   SHUNTADC_NCS, SHUNTADC_START,
                                   /* reportDieTemp: CH0 already carries it */ false,
                                   /* pgaGainCode */ 0, /* pgaBypass */ true};

PowerSampler_ShuntAdcZero shuntAdcZero{shuntAdc, 4, 13,
                                       SHUNTADC_SCLK, SHUNTADC_DIN, SHUNTADC_DOUT,
                                       SHUNTADC_NCS, SHUNTADC_START};

/* Front-end health as a recorded series: u = fCLK (Hz), i = AVDD-AVSS (V).
 * Publishes cached values only -- it drives no conversions of its own, so it is
 * free to register alongside whichever measurement channel is active. 30 s is
 * far faster than either quantity can meaningfully move, and the failure this
 * exists to catch ran for 35 minutes before the board died.
 * storageId 14: taken are 0, 2, 3, 5, 6, 11, 12, 13, 15. */
PowerSampler_ShuntAdcHealth shuntAdcHealth{shuntAdc, 30000, 14,
                                           SHUNTADC_SCLK, SHUNTADC_DIN, SHUNTADC_DOUT,
                                           SHUNTADC_NCS, SHUNTADC_START};

#ifdef WITH_RELAY_MUX
/* Coto 3502 relay mux (~/dev/ee/hw/dmm-mux-TMUX862/relay-mux) feeding the
 * ads1262-divider board into ONE ADS1262 pair. Off by default -- build with
 * -D WITH_RELAY_MUX to enable -- because turning it on costs the running rig
 * measurably and depends on wiring that is not on the board yet:
 *
 *   - registering a third pair slows EVERY channel. Two registered pairs already
 *     took SHUNT_ADC and DCCT from 19.15 to ~4.8 SPS each, because a pair switch
 *     restarts the conversion; a third divides the scan again.
 *   - the three control lines and the JST-XH control cable to J4 do not exist yet.
 *
 * PINS ARE THE WIRING, NOT A PREFERENCE. Chosen clear of every pin settings.h
 * declares for this board -- ADS1262 SPI (4/5/6/7/16), both I2C buses (3/8 and
 * 11/12), AUX (9), and the declared-but-unregistered ADS131 map (5/6/7/9/11/12/13)
 * -- but they are still a guess about a harness nobody has built. Confirm against
 * the actual cable before trusting a reading, and change them here, not in
 * settings.h, exactly as the ADS1262 SPI pins above are handled.
 *
 * Which pair the divider lands on is a WIRING FACT, so it is per-board. */
#if defined(RELAY_MUX_ONLY) || defined(XIAO_NEST)
/* XIAO ESP32-S3: relay mux control on D0/D1/D2. The XIAO breaks out just 11
 * GPIOs -- D0..D10 = 1,2,3,4,5,6,43,44,7,8,9 -- and GPIO2/3 are the I2C pins in
 * the XIAO settings branch, which is why these builds skip the I2C bring-up
 * entirely: there is nothing on the bus to find.
 *
 * Clear of this board's ADS1262 SPI (9/8/7/44/6) -- see SHUNTADC_* above. */
static constexpr uint8_t RELAYMUX_ARM = 1, RELAYMUX_REQ_A = 2, RELAYMUX_REQ_B = 3;
#else
static constexpr uint8_t RELAYMUX_ARM = 17, RELAYMUX_REQ_A = 18, RELAYMUX_REQ_B = 21;
#endif

#ifdef XIAO_NEST
/* MEASURED ON THE RISER, not assumed: 1 V into the mux's left input (IN_B open),
 * mux output through the voltage divider, divider into the ADS1262's CH0 = J3.
 * Confirmed by Fab at the bench 2026-09-03. */
#  define RELAYMUX_ADC_PAIR Ads1262ShuntAdc::PAIR_CH0
#else
/* PAIR_CH2 = J5 was the divider's ASSUMED landing point on the N16R8 rig and has
 * never been confirmed against a cable. If the divider is on J6, use PAIR_CH3. */
#  define RELAYMUX_ADC_PAIR Ads1262ShuntAdc::PAIR_CH2
#endif

RelayMux2 relayMux{RELAYMUX_ARM, RELAYMUX_REQ_A, RELAYMUX_REQ_B};
RelayMuxAdcBackend relayMuxAdc{relayMux, shuntAdc, RELAYMUX_ADC_PAIR};

/* storageId 21/22: the EEPROM calibration namespace is shared by every registered
 * sampler AND by INA228's separate resistor/range store. Taken across all builds:
 * 0, 2, 3, 5, 6, 9, 10, 11, 12, 13, 14, 15, 19, 20, plus 8+ for INA228 ranges;
 * 17/18 are permanently unusable (they overlap the aux state and board prefix
 * bytes). 21/22 are the next two free slots above CALIB_SLOT_FIRST_HIGH.
 *
 * dividerRatio 0: the ads1262-divider board's ratio is a bench fact measured
 * against a reference, not a nameplate number, so both channels publish RAW ADC
 * VOLTS until it is set. That is the same contract SHUNT_ADC uses, and it fails
 * visibly (millivolts where volts belong) rather than plausibly. */
PowerSampler_RelayMuxChannel relayMuxChA{relayMuxAdc, RelayMux2::Target::CH_A, 21, 0.0f,
                                         SHUNTADC_SCLK, SHUNTADC_DIN, SHUNTADC_DOUT,
                                         SHUNTADC_NCS, SHUNTADC_START,
                                         /* reportDieTemp: SHUNT_ADC is not registered in
                                          * this build, so without this the ADS1262 die
                                          * sensor goes unrecorded entirely. Exactly one
                                          * channel opts in -- there is one sensor. */
                                         true};
PowerSampler_RelayMuxChannel relayMuxChB{relayMuxAdc, RelayMux2::Target::CH_B, 22, 0.0f,
                                         SHUNTADC_SCLK, SHUNTADC_DIN, SHUNTADC_DOUT,
                                         SHUNTADC_NCS, SHUNTADC_START};
#endif

SamplerRegistry samplers;
BleSrv bleSrv;

volatile bool g_samplingHalted = false;
bool disableWifi = true;
bool wifiTimeSyncOnly = true;

class EspTelemetry : public Telemetry {
public:
    using Telemetry::Telemetry;

private:
    std::vector<WireSample> wireSampleBuf;

public:
    void onIdleSleep() override {
        UART_LOG("Zero power for %llds, sleeping for %llds (aux=%s)",
                 (long long)(IDLE_SLEEP_AFTER_US / 1000000),
                 (long long)(IDLE_SLEEP_WAKE_US / 1000000), auxGet() ? "ON" : "off");
        Serial.flush();
        auxArmDeepSleepHold();
        ESP.deepSleep(IDLE_SLEEP_WAKE_US);
    }

    /* The whole point of the power work is a quantity nobody could see. Reading the
     * serial port RESETS this board (native USB-CDC), so the die temperature cannot be
     * sampled by connecting, asking, and disconnecting -- every such reading is taken
     * ~0.5 s after a reset and describes the reset, not the running board. The only way
     * to watch a *running* board is to pay one reset, hold the port open, and let the
     * board talk. Hence a periodic line rather than a console query.
     *
     * The BLE half is here for the same reason: updateConnParams() is a REQUEST. A
     * central may refuse it or counter-offer, and a refusal is otherwise invisible --
     * the code path runs, nothing errors, and the link quietly keeps the old
     * parameters. Printing what was negotiated is what separates "asked and got it"
     * from "asked and was ignored". */
    void onPrint() override {
        float itvl;
        uint16_t lat, tmo;
        if (ble.connSnapshot(itvl, lat, tmo)) {
            UART_LOG("pwr: cpu %u MHz die %.1f C psram %u B heap %u B | "
                     "ble %.1f ms lat %u (eff %.0f ms) timeout %u ms",
                     (unsigned) getCpuFrequencyMhz(), temperatureRead(),
                     (unsigned) ESP.getPsramSize(),
                     (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                     itvl, lat, itvl * (lat + 1), tmo);
        } else {
            UART_LOG("pwr: cpu %u MHz die %.1f C psram %u B heap %u B | ble no link",
                     (unsigned) getCpuFrequencyMhz(), temperatureRead(),
                     (unsigned) ESP.getPsramSize(),
                     (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        }
    }

    void onSummary(const WireSample &ws) override {
        if (!disableWifi) wireSampleBuf.push_back(ws);
    }

    void onTelemetryFlush() override {
        if (!disableWifi) {
            for (auto &ws : wireSampleBuf)
                influxWritePointUDP(ws.getInfluxDbPoint());
        }
        wireSampleBuf.clear();
    }

    void processConsole() override {
        const uart_port_t uart_num = UART_NUM_0;
        static char buf[128];
        static int buf_pos = 0;
        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t *) &length));
        if (length > 0) {
            length = uart_read_bytes(uart_num, buf + buf_pos, min(length, 127 - buf_pos), 20);
            uart_write_bytes(uart_num, buf + buf_pos, length);
            buf_pos += length;
        }
        auto eol = strchr(buf, '\n');
        while (eol) {
            *eol = 0;
            buf_pos = 0;
            handleConsoleInput(String(buf), registry, ble);
            noteWakeEvent();
            break;
        }

        static String serialBuf;
        if (Serial.available()) {
            auto r = Serial.readString();
            serialBuf += r;
            Serial.print(r);
            Serial.flush();
            int lb;
            while ((lb = serialBuf.indexOf('\n')) != -1) {
                String line = serialBuf.substring(0, lb);
                handleConsoleInput(line, registry, ble);
                serialBuf = serialBuf.substring(lb + 1);
            }
            noteWakeEvent();
        }
    }
};

EspTelemetry *telemetry = nullptr;

/* Console bridge -- see the note in console.h for why these are free functions.
 * Null-safe: the console can be reached before setup() constructs the instance. */
unsigned long telemetryGetIntervalUs() {
    return telemetry ? telemetry->getTelemetryIntervalUs() : 0;
}
bool telemetrySetIntervalUs(unsigned long us) {
    return telemetry && telemetry->setTelemetryIntervalUs(us);
}
unsigned long telemetryMinIntervalUs() {
    return telemetry ? telemetry->minTelemetryIntervalUs() : 0;
}

#ifdef WITH_LCD
LCD lcd;
#endif

static constexpr uint32_t OTA_VALIDATE_UPTIME_MS = 60000;
static constexpr uint64_t BOOT_WATCHDOG_TIMEOUT_US = 30ull * 1000000ull;

extern "C" bool verifyRollbackLater() { return true; }

static esp_timer_handle_t bootWatchdog = nullptr;

static void bootWatchdogFire(void *) { esp_restart(); }

static void bootWatchdogArm() {
    esp_timer_create_args_t args = {};
    args.callback = &bootWatchdogFire;
    args.name = "bootwd";
    args.dispatch_method = ESP_TIMER_TASK;
    if (esp_timer_create(&args, &bootWatchdog) != ESP_OK) {
        ESP_LOGE("main", "boot watchdog could not be created -- a bad OTA will brick this device");
        bootWatchdog = nullptr;
        return;
    }
    esp_timer_start_once(bootWatchdog, BOOT_WATCHDOG_TIMEOUT_US);
}

static void bootWatchdogDisarm() {
    if (!bootWatchdog) return;
    esp_timer_stop(bootWatchdog);
    esp_timer_delete(bootWatchdog);
    bootWatchdog = nullptr;
}

static void markOtaValidIfHealthy() {
    static bool done = false;
    if (done) return;
    unsigned long total = 0;
    for (auto &ec: samplers.counters) total += ec.numSamples();
    static unsigned long snapshot = 0;
    static bool haveSnapshot = false;
    if (!haveSnapshot) {
        if (millis() < OTA_VALIDATE_UPTIME_MS / 2) return;
        snapshot = total;
        haveSnapshot = true;
        return;
    }
    if (millis() < OTA_VALIDATE_UPTIME_MS) return;
    if (total <= snapshot) return;
    done = true;
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(running, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY) {
        if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK)
            ESP_LOGI("main", "OTA image confirmed valid (rollback cancelled)");
        else
            ESP_LOGE("main", "failed to confirm OTA image valid -- a reset will roll back");
    }
}

#define RT_CORE 1
constexpr auto RT_PRIO = 20;

[[noreturn]] void realTimeTask(void *arg);
[[noreturn]] void appTask(void *arg);

void uartInit(int port_num);

/* Clock the CPU down from the board default of 240 MHz.
 *
 * This firmware is I/O bound, not compute bound: two INA228s at ~120 Hz over a
 * 400 kHz I2C bus, a ~2.5 Hz BLE indication, and a realTimeTask that spends most of
 * every pass blocked in a task notification. 240 MHz buys nothing here and costs
 * roughly 25 mA of continuous core current -- which is what makes the part warm to
 * the touch, and which the XIAO's LDO then dissipates a second time as (Vin-3.3)*I.
 *
 * 80 MHz is the FLOOR, not merely a choice. Below it arduino-esp32 switches the CPU
 * to the XTAL and powers the PLL down, which takes the 48 MHz USB-Serial-JTAG clock
 * with it (console gone, no recovery except BOOT+RESET) and drops APB below 80 MHz,
 * re-deriving every UART/I2C/SPI divider that was configured against it. At 80 the
 * PLL stays up and APB stays at 80 MHz, so the peripheral clocks -- ADS1262 hardware
 * SPI included -- are bit-for-bit unchanged. NimBLE also requires >= 80 MHz.
 *
 * Overridable per env (-D CPU_FREQ_MHZ=160) for a board that turns out to need the
 * headroom; the `cpufreq` console command changes it at runtime for A/B testing
 * without a reflash. The default itself lives in settings.h, so the console can
 * report what a reboot returns to. */

void setup(void) {
    bootWatchdogArm();
    /* Before Serial.begin(): setCpuFrequencyMhz() re-derives the baud divider of every
     * HardwareSerial it knows about, and doing it first means nothing has to be
     * re-derived at all. */
    /* Disarm the `download` command's RTC-domain bit on any boot that reaches the
     * app. NOTE WHAT THIS DOES NOT COVER: if the chip is sitting in the ROM
     * downloader, this line never runs, so it cannot rescue a board from there --
     * only a flash (esptool clears the bit itself) or a power cycle can. See the
     * `download` handler in console.h. REG_CLR_BIT, not REG_WRITE(0): the register
     * holds other fields. */
    REG_CLR_BIT(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);

    setCpuFrequencyMhz(CPU_FREQ_MHZ);
    Serial.begin(115200);
    auxBegin();
    uartInit(0);
    delay(500);

    /* PSRAM size is in the banner because it is the ONLY external evidence of
     * whether it came up: it is initialised by the IDF startup before app_main
     * (CONFIG_SPIRAM_BOOT_INIT in the qio_opi sdkconfig), so nothing in this file
     * gets a say, and a build that means to leave it powered down is otherwise
     * indistinguishable from one that silently still initialises it. 0 = down. */
    ESP_LOGI("main", "SmartShunt ESP32-S3 started, CPU %u MHz, die %.1f C, "
                     "psram %u B, heap %u B free (largest block %u B)",
             (unsigned) getCpuFrequencyMhz(), temperatureRead(),
             (unsigned) ESP.getPsramSize(),
             (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned) heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    // Before initAll(): the prefix is baked into each EnergyCounter's name there.
    boardPrefixLoad();
    boardPrefixLogState();

    /* SHUNT_ADC_ONLY: the ADS1262 bring-up board on an OTRONIC ESP32-S3 N16R8 clone.
     * settings.h hardcodes `#define XIAO_ESP32S3 1`, so settings_t always carries
     * the XIAO pin map -- where I2C is GPIO3/2 and the INA228 alerts and mux sit
     * on GPIO4/5/6/7, exactly the pins the ADS1262 SPI uses here. There are no
     * INA228s on the clone, so the mux/INA side stays skipped -- but there ARE
     * TMP117s, so settings.h now carries a SHUNT_ADC_ONLY pin branch (I2C on
     * GPIO11/12, alerts and mux at 255) and the bus is brought up on both builds. */

    /* Physical-layer check while the pins are still plain GPIOs. Not fatal: on the
     * shunt-adc board the ADS1262 is on SPI and must come up even with the I2C
     * harness unplugged. See i2c_check_pins() for what it can and cannot see. */
#if !defined(RELAY_MUX_ONLY) && !defined(XIAO_NEST)
    i2c_check_pins(settings.Pin_I2C_SDA, settings.Pin_I2C_SCL);

    Wire.begin(settings.Pin_I2C_SDA, settings.Pin_I2C_SCL, settings.I2C_Freq);
#elif defined(XIAO_NEST)
    /* No I2C parts on this board at all -- the XIAO carries the shunt-adc board
     * on SPI and nothing else. The SHUNT_ADC_ONLY branch in settings.h names
     * GPIO11/12 for I2C, which the XIAO does not even break out, so bringing
     * Wire up would configure a peripheral onto pads that go nowhere. */
    ESP_LOGI("main", "XIAO_NEST: I2C bring-up skipped, ADS1262 on SPI only");
#else
    /* No I2C parts on the relay-mux bring-up board, and GPIO2/3 -- which the XIAO
     * settings branch names as SCL/SDA -- are the relay REQ lines here. Bringing
     * up Wire would hand those pins to the I2C peripheral and the relays would
     * never switch. */
    ESP_LOGI("main", "RELAY_MUX_ONLY: I2C bring-up skipped, no samplers registered");
#endif

#if defined(SHUNT_ADC_ONLY) && !defined(XIAO_NEST)
    i2c_check_pins(TMP117_I2C2_SDA, TMP117_I2C2_SCL);
    Wire1.begin(TMP117_I2C2_SDA, TMP117_I2C2_SCL, settings.I2C_Freq);
#endif

    if (!disableWifi) {
        connect_wifi_async_once();
        wait_for_wifi();
        timeSync("CET-1CEST,M3.5.0,M10.5.0/3", "de.pool.ntp.org", "time.nis.gov");
        if (wifiTimeSyncOnly) {
            WiFi.disconnect(true);
            WiFiClass::mode(WIFI_OFF);
            disableWifi = true;
            g_wifiRequest = 0;
        } else {
            g_wifiRequest = 1;
        }
    }

    bleSrv.begin();

    if (sizeof(WireSample) != 64) assert(false);
    if (sizeof(Sample) != 32) assert(false);

#ifndef RELAY_MUX_ONLY

#ifdef DUAL_INA228
    /* Two plain parts, no mux, no 0x40. The names keep the series this harness has
     * always published: 0x41 is _2, and 0x43 takes _4 (the retired _3 was the 0x42
     * strap). Registering the mux channels here would be worse than useless -- they
     * would drive D8/D9, which carry this board's vbus-only jumper. */
    samplers.add("ESP32_INA228_2", &ina228_41);
    samplers.add("ESP32_INA228_4", &ina228_43);
#elif !defined(SHUNT_ADC_ONLY)
    samplers.add("ESP32_INA228", &ina228_40);
    samplers.add("ESP32_INA228_4", &ina228_43);
    samplers.add("ESP32_INA228_2A", &mux_chA);
    samplers.add("ESP32_INA228_2B", &mux_chB);
#endif
    /* Outside the SHUNT_ADC_ONLY guard: the N16R8 clone carries TMP117s even though it
     * carries no INA228s.
     *
     * The name is the InfluxDB series key (the collector tags points with
     * "BLE_" + this name), so it has to be unique across every board reporting into
     * the same database -- not just within one board. The XIAO already publishes
     * BLE_TMP117 and BLE_TMP117_2, so the clone's parts take an SA_ (shunt-adc)
     * prefix; the two boards can then never collide even if they populate the same
     * addresses. The XIAO keeps its historical names, so no existing series forks.
     *
     * Address-suffixed rather than ordinal: with parts appearing and disappearing by
     * strap, "TMP117_3" is not stable across boots, whereas the address is the one
     * name that always identifies the same physical part.
     *
     * The board half of the identity is NOT here: SamplerRegistry prepends the
     * NVS-stored board prefix (board_prefix.h), so the same binary gives each unit
     * distinct series without a per-unit build.
     *
     * LENGTH: WireSample::dev is 16 bytes and name.copy(ws.dev, 16) TRUNCATES, so
     * two names differing only past character 16 become one series. "t17" rather
     * than "TMP117" buys the room for a prefix: "ftr_t17_48" is 10. Registration
     * checks the composed names for exactly this, so a bad prefix is reported at
     * boot rather than discovered in the database. */
#ifndef XIAO_NEST
    samplers.add("t17_48", &tmp117);
    samplers.add("t17_49", &tmp117_49);
    samplers.add("t17_4A", &tmp117_4a);
    samplers.add("t17_4B", &tmp117_4b);
#endif
#if defined(SHUNT_ADC_ONLY) && !defined(XIAO_NEST)
    samplers.add("t17b_48", &tmp117_i2c2);
#endif
    /* ONE of these two, never both: they share the ADS1262 and configure it
     * differently. SHUNT_ADC_ZERO is the offset-drift characterisation run
     * (internal short, chop ON since 2026-08-15); SHUNT_ADC is normal measurement. */
    /* The bridge prefixes the sampler name with "BLE_", so these appear as
     * BLE_SHUNT_ADC and BLE_SHUNT_ADC_ZERO. Extra channels: SHUNT_ADC_ch1, ... */
#ifdef WITH_RELAY_MUX
    /* RELAY-MUX BUILD: EXACTLY ONE REGISTERED PAIR, AND THAT IS THE POINT.
     *
     * SHUNT_ADC and DCCT are deliberately not registered here. Ads1262ShuntAdc
     * parks its internal mux whenever exactly one pair is registered, and a parked
     * pair free-runs at the full 19.15 SPS; register a second and every pair switch
     * restarts the conversion (td(STDR) ~104 ms chopped), which is what took those
     * two channels to ~4.8 SPS each. A relay-mux ratio measurement wants the
     * opposite trade: one pair, parked, converting as fast as the part allows, with
     * the CHANNEL selection done by the relays upstream instead of by the ADC's own
     * multiplexer. That is the whole architecture -- multiplexing after the divider
     * so the ADC's gain error and drift cancel in the ratio.
     *
     * The consequence to be aware of: this build measures no shunt current and no
     * DCCT. It is a voltage-ratio instrument, not the general-purpose board.
     *
     * Names: dev[16] truncates and the board prefix is prepended, so "ftr_VMUX_A"
     * (10 chars) leaves room. A and B are the relay mux's own input labels (J1/J2),
     * not a physical ordering -- which is source and which is load is a patching
     * decision, and naming them for a role would go stale the first time the
     * pigtails are swapped. */
    samplers.add("VMUX_A", &relayMuxChA);
    samplers.add("VMUX_B", &relayMuxChB);
#else
    samplers.add("SHUNT_ADC", &shuntAdcIn);
#ifndef XIAO_NEST
    /* dev[16] truncation: "ftr_DCCT" fits whole; a SHUNT_ADC_-prefixed name
     * would collide with SHUNT_ADC's truncation in InfluxDB. */
    /* NOT on the XIAO bring-up board: nothing is rigged on J4/PAIR_CH1, and
     * registering a second pair un-parks the ADS1262 mux -- CH0 drops from
     * 19.15 SPS to ~4.8 because every pair switch restarts the conversion. One
     * registered pair is what keeps this at full rate. */
    samplers.add("DCCT", &shuntAdcDcct);
#endif
#endif
    //samplers.add("SHUNT_ADC_ZERO", &shuntAdcZero);
    //samplers.add("SHUNT_ADC_HEALTH", &shuntAdcHealth);  // fCLK/AVDD diagnostic series; disabled on request

/* DUAL_INA228 must be excluded here, not just from samplers.add(): the backend
 * talks to 0x41, which on that variant is ina228_41's OWN device. Leaving this
 * call in gave the part two drivers -- the backend rewrote its DIAG_ALRT and
 * SHUNT_CAL and attached a second ISR to Pin_INA22x_ALERT2, the same GPIO4 the
 * plain sampler uses -- and drove pinMode(255) three times over, which the HAL
 * logs as "Invalid IO 255 selected" on every boot. */
#if !defined(SHUNT_ADC_ONLY) && !defined(DUAL_INA228)
    if (!muxBackend.init()) {
        ESP_LOGW("main", "INA228 mux backend (0x41) init failed");
    }
#endif

#endif /* !RELAY_MUX_ONLY -- nothing is registered on the bring-up board: no
        * ADS1262, no INA228s, no TMP117s. Registering them would run SPI on pins
        * the XIAO does not have (SHUNTADC_START is GPIO16) and I2C on the relay
        * REQ lines. */

#ifdef WITH_RELAY_MUX
    /* Before initAll(), and independent of it. The relay state machine has to run
     * even when nothing else on the board came up -- on the bring-up board there is
     * no ADS1262, so no sampler registers and hasData() is never called. */
    relayMux.begin();
#endif

    samplers.initAll();
    samplers.startAll();

    if (!disableWifi) {
        client.setConnectionParamsV1("http://homeassistant.local:8086", "open_pe", "home_assistant", "h0me");
        client.setWriteOptions(WriteOptions()
            .writePrecision(WritePrecision::MS)
            .batchSize(200).bufferSize(400)
            .flushInterval(1).retryInterval(0));
    }

    telemetry = new EspTelemetry(samplers, bleSrv);
    telemetry->noteWakeEvent();

    xTaskCreatePinnedToCore(realTimeTask, "loopRt", 4096 * 4, NULL, RT_PRIO, NULL, RT_CORE);
    xTaskCreatePinnedToCore(appTask, "loopy", 4096 * 4, NULL, 1, NULL, RT_CORE - 1);
    if (xTaskCreatePinnedToCore(wifiTask, "wifi", 4096, NULL, 0, NULL, RT_CORE - 1) != pdPASS) {
        ESP_LOGE("main", "wifiTask creation failed");
    }

    bootWatchdogDisarm();
}

[[noreturn]] void realTimeTask(void *arg) {
    (void)arg;
    while (true) {
#ifdef WITH_RELAY_MUX
        /* THE one call site. RelayMux2 is single-threaded by construction and this
         * task owns it; every off-task request goes through its command inbox.
         * Ticked before the halt check so a switching sequence already in flight
         * completes rather than freezing with a coil energised. */
        relayMux.tick();
#endif
        if (g_samplingHalted) {
            vTaskDelay(10);
            continue;
        }
        samplers.updateAll();
#ifdef WITH_RELAY_MUX
        /* YIELD ON EVERY RELAY-MUX BUILD, not just the sampler-less bring-up one.
         *
         * RELAY_MUX_ONLY registers no samplers at all, so updateAll() returns
         * immediately and this loop -- priority 20, pinned to core 1 -- would spin
         * at 100% and starve the idle task into a watchdog reset. But a build that
         * DOES register VMUX_A/VMUX_B has the same hole for 300 ms at a time: while
         * the mux sits in DEAD_TIME + SETTLING, hasDataFor() answers false for both
         * channels by design, updateAll() does nothing, and the loop spins exactly
         * as hard as it does with nothing registered. The window is not an edge
         * case -- it is entered on every single channel switch.
         *
         * Nothing here is time critical: the relay sequence is measured in tens of
         * milliseconds and the ADC converts at ~19 SPS, so a 1-tick yield costs no
         * samples. It is the difference between pacing the loop and betting that
         * some sampler always has work. */
        vTaskDelay(1);
#endif
    }
}

void loop() { vTaskDelay(1000); }

[[noreturn]] void appTask(void *arg) {
    (void)arg;
    while (1) {
        wifi_tick();
        otaBleTick(millis());
        telemetry->update(false);
        if (otaBleActive()) {
            telemetry->noteWakeEvent();
        } else {
            telemetry->checkIdleSleep();
        }
        markOtaValidIfHealthy();
        vTaskDelay(10);
    }
}

const int BUF_SIZE = 1024;
QueueHandle_t uart_queue;

void uartInit(int port_num) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
#if CONFIG_IDF_TARGET_ESP32S3
    const int PIN_TX = 43, PIN_RX = 44;
#else
    const int PIN_TX = 1, PIN_RX = 3;
#endif
    const auto port = (uart_port_t) port_num;
    ESP_ERROR_CHECK(uart_set_pin(port, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_param_config(port, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(port, BUF_SIZE * 2, BUF_SIZE * 2, 10, &uart_queue, 0));
}
