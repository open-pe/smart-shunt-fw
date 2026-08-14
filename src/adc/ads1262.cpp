// DRDY plumbing for the shunt-adc ADS1262 sampler (src/adc/ads1262.h).
//
// Lives in its own translation unit on purpose. On this board the ready line IS
// the data line (schematic note N6: the isolator's single reverse channel
// carries DOUT, so the ready signal rides DOUT/nDRDY), which means:
//
//  - the edge handler must be blind while we clock, or it fires on every one of
//    the 40+ data bits in a conversion read;
//  - the flags it shares with the sampler must be ONE object program-wide.
//    `static` in the header would give each translation unit its own copy;
//    `inline` fixes that but gives an IRAM_ATTR function vague linkage, and the
//    xtensa linker then rejects it with "dangerous relocation: l32r: literal
//    placed after use".

#include "ads1262.h"

volatile bool ads1262_busBusy = false;
volatile bool ads1262_ready = false;
TaskNotification ads1262_notification;

void IRAM_ATTR ads1262_drdy_isr() {
    /* Masked: this edge is one of the SCLK-driven data bits on the shared wire,
     * not a conversion-ready edge (note N11 a). */
    if (ads1262_busBusy) return;
    ads1262_ready = true;
    ads1262_notification.notifyFromIsr();
}

void IRAM_ATTR ads1262_bus_busy(bool busy) {
    if (busy) {
        ads1262_busBusy = true;
    } else {
        /* Discard anything latched during our own clocking BEFORE unmasking --
         * "do not start a read on a stale edge" (note N11 a). */
        ads1262_ready = false;
        ads1262_busBusy = false;
    }
}
