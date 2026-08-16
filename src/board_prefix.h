#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <cstring>
// Explicitly, not via Arduino.h: arduino-esp32's pulls <string> in transitively but
// the STM32 Arduino core does not, so omitting it builds on ESP32 and breaks stm32h5.
#include <string>

#ifdef TARGET_STM32H5
#include "esp_compat.h"
#else
#include <esp_log.h>
#endif

/*
 * Board prefix: a short per-DEVICE name, persisted in NVS, prepended to every
 * sampler name.
 *
 * The sampler name is the InfluxDB series key -- the collector tags points with
 * "BLE_" + the name carried in WireSample::dev. That name only has to be unique
 * within one board's firmware, but the DATABASE is shared by every board reporting
 * into it, and two boards running this same firmware both published BLE_TMP117 and
 * BLE_TMP117_2. Two different physical sensors, one series, silently interleaved.
 *
 * This lives in NVS rather than in a build flag on purpose: it is an identity of
 * the BOARD, not of the firmware image. The same binary runs on several units, so a
 * compile-time #ifdef would mean a private build per unit and a rebuild to rename
 * one -- and nothing would stop two units from being flashed with the same image.
 * Set it once per board at provisioning time with the `board-prefix` console
 * command; it survives reflashing, which is exactly the property wanted.
 *
 * DEFAULT IS EMPTY, i.e. names pass through unchanged. That keeps an unprovisioned
 * board behaving exactly as before rather than inventing an identity for it, but it
 * also means "not configured" and "deliberately unprefixed" look the same -- so
 * boardPrefixLogState() announces which one it is at every boot instead of leaving
 * it to be inferred from the series names.
 *
 * LENGTH. WireSample::dev is a hard 16 bytes, filled by name.copy(ws.dev, 16),
 * which TRUNCATES and appends no terminator. Truncation is not merely cosmetic: two
 * names that differ only past character 16 become the SAME series. That is the
 * exact failure this file exists to prevent, so a too-long prefix would reintroduce
 * it one level up. boardPrefixCheckNames() tests the composed names for it.
 */

static constexpr int BOARD_PREFIX_MAGIC_ADDR = 154; // 152/153 are the aux switch
static constexpr int BOARD_PREFIX_ADDR = 155;       // 155..162, NUL-padded
static constexpr uint8_t BOARD_PREFIX_MAGIC = 0x5B;
static constexpr size_t BOARD_PREFIX_MAX = 7; // + NUL, fits 155..162
static constexpr size_t BOARD_NVS_SIZE = 256; // same image every other EEPROM user opens

/// Width of WireSample::dev. A composed name longer than this is truncated on the
/// wire; see the LENGTH note above.
static constexpr size_t BOARD_DEV_FIELD = 16;

static char s_boardPrefix[BOARD_PREFIX_MAX + 1] = {0};

inline bool boardPrefixValid(const char *p) {
    if (!p) return false;
    const size_t n = strlen(p);
    if (n == 0 || n > BOARD_PREFIX_MAX) return false;
    for (size_t i = 0; i < n; ++i) {
        const char c = p[i];
        if (!(isalnum((unsigned char) c) || c == '_')) return false;
    }
    return true;
}

/// Load from NVS into the cache. Absent or malformed magic leaves the prefix EMPTY;
/// a stored blob that fails validation is REJECTED rather than used, because a
/// corrupted prefix would rename every series on the board to something arbitrary.
inline void boardPrefixLoad() {
    char buf[BOARD_PREFIX_MAX + 1] = {0};
    uint8_t magic;
#ifdef TARGET_STM32H5
    magic = EEPROM.read(BOARD_PREFIX_MAGIC_ADDR);
    for (size_t i = 0; i < BOARD_PREFIX_MAX; ++i) buf[i] = (char) EEPROM.read(BOARD_PREFIX_ADDR + i);
#else
    EEPROM.begin(BOARD_NVS_SIZE);
    magic = EEPROM.read(BOARD_PREFIX_MAGIC_ADDR);
    for (size_t i = 0; i < BOARD_PREFIX_MAX; ++i) buf[i] = (char) EEPROM.read(BOARD_PREFIX_ADDR + i);
    EEPROM.end();
#endif
    buf[BOARD_PREFIX_MAX] = '\0';

    s_boardPrefix[0] = '\0';
    if (magic != BOARD_PREFIX_MAGIC) return;
    if (!boardPrefixValid(buf)) {
        ESP_LOGE("board", "stored prefix is invalid, ignoring it (series will be unprefixed)");
        return;
    }
    strncpy(s_boardPrefix, buf, BOARD_PREFIX_MAX);
    s_boardPrefix[BOARD_PREFIX_MAX] = '\0';
}

/// "" when unset. Never null.
inline const char *boardPrefixGet() { return s_boardPrefix; }

/// Persist and update the cache. Returns false without writing anything if the
/// value is not a valid prefix. Verifies by reading back: a prefix that silently
/// failed to store would come back as a different board identity after the next
/// reset, long after the console said it was set.
inline bool boardPrefixSet(const char *p) {
    if (!boardPrefixValid(p)) {
        ESP_LOGE("board", "rejected prefix '%s': need 1..%u chars of [A-Za-z0-9_]",
                 p ? p : "(null)", (unsigned) BOARD_PREFIX_MAX);
        return false;
    }

    char buf[BOARD_PREFIX_MAX] = {0};
    strncpy(buf, p, BOARD_PREFIX_MAX);

#ifdef TARGET_STM32H5
    EEPROM.write(BOARD_PREFIX_MAGIC_ADDR, BOARD_PREFIX_MAGIC);
    for (size_t i = 0; i < BOARD_PREFIX_MAX; ++i) EEPROM.write(BOARD_PREFIX_ADDR + i, (uint8_t) buf[i]);
#else
    EEPROM.begin(BOARD_NVS_SIZE);
    EEPROM.write(BOARD_PREFIX_MAGIC_ADDR, BOARD_PREFIX_MAGIC);
    for (size_t i = 0; i < BOARD_PREFIX_MAX; ++i) EEPROM.write(BOARD_PREFIX_ADDR + i, (uint8_t) buf[i]);
    EEPROM.commit();
    EEPROM.end();
#endif

    boardPrefixLoad();
    if (strcmp(s_boardPrefix, p) != 0) {
        ESP_LOGE("board", "prefix write did NOT stick: wrote '%s', read back '%s'", p, s_boardPrefix);
        return false;
    }
    ESP_LOGI("board", "board prefix set to '%s' (applies to sampler names after reset)", s_boardPrefix);
    return true;
}

/// prefix + "_" + name, or name unchanged when no prefix is set.
inline std::string boardPrefixApply(const char *name) {
    if (s_boardPrefix[0] == '\0') return std::string(name);
    return std::string(s_boardPrefix) + "_" + name;
}

/// Say which of "unset" and "set" is in force. An empty prefix is legitimate, so it
/// must not be silent -- otherwise a board whose provisioning never ran looks
/// identical to one deliberately left unprefixed, and the difference only shows up
/// as two boards fighting over one series in the database.
inline void boardPrefixLogState() {
    if (s_boardPrefix[0] == '\0')
        ESP_LOGW("board", "no board prefix set -- sampler names are unprefixed and may collide "
                          "with another board in InfluxDB. Set one with: board-prefix <name>");
    else
        ESP_LOGI("board", "board prefix '%s'", s_boardPrefix);
}

/// Check composed names for the truncation this file exists to avoid.
///
/// Reports two distinct things, because they are not equally bad:
///  - over-length, which loses characters but may still leave the name unique;
///  - a post-truncation DUPLICATE, which merges two samplers into one InfluxDB
///    series and silently interleaves readings from different sensors.
/// Returns false if any duplicate was found. Names are compared as they appear on
/// the wire -- truncated to BOARD_DEV_FIELD -- not as written in source.
template<typename EntryList>
inline bool boardPrefixCheckNames(const EntryList &names) {
    bool ok = true;
    for (size_t i = 0; i < names.size(); ++i) {
        const std::string &a = names[i];
        if (a.size() > BOARD_DEV_FIELD)
            ESP_LOGW("board", "'%s' is %u chars, truncated to '%s' on the wire (dev[%u])",
                     a.c_str(), (unsigned) a.size(), a.substr(0, BOARD_DEV_FIELD).c_str(),
                     (unsigned) BOARD_DEV_FIELD);
        for (size_t j = i + 1; j < names.size(); ++j) {
            const std::string &b = names[j];
            if (a.substr(0, BOARD_DEV_FIELD) == b.substr(0, BOARD_DEV_FIELD)) {
                ESP_LOGE("board", "NAME COLLISION: '%s' and '%s' are both '%s' after truncation "
                                  "-- they would share one InfluxDB series. Shorten the prefix "
                                  "or the sampler names.",
                         a.c_str(), b.c_str(), a.substr(0, BOARD_DEV_FIELD).c_str());
                ok = false;
            }
        }
    }
    return ok;
}
