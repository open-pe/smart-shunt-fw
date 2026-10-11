#pragma once

/* Per-sensor additive TEMPERATURE offset for TMP117 samplers (`calibrate <name> T ...`).
 *
 * WHY FIRMWARE-SIDE, NOT THE CHIP'S TEMP_OFFSET REGISTER (0x07): the chip register lives
 * in the part's own EEPROM, so it travels with the sensor and is invisible from here -- a
 * part moved to another board or address would carry a correction nobody can see, and
 * programming it costs EEPROM cycles (tmp117.h point 3 is about exactly that). A stored
 * firmware offset leaves the chip at its factory default, is reported at boot and by
 * `calibrate`, and is cleared with one command.
 *
 * STORAGE: its own NVS namespace, NOT the 256-byte <EEPROM.h> image that holds the U/I
 * slots, aux state and board prefix. That image is over-subscribed (settings.h,
 * aux_switch.h: slots 4 and 8 already have two writers, 17/18 are unusable), so adding a
 * field there would mean either a new alias or shifting existing fields -- and shifting
 * would make every deployed board's U/I calibration load from the wrong bytes. A separate
 * namespace touches no byte of that image. The key is the sampler's storageId (the same
 * per-sampler identity the U/I slot uses; SamplerRegistry rejects duplicates among
 * T-offset samplers at boot).
 *
 * FAILURE DIRECTION: anything that is not a well-formed, in-range stored float -- missing
 * namespace, missing key, wrong blob size, NaN, out of bounds -- loads as 0, i.e. the
 * sensor is published UNCORRECTED. Never a garbage correction.
 *
 * The pure part (bounds, parse, apply) has no Arduino dependency so test/host can build it. */

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace temp_offset {

/// Largest accepted |offset| in degC. TMP117 is +-0.1 degC (-20..50 degC) out of the box;
/// a real inter-sensor offset is a fraction of a degree. Anything near 2 degC is a typo or
/// a mounting problem, not a calibration.
static constexpr float MAX_ABS_C = 2.0f;

inline bool valid(float off) {
    return std::isfinite(off) && std::fabs(off) <= MAX_ABS_C;
}

/// What a stored value means when applied: itself if valid, otherwise 0 (uncorrected).
inline float sanitize(float stored) {
    return valid(stored) ? stored : 0.0f;
}

/// Corrected temperature. A NaN reading stays NaN (failed read must not become a value);
/// an invalid offset is not applied.
inline float apply(float t, float off) {
    return t + sanitize(off);
}

enum class Parse {
    Show,        ///< no argument: report the current offset
    Clear,       ///< "clear", or an offset of exactly 0
    Set,         ///< `out` holds a valid non-zero offset
    Malformed,   ///< not a number (or trailing junk, or the multiplicative '*' form)
    OutOfRange,  ///< a number, but not finite or |x| > MAX_ABS_C
};

/// Parse the argument after `calibrate <sampler> T`. Strict: Arduino's String::toFloat()
/// returns 0 for garbage, which here would silently CLEAR a calibration, so this uses
/// strtof with a full-consumption check instead.
inline Parse parse(const char *arg, float &out) {
    out = 0.0f;
    if (!arg) return Parse::Show;
    while (*arg == ' ' || *arg == '\t') ++arg;
    size_t n = strlen(arg);
    while (n > 0 && (arg[n - 1] == ' ' || arg[n - 1] == '\t' || arg[n - 1] == '\r' || arg[n - 1] == '\n')) --n;
    if (n == 0) return Parse::Show;
    if (n == 5 && strncmp(arg, "clear", 5) == 0) return Parse::Clear;
    if (n >= 32) return Parse::Malformed;

    char buf[32];
    memcpy(buf, arg, n);
    buf[n] = '\0';

    // Only plain decimal numbers: an optional sign, digits, '.', exponent. This rejects
    // '*' (multiplicative form makes no sense for an offset), hex floats, and the
    // "nan"/"inf" spellings strtof would otherwise accept.
    for (size_t k = 0; k < n; ++k) {
        const char c = buf[k];
        if (!((c >= '0' && c <= '9') || c == '+' || c == '-' || c == '.' || c == 'e' || c == 'E'))
            return Parse::Malformed;
    }

    char *end = nullptr;
    const float v = strtof(buf, &end);
    if (end == buf || *end != '\0') return Parse::Malformed;
    if (!valid(v)) return Parse::OutOfRange;
    if (v == 0.0f) return Parse::Clear;
    out = v;
    return Parse::Set;
}

/* ---- Audit frames ---------------------------------------------------------------------
 *
 * Every applied offset is also PUBLISHED, so a stored T can be un-corrected later: at the first
 * telemetry subscription after boot, at every re-subscription, when the console changes it, and
 * every REPUBLISH_US as a backstop against a dropped indication.
 *
 * It rides the ordinary telemetry stream as a v=1 WireSample -- no new characteristic (macOS
 * caches GATT tables) and no new frame version (a pre-2026-10-11 collector asserts ver == 1 and
 * would drop the link). The frame is told apart by its NAME: FRAME_MARKER + the sampler's
 * composed name, e.g. "~t17_48". '~' is outside the board-prefix/sampler-name alphabet
 * ([A-Za-z0-9_]), so it can never collide with a real series. A collector that does not know
 * the marker files it as an ordinary series BLE_~t17_48 (U = offset), which is still auditable.
 *
 * Fields: u = the applied offset in degC -- an explicit 0 when none is stored -- or NaN when the
 * stored value could not be trusted (then 0 is APPLIED but not asserted); t = now; everything
 * else NaN; diag = FRAME_MAGIC << 24 | event << 8 | status. */
static constexpr char FRAME_MARKER = '~';
static constexpr uint32_t FRAME_MAGIC = 0xCA;
static constexpr unsigned long long REPUBLISH_US = 600ull * 1000000ull; // 10 min

enum class Status : uint8_t {
    Unset = 0,    ///< nothing stored: offset is 0, and that 0 is a known fact
    Set = 1,      ///< a valid stored offset is applied
    Invalid = 2,  ///< something stored/NVS unreadable: 0 applied, value NOT known
    Blocked = 3,  ///< storageId shared with another TMP117: 0 applied, value NOT asserted
};

enum class Event : uint8_t {
    Boot = 1,      ///< first telemetry subscription since boot
    Reconnect = 2, ///< a later subscription (collector reconnected)
    Console = 3,   ///< changed by `calibrate <name> T ...`
    Periodic = 4,  ///< REPUBLISH_US backstop
};

/// The u value an audit frame carries: the offset when it is a known fact, else NaN.
inline float frameValue(Status st, float applied) {
    if (st == Status::Set) return applied;
    if (st == Status::Unset) return 0.0f;
    return NAN;
}

inline uint32_t frameDiag(Event ev, Status st) {
    return FRAME_MAGIC << 24 | (uint32_t) ev << 8 | (uint32_t) st;
}

} // namespace temp_offset


/* ---- Persistence (target only) ---------------------------------------------------- */
#if defined(ARDUINO) && !defined(TARGET_STM32H5)
#include <nvs.h>
#include <cstdio>

namespace temp_offset {

static constexpr const char *NVS_NAMESPACE = "tempoff";

inline void nvsKey(uint8_t storageId, char (&key)[8]) {
    snprintf(key, sizeof key, "s%u", (unsigned) storageId);
}

enum class Load { Unset, Ok, Invalid };

/// Stored offset for this sampler, or 0. `status` says which: Unset (never set or cleared),
/// Ok, or Invalid (something is stored but it is not a valid offset, or NVS could not be
/// read -- 0 is applied, but it is not a known value).
inline float load(uint8_t storageId, Load &status) {
    status = Load::Unset;
    nvs_handle_t h;
    // NOT_FOUND here just means no offset was ever set on this board. Any OTHER failure means
    // we do not know -- that must not be reported as "0 stored".
    const esp_err_t oerr = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (oerr == ESP_ERR_NVS_NOT_FOUND) return 0.0f;
    if (oerr != ESP_OK) {
        status = Load::Invalid;
        return 0.0f;
    }
    char key[8];
    nvsKey(storageId, key);
    float v = NAN;
    size_t len = sizeof v;
    const esp_err_t err = nvs_get_blob(h, key, &v, &len);
    nvs_close(h);
    if (err == ESP_ERR_NVS_NOT_FOUND) return 0.0f;
    if (err != ESP_OK || len != sizeof v || !valid(v)) {
        status = Load::Invalid;
        return 0.0f;
    }
    status = Load::Ok;
    return v;
}

/// Persist (off != 0) or erase (off == 0) the offset, then read it back. Returns false if
/// the value is invalid or the write/readback failed; the caller must then not apply it.
inline bool store(uint8_t storageId, float off) {
    if (!valid(off)) return false;
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return false;
    char key[8];
    nvsKey(storageId, key);
    esp_err_t err;
    if (off == 0.0f) {
        err = nvs_erase_key(h, key);
        if (err == ESP_ERR_NVS_NOT_FOUND) err = ESP_OK; // already clear
    } else {
        err = nvs_set_blob(h, key, &off, sizeof off);
    }
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) return false;

    Load st;
    const float back = load(storageId, st);
    return off == 0.0f ? st == Load::Unset : (st == Load::Ok && back == off);
}

} // namespace temp_offset

#else  // host build or STM32: no persistent store for this

namespace temp_offset {
enum class Load { Unset, Ok, Invalid };
inline float load(uint8_t, Load &status) { status = Load::Unset; return 0.0f; }
inline bool store(uint8_t, float) { return false; } // not supported: caller must reject
} // namespace temp_offset

#endif
