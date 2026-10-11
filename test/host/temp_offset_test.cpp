// Host test for the TMP117 temperature offset (src/temp_offset.h): parse, bounds, apply,
// and that anything invalid means 0 (uncorrected). No Arduino, no PlatformIO:
//   c++ -std=c++17 -Wall -I src test/host/temp_offset_test.cpp -o /tmp/temp_offset_test && /tmp/temp_offset_test
#include "temp_offset.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); ++failures; } } while (0)

using namespace temp_offset;

static Parse p(const char *s, float &v) { return parse(s, v); }

int main() {
    float v;

    {   // The command this exists for: 0x48 reads 0.145 high.
        CHECK(p("-0.145", v) == Parse::Set && v == -0.145f);
        CHECK(p("  -0.145  ", v) == Parse::Set && v == -0.145f);
        CHECK(p("+0.145", v) == Parse::Set && v == 0.145f);
        CHECK(p("1.5e-1", v) == Parse::Set && std::fabs(v - 0.15f) < 1e-7f);
        CHECK(p("-0.145\r\n", v) == Parse::Set && v == -0.145f);
    }
    {   // Show / clear.
        CHECK(p("", v) == Parse::Show);
        CHECK(p("   ", v) == Parse::Show);
        CHECK(p(nullptr, v) == Parse::Show);
        CHECK(p("clear", v) == Parse::Clear && v == 0.f);
        CHECK(p("0", v) == Parse::Clear);
        CHECK(p("-0.0", v) == Parse::Clear);
    }
    {   // Garbage must NOT become 0 (= a silent clear) the way String::toFloat() would.
        CHECK(p("abc", v) == Parse::Malformed && v == 0.f);
        CHECK(p("0.1x", v) == Parse::Malformed);
        CHECK(p("*0.9", v) == Parse::Malformed);   // the U/I multiplicative form
        CHECK(p("0.1 0.2", v) == Parse::Malformed);
        CHECK(p("-", v) == Parse::Malformed);
        CHECK(p(".", v) == Parse::Malformed);
        CHECK(p("0x1p-3", v) == Parse::Malformed);
        CHECK(p("nan", v) == Parse::Malformed);
        CHECK(p("inf", v) == Parse::Malformed);
        CHECK(p("-inf", v) == Parse::Malformed);
        CHECK(p("CLEAR", v) == Parse::Malformed);
        char longArg[64];
        std::memset(longArg, '1', sizeof longArg - 1);
        longArg[sizeof longArg - 1] = 0;
        CHECK(p(longArg, v) == Parse::Malformed);
    }
    {   // Bounds, including the far tail (monotonic: worse never flips back to Set).
        CHECK(p("2", v) == Parse::Set && v == 2.f);
        CHECK(p("-2", v) == Parse::Set && v == -2.f);
        CHECK(p("2.001", v) == Parse::OutOfRange && v == 0.f);
        CHECK(p("-2.001", v) == Parse::OutOfRange);
        CHECK(p("100", v) == Parse::OutOfRange);
        CHECK(p("1e30", v) == Parse::OutOfRange);
        CHECK(p("1e39", v) == Parse::OutOfRange);    // overflows float to inf
        CHECK(p("-1e300", v) == Parse::OutOfRange);
        for (float x = 2.0001f; x < 1e30f; x *= 3.f) {
            CHECK(!valid(x));
            CHECK(!valid(-x));
        }
    }
    {   // valid / sanitize: a stored value that is not a valid offset means 0.
        CHECK(valid(0.f) && valid(-0.145f) && valid(2.f) && valid(-2.f));
        CHECK(!valid(NAN) && !valid(INFINITY) && !valid(-INFINITY));
        CHECK(!valid(2.0001f) && !valid(-2.0001f));
        CHECK(sanitize(-0.145f) == -0.145f);
        CHECK(sanitize(NAN) == 0.f);
        CHECK(sanitize(INFINITY) == 0.f);
        CHECK(sanitize(3.f) == 0.f);
        CHECK(sanitize(-1e30f) == 0.f);
        // Typical erased/garbage bit patterns read as a float.
        const uint32_t patterns[] = {0xFFFFFFFFu, 0x7FC00000u, 0xDEADBEEFu, 0x7F800000u};
        for (uint32_t bits : patterns) {
            float f;
            std::memcpy(&f, &bits, sizeof f);
            CHECK(sanitize(f) == 0.f);
        }
    }
    {   // apply
        CHECK(apply(19.645f, -0.145f) == 19.645f + -0.145f);
        CHECK(std::fabs(apply(19.645f, -0.145f) - 19.5f) < 1e-5f);
        CHECK(apply(25.f, 0.f) == 25.f);
        CHECK(apply(25.f, NAN) == 25.f);         // corrupt offset: uncorrected, not NaN
        CHECK(apply(25.f, 50.f) == 25.f);        // out-of-range offset: not applied
        CHECK(std::isnan(apply(NAN, -0.145f)));  // failed read stays a failed read
    }
    {   // Audit frames: a value is asserted only when it is KNOWN; unreadable is never 0.
        CHECK(frameValue(Status::Set, -0.145f) == -0.145f);
        CHECK(frameValue(Status::Unset, 0.f) == 0.f);
        CHECK(frameValue(Status::Unset, 1.f) == 0.f);          // Unset is 0 by definition
        CHECK(std::isnan(frameValue(Status::Invalid, 0.f)));   // 0 applied, but not asserted
        CHECK(std::isnan(frameValue(Status::Blocked, 0.f)));
        CHECK(frameDiag(Event::Boot, Status::Set) == 0xCA000101u);
        CHECK(frameDiag(Event::Periodic, Status::Blocked) == 0xCA000403u);
        CHECK((frameDiag(Event::Console, Status::Unset) >> 24) == FRAME_MAGIC);
        CHECK(FRAME_MARKER == '~');
    }
    {   // Host build has no persistent store: store refuses, load is 0/Unset.
        Load st;
        CHECK(load(5, st) == 0.f && st == Load::Unset);
        CHECK(!store(5, -0.145f));
    }

    if (failures) {
        std::printf("%d FAILED\n", failures);
        return 1;
    }
    std::printf("temp_offset_test: all passed\n");
    return 0;
}
