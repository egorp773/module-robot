#pragma once

// Forced into every pi_bridge C++ translation unit. Classic ESP32 has no spare
// console here: UART0/USB is the Pi COBS transport, UART1 is F9P and UART2 is
// the hoverboard controller. Upstream BNO08x/SparkFun code contains a few
// unconditional `Serial` paths, so the environment maps Serial to this sink
// and defines NO_GLOBAL_SERIAL. This is intentional, not missing logging:
//
//   boot/reset/state/ARM/DISARM -> HELLO_ACK, COMMAND_ACK and STATUS
//   fault/watchdog             -> one-shot FAULT_EVENT and STATUS
//   ESTOP/reset                -> one-shot ESTOP_EVENT and STATUS
//   malformed/CRC/sequence     -> DIAGNOSTICS counters
//   unavailable sensors        -> STATUS ages and POWER_STATUS flags
//
// Text must never be interleaved with framed traffic. C translation units
// intentionally see an empty header.
#ifdef __cplusplus

#include <Stream.h>

class PiBridgeNullPrint final : public Stream {
public:
    using Print::write;
    size_t write(uint8_t) override { return 1u; }
    size_t write(const uint8_t*, size_t length) override { return length; }
    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }
    void flush() override {}
};

extern PiBridgeNullPrint PiBridgeLibraryLogSink;

#endif
