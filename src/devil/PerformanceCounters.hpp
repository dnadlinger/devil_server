#ifndef DEVIL_PERFORMANCECOUNTERS_HPP
#define DEVIL_PERFORMANCECOUNTERS_HPP

#include <cstdint>

namespace devil {

/// \brief Various global statistics for diagnostics purposes.
struct PerformanceCounters {
    PerformanceCounters() { reset(); }

    /// \brief Resets all counters.
    void reset() {
        serialCommandsSent = 0;
        serialBytesReceived = 0;
    }

    /// \brief The number of commands sent to hardware over the serial bus since
    /// the last reset() call.
    uint32_t serialCommandsSent;

    /// \brief The total amount of data received from connected hardware over
    /// serial bus since the last reset() call.
    uint32_t serialBytesReceived;
};
}

#endif
