#ifndef DEVIL_PERFORMANCECOUNTERS_HPP
#define DEVIL_PERFORMANCECOUNTERS_HPP

#include <cstdint>

namespace devil {

/// \brief Various global statistics for diagnostics purposes.
struct PerformanceCounters {
    PerformanceCounters() { reset(); }

    void reset() {
        serialCommandsSent = 0;
        serialBytesReceived = 0;
    }

    uint32_t serialCommandsSent;
    uint32_t serialBytesReceived;
};
}

#endif
