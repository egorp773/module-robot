#pragma once

#include <cstdint>

namespace pvtsafety {

// Main-loop publication record shared by Safety, RouteExecutor plumbing and
// AUTO_ALIGN. It identifies a new PVT by publication id/loop, never by a
// cached age value which can be indistinguishable from a later loop.
class PvtPublicationTimeline {
public:
    void publish(uint32_t timestampMs, uint32_t pvtId,
                 uint32_t loopGeneration) {
        timestampMs_ = timestampMs;
        pvtId_ = pvtId;
        loopGeneration_ = loopGeneration;
    }

    uint32_t timestampMs() const { return timestampMs_; }
    uint32_t pvtId() const { return pvtId_; }
    uint32_t loopGeneration() const { return loopGeneration_; }
    bool valid() const { return pvtId_ != 0u && timestampMs_ != 0u; }
    bool publishedInLoop(uint32_t currentLoopGeneration) const {
        return valid() && loopGeneration_ == currentLoopGeneration;
    }
    uint32_t ageMs(uint32_t nowMs) const {
        if (!valid()) return UINT32_MAX;
        return nowMs < timestampMs_ ? 0u : nowMs - timestampMs_;
    }

private:
    uint32_t timestampMs_ = 0u;
    uint32_t pvtId_ = 0u;
    uint32_t loopGeneration_ = 0u;
};

struct AlignStartupRecoveryConditions {
    bool startSettle = false;
    bool fixed = false;
    bool hAccAllowed = false;
    bool pvtPublishedThisLoop = false;
    bool motorsZero = false;
    bool safetyEvaluatedPublishedPvt = false;
};

inline bool requestAlignStartupFreshPvtRecovery(
        const AlignStartupRecoveryConditions& in) {
    return in.startSettle && in.fixed && in.hAccAllowed &&
           in.pvtPublishedThisLoop && in.motorsZero &&
           in.safetyEvaluatedPublishedPvt;
}

// Safety may skip its normal recovery hold only when the retained status is
// specifically an old pvt_stale decision and the freshly recomputed candidate
// permits motion. Other current or retained faults are never bypassed.
inline bool applyFreshPvtStatusImmediately(bool startupRecoveryRequested,
                                           bool retainedPvtStale,
                                           bool candidateAllowsMotion) {
    return startupRecoveryRequested && retainedPvtStale &&
           candidateAllowsMotion;
}

}  // namespace pvtsafety
