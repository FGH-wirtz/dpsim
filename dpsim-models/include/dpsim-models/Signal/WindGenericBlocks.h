/* Author: Christoph Wirtz <christoph.wirtz@fgh-ma.de>
 * SPDX-FileCopyrightText: 2025 FGH e.V.
 * SPDX-License-Identifier: MPL-2.0
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

#include <dpsim-models/Definitions.h>

namespace CPS {
namespace Signal {
namespace WindGen {

// Fixed-step control blocks ported from the Dynawo 1.5.0 NonElectrical.Blocks
// library, shared by the WECC WT4B and IEC WT4A wind turbine models.

inline Real clampReal(Real u, Real lo, Real hi) {
  return (u > hi) ? hi : ((u < lo) ? lo : u);
}

// Dynawo: NonLinear/VariableLimiter.mo
// y = if u > limit1 then limit1 else if u < limit2 then limit2 else u
struct VariableLimiter {
  Real step(Real u, Real limit2_min, Real limit1_max) const {
    return clampReal(u, limit2_min, limit1_max);
  }
};

// Dynawo: NonLinear/SwitchInteger.mo   y = if u2 then u1 else u3
struct SwitchInteger {
  Int step(Int u1, bool sel, Int u3) const { return sel ? u1 : u3; }
};

// Dynawo: NonLinear/MultiSwitch.mo   y = u[f+1]  (f is 0-based here)
struct MultiSwitch {
  Real step(const std::vector<Real> &u, Int f) const {
    if (f < 0)
      f = 0;
    if (f >= static_cast<Int>(u.size()))
      f = static_cast<Int>(u.size()) - 1;
    return u[static_cast<size_t>(f)];
  }
};

// Dynawo: Continuous/RateLimFirstOrderFreeze.mo
//   der(y) = clamp((k*u - y)/T, dyMin, dyMax); 0 if frozen
struct RateLimFirstOrderFreeze {
  Real mK = 1.0;
  Real mT = 1.0;
  Real mDyMin = -9999.0;
  Real mDyMax = 9999.0;
  Real mY = 0.0;

  void setParameters(Real k, Real T, Real dyMin, Real dyMax) {
    mK = k;
    mT = T;
    mDyMin = dyMin;
    mDyMax = dyMax;
  }
  void initialize(Real y0) { mY = y0; }

  Real step(Real u, Real dt, bool freeze) {
    Real ydot = freeze ? 0.0 : clampReal((mK * u - mY) / mT, mDyMin, mDyMax);
    mY += dt * ydot;
    return mY;
  }
  Real output() const { return mY; }
};

// Dynawo: Continuous/IntegratorSetFreeze.mo
//   der(y) = K*u; 0 if frozen; reinit(y, set) on rising reset
struct IntegratorSetFreeze {
  Real mK = 1.0;
  Real mY = 0.0;
  bool mResetPrev = false;

  void setParameters(Real K) { mK = K; }
  void initialize(Real y0) {
    mY = y0;
    mResetPrev = false;
  }

  Real step(Real u, Real dt, bool freeze, bool reset, Real setValue) {
    if (reset && !mResetPrev) {
      mY = setValue;
    } else if (!freeze) {
      mY += dt * mK * u;
    }
    mResetPrev = reset;
    return mY;
  }
  Real step(Real u, Real dt, bool freeze) {
    return step(u, dt, freeze, false, 0.0);
  }
  Real output() const { return mY; }
};

// Dynawo: Continuous/AbsLimRateLimFirstOrderFreeze.mo
//   PT1 (time constant tI) with rate limit, output limits and freeze
struct AbsLimRateLimFirstOrderFreeze {
  Real mTi = 1.0;
  Real mDyMin = -9999.0;
  Real mDyMax = 9999.0;
  Real mX = 0.0; // integrator state (pre-limit)

  void setParameters(Real tI, Real dyMin, Real dyMax) {
    mTi = tI;
    mDyMin = dyMin;
    mDyMax = dyMax;
  }
  void initialize(Real y0) { mX = y0; }

  Real step(Real u, Real dt, bool freeze, Real yMin, Real yMax) {
    Real yLim = clampReal(mX, yMin, yMax);
    Real rate = freeze ? 0.0 : clampReal((u - yLim) / mTi, mDyMin, mDyMax);
    mX += dt * rate;
    return clampReal(mX, yMin, yMax);
  }
  Real output(Real yMin, Real yMax) const { return clampReal(mX, yMin, yMax); }
};

// Dynawo: Continuous/AbsLimRateLimFeedthroughFreeze.mo
//   absolute limit, then ramp-rate limit; freeze holds the output
struct AbsLimRateLimFeedthroughFreeze {
  Real mDyMin = -9999.0;
  Real mDyMax = 9999.0;
  Real mYMin = -9999.0;
  Real mYMax = 9999.0;
  Real mB = 0.0;    // ramp-limiter state
  Real mYprev = 0.0;

  void setParameters(Real dyMin, Real dyMax, Real yMin, Real yMax) {
    mDyMin = dyMin;
    mDyMax = dyMax;
    mYMin = yMin;
    mYMax = yMax;
  }
  void initialize(Real y0) {
    mB = y0;
    mYprev = y0;
  }

  Real step(Real u, Real dt, bool freeze) {
    Real a = clampReal(u, mYMin, mYMax);
    mB += clampReal(a - mB, dt * mDyMin, dt * mDyMax);
    Real y = freeze ? mYprev : mB;
    mYprev = y;
    return y;
  }
  Real output() const { return mYprev; }
};

// Dynawo: Continuous/IncreaseDetection.mo   y = (u > u_delayed)
struct IncreaseDetection {
  Real mUprev = 0.0;
  void initialize(Real u0) { mUprev = u0; }
  bool step(Real u) {
    bool y = u > mUprev;
    mUprev = u;
    return y;
  }
};

// Active-current recovery after a fault (VDE-AR-N 4110/4120 10.2.3.3): linear
// ramp from the value at the fault end to the command over mT (<= 0: off).
struct ActiveCurrentRecoveryRamp {
  Real mT = 1.0;
  Real mX0 = 0.0;
  Real mTau = 0.0;
  bool mPrevFault = false;
  bool mActive = false;

  void setParameters(Real t) { mT = t; }
  void initialize(Real x0) {
    mX0 = x0;
    mTau = 0.0;
    mPrevFault = mActive = false;
  }
  Real step(bool fault, Real x, Real dt) {
    if (fault) {
      mX0 = x;
      mActive = false;
    } else if (mPrevFault && mT > 0.0) {
      mActive = true;
      mTau = 0.0;
    }
    mPrevFault = fault;
    if (!mActive)
      return x;
    mTau += dt;
    Real s = std::min(1.0, mTau / mT);
    if (s >= 1.0)
      mActive = false;
    return mX0 + (x - mX0) * s;
  }
};

// VDE-AR-N 4110/4120 fault detection and dynamic reactive-current support
// (4120 10.2.3.1 / 10.2.3.3, Bild 13). Not a Dynawo block.
//   mMode 0 = WECC REEC: fault = voltage outside [mBandLo, mBandHi]
//   mMode 1 = VDE triggered: support k*(U1min - U) during the fault, ramped
//             out over mRamp
//   mMode 2 = VDE continuous: support k*(U1min - U) at all times
// U1min: 60 s mean voltage, held during a fault and the lockout after it
// (4120 10.2.3.1 a, Anmerkung 6).
// supportCurrent(): reactive current in p.u. of rated current,
// positive = capacitive (raises the voltage).
struct VdeFaultRideThrough {
  int mMode = 1;
  Real mK = 5.0;
  Real mRamp = 0.1;
  Real mBandLo = 0.9, mBandHi = 1.1;

  static constexpr Real cJump = 0.05;
  static constexpr Real cHyst = 0.005;
  static constexpr Real cTMax = 5.0;
  static constexpr Real cArm = 0.1;
  static constexpr Real cWindow = 60.0;
  static constexpr Real cSample = 0.01;
  static constexpr Real cTauLock = 1.0;

  Real mU1min = 1.0;
  Real mElapsed = 0.0;
  Real mSampleT = 0.0;
  Real mFaultT = 0.0;
  Real mRampT = 0.0;
  Real mIqEnd = 0.0;
  Real mLockRef = 1.0;
  Real mLockT = 0.0;
  bool mFault = false;
  bool mLockout = false;
  bool mRamping = false;
  bool mLeftBand = false;
  bool mEndInBand = false;
  std::deque<Real> mSamples;
  Real mSum = 0.0;

  void setParameters(int mode, Real k, Real ramp, Real bandLo, Real bandHi) {
    mMode = mode;
    mK = k;
    mRamp = ramp;
    mBandLo = bandLo;
    mBandHi = bandHi;
  }
  void initialize(Real u0) {
    mElapsed = mSampleT = mFaultT = mRampT = mIqEnd = mLockT = 0.0;
    mLockRef = u0;
    mFault = mLockout = mRamping = mLeftBand = mEndInBand = false;
    reseed(u0);
  }

  bool step(Real u, Real dt) {
    mElapsed += dt;
    bool inBand = (u >= mBandLo) && (u <= mBandHi);
    if (mMode == 0) {
      mFault = !inBand;
      if (!mFault)
        sample(u, dt);
      return mFault;
    }
    if (mElapsed < cArm)
      return false;
    bool healthy = inBand && std::abs(u - mU1min) < cJump;
    if (!mFault) {
      if (mLockout) {
        mLockRef += (u - mLockRef) * (dt / (dt + cTauLock));
        mLockT += dt;
        if (mRamping) {
          mLockRef = u;
          if (mEndInBand && !inBand)
            startFault();
        } else if (healthy) {
          mLockout = false;
        } else if (std::abs(u - mU1min) - std::abs(mLockRef - mU1min) >= cJump) {
          startFault();
        } else if (mLockT >= cWindow) {
          mLockout = false;
          reseed(u);
        }
      } else if (!healthy) {
        startFault();
      }
    } else {
      mFaultT += dt;
      if (!inBand)
        mLeftBand = true;
      bool recovered = mLeftBand && (u >= mBandLo + cHyst) && (u <= mBandHi - cHyst) &&
                       std::abs(u - mU1min) < cJump - cHyst;
      if (recovered || mFaultT >= cTMax) {
        mFault = false;
        mLockout = true;
        mEndInBand = inBand;
        mLockRef = u;
        mLockT = 0.0;
        mIqEnd = mK * (mU1min - u);
        mRamping = mRamp > 0.0;
        mRampT = 0.0;
      }
    }
    if (mRamping) {
      mRampT += dt;
      if (mRampT >= mRamp)
        mRamping = false;
    }
    if (!mFault && !mRamping && !mLockout)
      sample(u, dt);
    return mFault;
  }

  Real supportCurrent(Real u) const {
    if (mElapsed < cArm)
      return 0.0;
    if (mMode == 2 || mFault)
      return mK * (mU1min - u);
    if (mRamping)
      return mIqEnd * (1.0 - mRampT / mRamp);
    return 0.0;
  }
  Real u1min() const { return mU1min; }
  bool fault() const { return mFault; }

private:
  void startFault() {
    mFault = true;
    mLockout = false;
    mRamping = false;
    mLeftBand = false;
    mFaultT = 0.0;
  }
  void reseed(Real u) {
    mU1min = u;
    mSamples.clear();
    mSamples.push_back(u);
    mSum = u;
  }
  void sample(Real u, Real dt) {
    if (mElapsed < cArm)
      return;
    mSampleT += dt;
    if (mSampleT < cSample)
      return;
    mSampleT = 0.0;
    mSamples.push_back(u);
    mSum += u;
    while (mSamples.size() > static_cast<size_t>(cWindow / cSample)) {
      mSum -= mSamples.front();
      mSamples.pop_front();
    }
    mU1min = mSum / static_cast<Real>(mSamples.size());
  }
};

/// VDE-AR-N 4120 negative-sequence reactive current Delta i2 = k Delta u2,
/// inductive (injected j k Delta u2, generator convention). u2 from the
/// envelope E and its value a quarter period earlier,
/// E(t) - E(t - T/4) = 2 conj(U2) e^-2jwt, low-pass filtered. Released by the
/// positive-sequence fault state or a line-line voltage outside 0.9..1.1 pu.
struct NegativeSequenceSupport {
  Real mK = 0.0;
  Real mTau = 0.005;
  Real mTauMeas = 0.005;
  static constexpr Real cTauRef = 10.0;
  static constexpr Real cArm = 0.5;
  static constexpr Real cTauArm = 0.1;
  static constexpr Real cBandLo = 0.9, cBandHi = 1.1, cHyst = 0.005;

  std::deque<Complex> mHistory;
  Complex mU1{0., 0.}, mU2{0., 0.}, mRef{0., 0.}, mI2{0., 0.}, mI2End{0., 0.};
  Real mElapsed = 0.;
  Real mRampT = 0.;
  Real mKd = 1.;
  bool mValid = false;
  bool mOutside = false;
  bool mReleased = false;

  void setParameters(Real k, Real tau, Real tauMeas) {
    mK = k;
    mTau = tau;
    mTauMeas = tauMeas;
  }
  /// negative-sequence current in pu
  Complex step(Complex envelope, Real omega, Real time, Real dt, Real uBase,
               bool fault, Real ramp) {
    mElapsed += dt;
    const auto delay = static_cast<std::size_t>(
        std::max(1.0, std::round(0.5 * M_PI / omega / dt)));
    mHistory.push_back(envelope);
    while (mHistory.size() > delay + 1)
      mHistory.pop_front();
    if (mHistory.size() == delay + 1) {
      const Real a = 1. - std::exp(-dt / mTauMeas);
      const Complex u1 = (envelope + mHistory.front()) / 2. / uBase;
      const Complex u2 = std::conj((envelope - mHistory.front()) / 2. *
                                   std::polar(1.0, 2. * omega * time)) /
                         uBase;
      mU1 = mValid ? mU1 + a * (u1 - mU1) : u1;
      mU2 += a * (u2 - mU2);
      mValid = true;
    }
    // line-line voltages |u1 + u2 e^-j60 a^2p|
    if (mValid && mElapsed >= cArm) {
      Real lo = 1e9, hi = 0.;
      for (int p = 0; p < 3; ++p) {
        const Real u = std::abs(
            mU1 + mU2 * std::polar(1.0, -M_PI / 3. + 4. * M_PI * p / 3.));
        lo = std::min(lo, u);
        hi = std::max(hi, u);
      }
      mOutside = mOutside ? !(lo >= cBandLo + cHyst && hi <= cBandHi - cHyst)
                          : (lo < cBandLo || hi > cBandHi);
    }
    const bool release = fault || mOutside;
    if (!release)
      mRef += (1. - std::exp(-dt / (mElapsed < cArm ? cTauArm : cTauRef))) *
              (mU2 - mRef);
    Complex target(0., 0.);
    if (release && mK > 0.) {
      target = Complex(0., mK) * (mU2 - mRef);
      mRampT = 0.;
    } else if (mReleased) {
      mI2End = mI2;
    }
    if (!release && mRampT < ramp) {
      mRampT += dt;
      target = mI2End * std::max(0., 1. - mRampT / ramp);
    }
    mReleased = release;
    mI2 += (1. - std::exp(-dt / mTau)) * (target - mI2);
    return mI2;
  }
  bool released() const { return mReleased; }
  /// active-current factor: falls at once, rises with the ramp
  Real activeFactor(Real kd, Real dt, Real ramp) {
    mKd = std::min(kd, mKd + (ramp > 0. ? dt / ramp : 1.));
    return mKd;
  }
  /// Factors (active, reactive) on id and on iq, i2. activeFirst (fault):
  /// |i1| + |i2| <= imax, active current lowered first, then both sequences
  /// alike (4120 10.2.3.3); otherwise all alike on the largest phase current.
  /// Phase p carries i1 + i2 a^2p.
  static std::pair<Real, Real> phaseLimit(Complex id, Complex iq, Complex i2,
                                          Real imax, bool activeFirst) {
    if (activeFirst) {
      const Real a = std::abs(iq), b = std::abs(i2);
      if (a + b >= imax)
        return {0., imax / (a + b)};
      const Real idMax = std::sqrt((imax - b) * (imax - b) - a * a);
      return {std::abs(id) > idMax ? idMax / std::abs(id) : 1., 1.};
    }
    Real peak = 0.;
    for (int p = 0; p < 3; ++p)
      peak = std::max(peak, std::abs(id + iq +
                                     i2 * std::polar(1.0, 4. * M_PI * p / 3.)));
    return peak > imax ? std::pair<Real, Real>{imax / peak, imax / peak}
                       : std::pair<Real, Real>{1., 1.};
  }
};

// half-cycle RMS value: sliding mean of the squared voltage (p.u.^2) over
// half a fundamental period, seeded with mSeed
struct HalfCycleRms {
  std::vector<Real> mBuf;
  std::size_t mPos = 0;
  Real mSum = 0.0;
  Real mSeed = 1.0;

  void initialize(Real seed) {
    mSeed = seed;
    mBuf.clear();
  }
  Real step(Real x2, Real dt, Real omega) {
    if (mBuf.empty()) {
      std::size_t n = std::max<long>(1, std::lround(PI / (omega * dt)));
      mBuf.assign(n, mSeed);
      mSum = mSeed * n;
      mPos = 0;
    }
    mSum += x2 - mBuf[mPos];
    mBuf[mPos] = x2;
    mPos = (mPos + 1) % mBuf.size();
    return std::sqrt(std::max(mSum, 0.0) / mBuf.size());
  }
};

// VDE-AR-N 4110/4120 over-/undervoltage ride-through protection with trip and
// optional reconnection. Not a Dynawo block; after RWTH/FGH DERAvVSIDQ.
// mHVRT false: non-HVRT unit, trips above ~1.11 p.u.; true: HVRT-capable,
// envelope per mLevel (1 = EHV/HV 4120, 2 = MV 4110).
// Input: filtered terminal voltage, p.u. Voltage-time envelope plus a TOV
// limit relative to the pre-fault voltage.
struct VoltageRideThrough {
  bool mHVRT = false; // false = non-HVRT (old), true = HVRT-capable
  int mLevel = 1;     // HVRT voltage level: 1 = EHV/HV (4120), 2 = MV (4110)
  bool mEnabled = false; // false = never trips (default)

  Real mBaseline = 1.0;    // pre-fault operating voltage, p.u.
  Real mExcursionT = 0.0;  // elapsed time in the current excursion, s
  Real mElapsed = 0.0;     // time since start, s
  bool mTripped = false;   // tripped or reconnecting

  // --- optional reconnection; off: the trip is latched ---
  bool mReconnect = false;
  Real mRecLo = 0.90, mRecHi = 1.10; // voltage band that counts as "healthy"
  Real mRecDelay = 1.0;              // healthy hold before reconnecting, s
  Real mRecRamp = 2.0;               // 0->1 injection ramp on reconnection, s
  Real mHealthyT = 0.0;              // healthy time while tripped, s
  Real mGate = 1.0;                  // current scaling 0..1

  // no trip during start-up until the converter transient has settled
  static constexpr Real cArmTime = 0.5;
  // fast baseline time constant during start-up, so the baseline equals the
  // settled operating point when the protection becomes active
  static constexpr Real cTauArm = 0.1;

  // excursion: deviation from the tracked operating point above this; starts
  // the envelope timer and holds the TOV baseline
  static constexpr Real cExcBand = 0.08;
  static constexpr Real cTauTrack = 5.0; // baseline tracking time constant, s
  // relay pickup time; rejects single-step numerical voltage spikes
  static constexpr Real cTripDelay = 0.01;
  // pulse blocking: the current decays to zero over this time
  static constexpr Real cBlockRamp = 0.002;
  Real mViolationT = 0.0; // accumulated continuous violation time, s
  // external reference (VdeFaultRideThrough::u1min()) used as baseline
  bool mUseRef = false;
  Real mRef = 1.0;
  // measured quantity: 0 = filtered positive-sequence magnitude, 1 = phase
  // voltage peak above cPeakLimit for cPeakTime, 2 = half-cycle RMS of the
  // largest line-line voltage
  int mMeasurement = 0;
  static constexpr Real cPeakLimit = 1.25; // p.u. of the nominal phase peak
  static constexpr Real cPeakTime = 2e-4;  // s
  // pickup time of the half-cycle RMS relay
  static constexpr Real cHalfTripDelay = 0.005;
  Real mPeakT = 0.0;

  void setParameters(bool hvrt, int level, bool enabled = true) {
    mHVRT = hvrt;
    mLevel = level;
    mEnabled = enabled;
  }
  // reconnection after a trip, once the voltage has stayed inside
  // [recLo, recHi] for recDelay, then ramp the injection back up over recRamp.
  void setReconnection(bool enable, Real recDelay, Real recRamp, Real recLo = 0.90,
                       Real recHi = 1.10) {
    mReconnect = enable;
    mRecDelay = recDelay;
    mRecRamp = recRamp;
    mRecLo = recLo;
    mRecHi = recHi;
  }
  void setMeasurement(int measurement) { mMeasurement = measurement; }
  void initialize(Real vBaseline) {
    mBaseline = vBaseline;
    mPeakT = 0.0;
    mExcursionT = 0.0;
    mElapsed = 0.0;
    mTripped = false;
    mHealthyT = 0.0;
    mGate = 1.0;
    mViolationT = 0.0;
  }

  // VDE staircase envelope as a function of the excursion elapsed time.
  void envelope(Real t, Real &vMin, Real &vMax, Real &tovLimit) const {
    if (!mHVRT) {
      // old / non-HVRT: no FRT above ~1.1 p.u.
      tovLimit = 0.03;
      if (t < 0.05) { vMax = 1.11; vMin = 0.0; }
      else if (t < 0.15) { vMax = 1.102; vMin = 0.0; }
      else if (t < 3.0) { vMax = 1.101; vMin = (0.85 / 2.85) * t - 0.0447; }
      else { vMax = 1.101; vMin = 0.85; }
    } else if (mLevel == 1) {
      // EHV/HV, VDE-AR-N 4120/4130 Bild 12: 1.30 Un to 0.15 s, 1.25 Un to 60 s
      if (t < 0.15) { vMax = 1.30; vMin = 0.0; tovLimit = 0.25; }
      else if (t < 3.0) { vMax = 1.25; vMin = std::max(0.0, (0.85 / 2.85) * t - 0.0447); tovLimit = 0.15; }
      else if (t < 60.0) { vMax = 1.25; vMin = 0.85; tovLimit = 0.15; }
      else { vMax = 1.15; vMin = 0.85; tovLimit = 0.10; }
    } else {
      // MV, VDE-AR-N 4110
      if (t < 0.1) { vMax = 1.25; vMin = 0.15; tovLimit = 0.25; }
      else if (t < 0.151) { vMax = 1.20; vMin = 0.15; tovLimit = 0.15; }
      else if (t < 3.0) { vMax = 1.20; vMin = 0.11325 + t * 0.245; tovLimit = 0.15; }
      else if (t < 5.0) { vMax = 1.20; vMin = 0.85; tovLimit = 0.15; }
      else if (t < 60.0) { vMax = 1.15; vMin = 0.85; tovLimit = 0.15; }
      else { vMax = 1.10; vMin = 0.90; tovLimit = 0.10; }
    }
  }

  // one control step; true while tripped or reconnecting. The component
  // scales its injected current and P/Q by gate().
  bool step(Real vPu, Real dt, Real vRef) {
    mUseRef = true;
    mRef = vRef;
    return step(vPu, dt);
  }
  bool step(Real vPu, Real dt) {
    if (!mEnabled) { mGate = 1.0; return false; }
    mElapsed += dt;
    // start-up: track the operating point fast, no trip
    if (mElapsed < cArmTime) {
      mBaseline = mUseRef ? mRef : mBaseline + (vPu - mBaseline) * (dt / (dt + cTauArm));
      mGate = 1.0;
      return false;
    }
    // deviation from the tracked operating point => excursion
    bool excursion = std::abs(vPu - mBaseline) > cExcBand;
    if (excursion) {
      mExcursionT += dt; // envelope tightens as the excursion persists
    } else {
      mExcursionT = 0.0;
      // track the steady operating point
      mBaseline = mUseRef ? mRef : mBaseline + (vPu - mBaseline) * (dt / (dt + cTauTrack));
    }
    Real vMin, vMax, tovLimit;
    envelope(mExcursionT, vMin, vMax, tovLimit);
    // violation: TOV overshoot during an excursion, or outside the envelope
    // (VDE limits in p.u. of nominal)
    bool violation = (excursion && vPu > mBaseline + tovLimit) ||
                     (vPu > vMax) || (vPu < vMin);
    // relay pickup time
    mViolationT = violation ? mViolationT + dt : 0.0;
    bool fire = mMeasurement == 2 ? mViolationT >= cHalfTripDelay * (1.0 - 1e-9)
                                  : mViolationT >= cTripDelay;
    return latch(violation, fire, vPu, dt);
  }

  // protection step on the measured quantity mMeasurement. vPeak: largest
  // phase voltage sample (EMT) or largest phase envelope (phasor, envelope =
  // true); the envelope limit is raised by 1 / cos(omega cPeakTime / 2).
  bool stepMeasured(Real uFilt, Real vPeak, Real uHalf, bool envelope, Real omega,
                    Real dt, Real &uProt) {
    if (mMeasurement == 2) {
      uProt = uHalf;
      return step(uHalf, dt);
    }
    if (mMeasurement != 1) {
      uProt = uFilt;
      return step(uFilt, dt);
    }
    uProt = vPeak;
    if (!mEnabled) { mGate = 1.0; return false; }
    mElapsed += dt;
    if (mElapsed < cArmTime) {
      mGate = 1.0;
      return false;
    }
    bool violation;
    bool fire;
    if (envelope) {
      violation = vPeak > cPeakLimit / std::cos(0.5 * omega * cPeakTime);
      fire = violation;
    } else {
      violation = vPeak > cPeakLimit;
      mPeakT = violation ? mPeakT + dt : 0.0;
      fire = mPeakT >= cPeakTime * (1.0 - 1e-9);
    }
    return latch(violation, fire, uFilt, dt);
  }

  // trip latch, pulse blocking and optional reconnection
  bool latch(bool violation, bool fire, Real vPu, Real dt) {
    if (!mTripped) {
      if (fire) { // disconnect
        mTripped = true;
        mGate = std::max(0.0, mGate - dt / cBlockRamp);
        mHealthyT = 0.0;
      } else {
        mGate = 1.0;
      }
    } else if (!mReconnect) {
      mGate = std::max(0.0, mGate - dt / cBlockRamp); // latched trip
    } else {
      // reconnection after a healthy-voltage time, then ramp up; a new
      // violation disconnects again
      bool healthy = (vPu > mRecLo && vPu < mRecHi) && !violation;
      if (healthy) {
        mHealthyT += dt;
      } else {
        mHealthyT = 0.0;
        if (violation) mGate = std::max(0.0, mGate - dt / cBlockRamp);
      }
      if (mHealthyT >= mRecDelay) {
        mGate += dt / std::max(mRecRamp, dt);
        if (mGate >= 1.0) { // fully reconnected
          mGate = 1.0;
          mTripped = false;
          mHealthyT = 0.0;
        }
      }
    }
    return mTripped;
  }
  bool tripped() const { return mTripped; }
  Real gate() const { return mGate; }
};

// Dynawo: NonLinear/DelayFlag.mo
//   fO in {0,1,2}: 1 while fault active (fI), 2 during post-fault window
//   (tD after clearing), 0 otherwise.
struct DelayFlag {
  Real mTd = 0.0;
  Real mNotFaultTimer = 0.0;
  Int mFO = 0;

  void setParameters(Real tD) { mTd = tD; }
  void initialize(Int fo0) {
    mFO = fo0;
    // fo0 == 0: start the timer past tD so the first step gives 0, not the
    // post-fault state 2. Requires setParameters first.
    mNotFaultTimer = (fo0 == 0) ? mTd : 0.0;
  }

  Int step(bool fI, Real dt) {
    if (fI) {
      mNotFaultTimer = 0.0;
      mFO = 1;
    } else {
      mNotFaultTimer += dt;
      mFO = (mNotFaultTimer < mTd) ? 2 : 0;
    }
    return mFO;
  }
  Int output() const { return mFO; }
};

// Dynawo: Continuous/AntiWindupIntegrator.mo
//   rate- and output-limited integrator; fMax/fMin stop further rise/fall
struct AntiWindupIntegrator {
  Real mTi = 1.0;
  Real mDyMin = -9999.0;
  Real mDyMax = 9999.0;
  Real mYMin = -9999.0;
  Real mYMax = 9999.0;
  Real mX = 0.0;

  void setParameters(Real tI, Real dyMin, Real dyMax, Real yMin, Real yMax) {
    mTi = tI;
    mDyMin = dyMin;
    mDyMax = dyMax;
    mYMin = yMin;
    mYMax = yMax;
  }
  void initialize(Real y0) { mX = y0; }

  Real step(Real u, Real dt, bool fMax, bool fMin) {
    Real r = clampReal(u / mTi, mDyMin, mDyMax);
    Real s1 = fMin ? std::max(r, 0.0) : r;
    Real s2 = fMax ? std::min(0.0, s1) : s1;
    mX += dt * s2;
    return clampReal(mX, mYMin, mYMax);
  }
  Real output() const { return clampReal(mX, mYMin, mYMax); }
};

// Dynawo: Continuous/LimPIDFreeze.mo  (fixed output limits)
//   PI with setpoint weight Wp, feedforward Kff and anti-windup tracking
//   (gain 1/(K*Ni), one step delayed)
struct LimPIDFreeze {
  Real mK = 1.0;
  Real mTi = 0.5;
  Real mWp = 1.0;
  Real mNi = 0.9;
  Real mKff = 1.0;
  bool mWithFeedForward = false;
  Real mI = 0.0;       // integrator state
  Real mSatErr = 0.0;  // previous-step saturation error

  void setParameters(Real K, Real Ti, Real Wp, Real Ni, bool withFF, Real Kff) {
    mK = K;
    mTi = Ti;
    mWp = Wp;
    mNi = Ni;
    mWithFeedForward = withFF;
    mKff = Kff;
  }
  void initialize(Real xi0) {
    mI = xi0;
    mSatErr = 0.0;
  }

  Real step(Real u_s, Real u_m, Real dt, bool freeze, Real yMin, Real yMax,
            Real uFF = 0.0) {
    Real awTerm = (mK * mNi != 0.0) ? mSatErr / (mK * mNi) : 0.0;
    Real errI = (u_s - u_m) + awTerm;
    if (!freeze)
      mI += dt * (1.0 / mTi) * errI;
    Real P = mWp * u_s - u_m;
    Real ff = mWithFeedForward ? mKff * uFF : 0.0;
    Real presat = mK * (P + mI) + ff;
    Real y = clampReal(presat, yMin, yMax);
    mSatErr = y - presat;
    return y;
  }
};

// Dynawo: Continuous/VarLimPIDFreeze.mo  (variable output limits)
//   Identical control law to LimPIDFreeze with per-step yMin/yMax inputs.
struct VarLimPIDFreeze {
  LimPIDFreeze mPid;

  void setParameters(Real K, Real Ti, Real Wp, Real Ni, bool withFF, Real Kff) {
    mPid.setParameters(K, Ti, Wp, Ni, withFF, Kff);
  }
  void initialize(Real xi0) { mPid.initialize(xi0); }
  Real step(Real u_s, Real u_m, Real dt, bool freeze, Real yMin, Real yMax,
            Real uFF = 0.0) {
    return mPid.step(u_s, u_m, dt, freeze, yMin, yMax, uFF);
  }
};

// Dynawo: IEC61400 BaseControls/WT/Mechanical.mo, two-mass drive train,
// forward Euler. Aerodynamic and air-gap power in, generator speed out, p.u.
struct Mechanical {
  Real mHgen = 1.0, mHwtr = 1.0, mKdrt = 0.0, mCdrt = 0.0;
  // states
  Real mOmegaGen = 1.0, mOmegaWtr = 1.0, mTwistInt = 0.0;

  void setParameters(Real Hgen, Real Hwtr, Real Kdrt, Real Cdrt) {
    mHgen = (Hgen > 1e-9) ? Hgen : 1e-9;
    mHwtr = (Hwtr > 1e-9) ? Hwtr : 1e-9;
    mKdrt = Kdrt;
    mCdrt = Cdrt;
  }
  /// Initial speeds omega0, initial shaft torque equal to air-gap power PAg0.
  void initialize(Real omega0, Real PAg0) {
    mOmegaGen = omega0;
    mOmegaWtr = omega0;
    mTwistInt = (std::abs(mKdrt) > 1e-9) ? (PAg0 / mKdrt) : 0.0;
  }
  Real step(Real PAero, Real PAg, Real dt) {
    Real Taero = PAero / ((std::abs(mOmegaWtr) > 1e-3) ? mOmegaWtr : 1e-3);
    Real Tag = PAg / ((std::abs(mOmegaGen) > 1e-3) ? mOmegaGen : 1e-3);
    Real dOmega = mOmegaWtr - mOmegaGen;
    mTwistInt += dt * dOmega;
    Real Tshaft = mCdrt * dOmega + mKdrt * mTwistInt;
    mOmegaWtr += dt * (Taero - Tshaft) / (2.0 * mHwtr);
    mOmegaGen += dt * (Tshaft - Tag) / (2.0 * mHgen);
    return mOmegaGen;
  }
  Real outputGen() const { return mOmegaGen; }
  Real outputWtr() const { return mOmegaWtr; }
};

} // namespace WindGen
} // namespace Signal
} // namespace CPS
