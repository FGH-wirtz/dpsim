/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/PLL_Enhanced.h>

using namespace CPS;
using namespace CPS::Signal;

PLL_Enhanced::PLL_Enhanced(String name, Logger::Level logLevel) :
	SimSignalComp(name, name, logLevel),
    mInputRef(mAttributes->createDynamic<Real>("input_ref")),
	mInputRefD(mAttributes->createDynamic<Real>("input_refD")),
    mInputPrev(mAttributes->create<Matrix>("input_prev", Matrix::Zero(2,1))),
    mStatePrev(mAttributes->create<Matrix>("state_prev", Matrix::Zero(2,1))),
    mOutputPrev(mAttributes->create<Matrix>("output_prev", Matrix::Zero(2,1))),
    mInputCurr(mAttributes->create<Matrix>("input_curr", Matrix::Zero(2,1))),
    mStateCurr(mAttributes->create<Matrix>("state_curr", Matrix::Zero(2,1))),
    mOutputCurr(mAttributes->create<Matrix>("output_curr", Matrix::Zero(2,1))) { }


void PLL_Enhanced::setParameters(Real kpPLL, Real kiPLL, Real omegaNom) {
    mKp = kpPLL;
    mKi = kiPLL;
    mOmegaNom = omegaNom;
	mOmegaPrev = mOmegaNom;
    SPDLOG_LOGGER_INFO(mSLog, "Kp = {}, Ki = {}", mKp, mKi);

    // First entry of input vector is constant omega
    (**mInputCurr)(0,0) = mOmegaNom;
}

void PLL_Enhanced::setSimulationParameters(Real timestep) {
    mTimeStep = timestep;
    SPDLOG_LOGGER_INFO(mSLog, "Integration step = {}", mTimeStep);
}

void PLL_Enhanced::setInitialValues(Real input_init, Matrix state_init, Matrix output_init) {
	(**mInputCurr)(1,0) = input_init;
    **mStateCurr = state_init;
    **mOutputCurr = output_init;

    SPDLOG_LOGGER_INFO(mSLog, "Initial values:");
    SPDLOG_LOGGER_INFO(mSLog, "inputCurrInit = ({}, {}), stateCurrInit = ({}, {}), outputCurrInit = ({}, {})", (**mInputCurr)(0,0), (**mInputCurr)(1,0), (**mInputPrev)(0,0), (**mInputPrev)(1,0), (**mStateCurr)(0,0), (**mStateCurr)(1,0), (**mStatePrev)(0,0), (**mStatePrev)(1,0));
}

void PLL_Enhanced::composeStateSpaceMatrices() {
    // default leak time constant if tau <= 0
    const double tau_leak = (mTauLeak_s > 0.0) ? mTauLeak_s : 3.0;
	const double eps = 1e-2; // leak, 1/s
	mA << 0, mKi,
		0, -eps;

    mB <<   1,  mKp,
            0,  1;
    mC <<   1,  0,
            0,  1;
    mD <<   0,  0,
            0,  0;

    SPDLOG_LOGGER_INFO(mSLog, "State space matrices:");
    SPDLOG_LOGGER_INFO(mSLog, "A = \n{}", mA);
    SPDLOG_LOGGER_INFO(mSLog, "B = \n{}", mB);
    SPDLOG_LOGGER_INFO(mSLog, "C = \n{}", mC);
    SPDLOG_LOGGER_INFO(mSLog, "D = \n{}", mD);
}

void PLL_Enhanced::signalAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
    prevStepDependencies.push_back(mInputCurr);
	prevStepDependencies.push_back(mOutputCurr);
	modifiedAttributes.push_back(mInputPrev);
    modifiedAttributes.push_back(mOutputPrev);
};

void PLL_Enhanced::signalPreStep(Real time, Int timeStepCount) {
    **mInputPrev = **mInputCurr;
    **mStatePrev = **mStateCurr;
    **mOutputPrev = **mOutputCurr;
}

void PLL_Enhanced::signalAddStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	attributeDependencies.push_back(mInputRef);
	attributeDependencies.push_back(mInputRefD);
	modifiedAttributes.push_back(mInputCurr);
    modifiedAttributes.push_back(mOutputCurr);
};

void PLL_Enhanced::signalStep(Real time, Int timeStepCount) {
	// ---- 1) inputs ----
	const double vq_raw = **mInputRef;
	const double vd_raw = **mInputRefD;
	const double dt = mTimeStep;



	// small helper: wrap angle to (-pi, pi]
	auto wrapPi = [](double x) {
		x = std::fmod(x + M_PI, 2.0*M_PI);
		if (x < 0) x += 2.0*M_PI;
		return x - M_PI;
	};

	// filtered |v| in p.u. for mode detection
	const double vmag = std::sqrt(vd_raw*vd_raw + vq_raw * vq_raw);
	const double alpha_v = std::exp(-2.0 * M_PI * mSagLPF_Hz * dt);
	mVmagFilt = alpha_v * mVmagFilt + (1.0 - alpha_v) * (mVnom > 0 ? vmag / mVnom : vmag);

	// 2) mode with hysteresis
	PllMode prevMode = mMode;
	if (mVmagFilt < mDeepSagTh)        mMode = PllMode::DeepSag;
	else if (mVmagFilt < mSagTh)       mMode = PllMode::Sag;
	else if (mVmagFilt > mSagRelTh)    mMode = PllMode::Normal;

	// 3) vq low-pass, lower cutoff during sag
	double fc = (mMode == PllMode::Normal) ? mLPF_fc_Hz
		: (mMode == PllMode::Sag ? std::max(8.0, 0.5*mLPF_fc_Hz)
			: std::max(5.0, 0.33*mLPF_fc_Hz));
	const double alpha = std::exp(-2.0 * M_PI * fc * dt);
	const double vq_pd = vq_raw / std::max(vmag, 1.0);
	mVqFilt = alpha * mVqFilt + (1.0 - alpha) * vq_pd;

	// 4) lower PI gains during sag
	const double kp_scale = (mMode == PllMode::Normal) ? 1.0
		: (mMode == PllMode::Sag ? 0.35
			: 0.10);   // DeepSag: very slow
	const double ki_scale = (mMode == PllMode::Normal) ? 1.0
		: (mMode == PllMode::Sag ? 0.30
			: 0.05);
	mKpEff = mKp * kp_scale;
	mKiEff = mKi * ki_scale;

	// A and B entries carrying Ki and Kp
	mA(0, 1) = mKiEff;
	mB(0, 1) = mKpEff;

	// input u = [omega_nom; vq_filt]
	(**mInputCurr)(0, 0) = mOmegaNom;
	(**mInputCurr)(1, 0) = mVqFilt;

	// ---- 5) DeepSag: hold omega, advance theta only ----
	if (mMode == PllMode::DeepSag) {
		(**mStateCurr) = **mStatePrev; // states held
		(**mOutputCurr) = mC * **mStateCurr;
		double theta_prev = (**mOutputPrev)(0, 0);
		double theta_now = theta_prev + mOmegaPrev * dt;
		(**mOutputCurr)(0, 0) = theta_now;

		// delta omega from the held omega
		double delta_omega_limited = mOmegaPrev - mOmegaNom;
		(**mOutputCurr)(1, 0) = delta_omega_limited;
		(**mStateCurr)(1, 0) = delta_omega_limited;
		return;
	}

	// ---- 6) state update, backward Euler ----
	Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
	Eigen::Matrix2d M = I - dt * mA;
	Eigen::Vector2d rhs = **mStatePrev + dt * (mB * **mInputCurr);
	**mStateCurr = M.partialPivLu().solve(rhs);

	// Outputs
	**mOutputCurr = mC * **mStateCurr;
	double theta_hat = (**mOutputCurr)(0, 0);
	double delta_omega = (**mOutputCurr)(1, 0);
	double omega_est = mOmegaNom + delta_omega;

	// ---- 7) frequency and rate limit, tighter during sag ----
	double clampHz = (mMode == PllMode::Normal) ? mClampHz
		: (mMode == PllMode::Sag ? std::min(mClampHz, 3.0)
			: std::min(mClampHz, 1.5));
	const double wMin = mOmegaNom - 2.0 * M_PI * clampHz;
	const double wMax = mOmegaNom + 2.0 * M_PI * clampHz;
	double omega_clamped = std::clamp(omega_est, wMin, wMax);

	double maxSlew = 2.0 * M_PI * ((mMode == PllMode::Normal) ?
		mMaxSlewHzS : (mMode == PllMode::Sag ?
			std::min(mMaxSlewHzS, 6.0) : std::min(mMaxSlewHzS, 3.0)));

	const double dOmega = omega_clamped - mOmegaPrev;
	const double dOmegaMax = maxSlew * dt;
	if (dOmega > dOmegaMax) omega_clamped = mOmegaPrev + dOmegaMax;
	if (dOmega < -dOmegaMax) omega_clamped = mOmegaPrev - dOmegaMax;
	mOmegaPrev = omega_clamped;

	// ---- 8) anti-windup: limited omega back into the state ----
	double delta_omega_limited = omega_clamped - mOmegaNom;
	(**mStateCurr)(1, 0) = delta_omega_limited;
	(**mOutputCurr)(1, 0) = delta_omega_limited;
}

Task::List PLL_Enhanced::getTasks() {
	return Task::List({std::make_shared<PreStep>(*this), std::make_shared<Step>(*this)});
}
