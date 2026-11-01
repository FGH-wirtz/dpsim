/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <vector>

#include <dpsim-models/SimPowerComp.h>
#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Task.h>

 // PLL modes by terminal voltage magnitude
enum class PllMode { Normal, Sag, DeepSag, Recovery };

///
///              vq  |---------KP_pll------|  w       theta
///  -->(abc2dq0)--->|					   +---->int----->
///          |       |-------int->KI_pll---|          |
///          |________________________________________|

namespace CPS {
namespace Signal {
	class PLL_Enhanced :
		public SimSignalComp,
		public SharedFactory<PLL_Enhanced> {

	protected:
		/// Proportional constant of PI controller
		Real mKp;
		/// Integration constant of PI controller
		Real mKi;

		/// Nominal frequency
		Real mOmegaNom;
		/// Integration time step
        Real mTimeStep;

		/// matrix A of state space model
		Matrix mA = Matrix::Zero(2, 2);
		/// matrix B of state space model
		Matrix mB = Matrix::Zero(2, 2);
		/// matrix C of state space model
		Matrix mC = Matrix::Zero(2, 2);
		/// matrix D of state space model
		Matrix mD = Matrix::Zero(2, 2);



		// base tuning
		Real mTauLeak_s = 3.0;   // integrator leak time constant, s
		Real mEpsLeak = 1e-3;
		Real mLPF_fc_Hz = 15.0;

		// frequency limits
		Real mClampHz = 5.0;                  // +/- Hz around nominal
		Real mMaxSlewHzS = 10.0;                 // Hz/s
		Real mOmegaPrev = mOmegaNom;
		Real mVqFilt = 0.0;                  // filtered vq

		// voltage sag detection, p.u. of mVnom
		Real mVnom = 1.0;                  // p.u. base of |v|
		Real mSagTh = 0.5;                  // <0.5 pu => Sag
		Real mDeepSagTh = 0.2;                  // <0.2 pu => DeepSag (omega held)
		Real mSagRelTh = 0.6;                  // return threshold (hysteresis)
		Real mSagLPF_Hz = 50.0;                 // |v| filter cutoff, Hz
		Real mVmagFilt = 1.0;                  // filtered |v|, p.u.
		PllMode mMode = PllMode::Normal;

		// PI gains scheduled by mode
		Real mKpEff = 0.0;
		Real mKiEff = 0.0;

	public:

		/// vq and vd inputs; the reference is set by the owning component.
		const Attribute<Real>::Ptr mInputRef;
		const Attribute<Real>::Ptr mInputRefD;

		// input u = [omega_nom, vq], state and output [theta, delta omega]

		/// Previous Input
        const Attribute<Matrix>::Ptr mInputPrev;
        /// Current Input
        const Attribute<Matrix>::Ptr mInputCurr;
        /// Previous State
        const Attribute<Matrix>::Ptr mStatePrev;
        /// Current State
        const Attribute<Matrix>::Ptr mStateCurr;
        /// Previous Output
        const Attribute<Matrix>::Ptr mOutputPrev;
        /// Current Output
        const Attribute<Matrix>::Ptr mOutputCurr;

		PLL_Enhanced(String name, Logger::Level logLevel = Logger::Level::off);
		/// Setter for PLL parameters
		void setParameters(Real kpPLL, Real kiPLL, Real omegaNom);
		/// Setter for simulation parameters
		void setSimulationParameters(Real timestep);
		/// Setter for initial values
        void setInitialValues(Real input_init, Matrix state_init, Matrix output_init);
		/// Composition of A, B, C, D matrices based on PLL parameters
		void composeStateSpaceMatrices();
		/// pre step operations
		void signalPreStep(Real time, Int timeStepCount);
		/// step operations
		void signalStep(Real time, Int timeStepCount);
		/// pre step dependencies
		void signalAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
		/// add step dependencies
		void signalAddStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);

		void setSagProtection(Real vnom_pu, Real sagTh = 0.5, Real deepSagTh = 0.2, Real sagRelTh = 0.6) {
			mVnom = vnom_pu; mSagTh = sagTh; mDeepSagTh = deepSagTh; mSagRelTh = sagRelTh;
		}
		void setProtection(Real clampHz, Real maxSlewHzPerSec) {
			mClampHz = clampHz; mMaxSlewHzS = maxSlewHzPerSec;
		}
		void setVqLPF(Real cutoffHz) { mLPF_fc_Hz = cutoffHz; }
		void setLeak(Real eps) { mEpsLeak = eps; }


		Task::List getTasks();

        class PreStep : public Task {
        public:
			PreStep(PLL_Enhanced& pll) :
                Task(**pll.mName + ".PreStep"), mPLL(pll) {
					mPLL.signalAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mPLL.signalPreStep(time, timeStepCount); };
		private:
			PLL_Enhanced& mPLL;
        };

		class Step : public Task {
		public:
			Step(PLL_Enhanced& pll) :
				Task(**pll.mName + ".Step"), mPLL(pll) {
					mPLL.signalAddStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mPLL.signalStep(time, timeStepCount); };
		private:
			PLL_Enhanced& mPLL;
		};
	};
}
}
