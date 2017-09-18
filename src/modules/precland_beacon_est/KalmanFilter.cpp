/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file KalmanFilter.h
 * Simple Kalman Filter for variable gain low-passing
 *
 * @author Nicolas de Palezieux <ndepal@gmail.com>
 */

#include "KalmanFilter.h"

namespace precland_beacon_est
{
KalmanFilter::KalmanFilter(matrix::Vector<float, 2> initial, matrix::Matrix<float, 2, 2> covInit)
{
	init(initial, covInit);
}

void KalmanFilter::init(matrix::Vector<float, 2> initial, matrix::Matrix<float, 2, 2> covInit)
{
	_x = initial;
	_covariance = covInit;
}

void KalmanFilter::init(double initial0, double initial1, double covInit00, double covInit11)
{
	matrix::Vector<float, 2> initial;
	initial(0) = initial0;
	initial(1) = initial1;
	matrix::Matrix<float, 2, 2> covInit;
	covInit(0, 0) = covInit00;
	covInit(1, 1) = covInit11;

	init(initial, covInit);
}

void KalmanFilter::predictConst(double dt, matrix::Matrix<float, 2, 2> process_noise)
{
	_x(0) += _x(1) * dt;

	matrix::Matrix<float, 2, 2> A; // propagation matrix
	A(0, 0) = 1;
	A(1, 1) = 1;
	A(0, 1) = dt;

	_covariance = A * _covariance * A.transpose() + process_noise;
}

void KalmanFilter::predictConst(double dt, double process_noise00, double process_noise11)
{
	matrix::Matrix<float, 2, 2> process_noise;
	process_noise(0, 0) = process_noise00;
	process_noise(1, 1) = process_noise11;

	predictConst(dt, process_noise);
}

void KalmanFilter::predict(double dt, double acc, matrix::Matrix<float, 2, 2> process_noise)
{
	_x(0) += _x(1) * dt;
	_x(1) += acc * dt;

	matrix::Matrix<float, 2, 2> A; // propagation matrix
	A(0, 0) = 1;
	A(1, 1) = 1;
	A(0, 1) = dt;

	_covariance = A * _covariance * A.transpose() + process_noise;
}

void KalmanFilter::predict(double dt, double acc, double process_noise00, double process_noise11)
{
	matrix::Matrix<float, 2, 2> process_noise;
	process_noise(0, 0) = process_noise00;
	process_noise(1, 1) = process_noise11;

	predict(dt, acc, process_noise);
}

void KalmanFilter::update(double meas, double measUnc)
{

	// Implicitly take H = [1, 0]
	double residual = meas - _x(0);

	// H * P * H^T simply selects P(0,0)
	double innovCovar = _covariance(0, 0) + measUnc;

	// TODO outlier rejection

	matrix::Vector<float, 2> kalmanGain;
	kalmanGain(0) = _covariance(0, 0);
	kalmanGain(1) = _covariance(1, 0);
	kalmanGain /= innovCovar;

	_x += kalmanGain * residual;

	matrix::Matrix<float, 2, 2> identity;
	identity.identity();

	matrix::Matrix<float, 2, 2> KH; // kalmanGain * H
	KH(0, 0) = kalmanGain(0);
	KH(1, 0) = kalmanGain(1);

	_covariance = (identity - KH) * _covariance;

}

void KalmanFilter::getState(matrix::Vector<float, 2> &state)
{
	state = _x;
}

void KalmanFilter::getState(double &state0, double &state1)
{
	state0 = _x(0);
	state1 = _x(1);
}

void KalmanFilter::getCovariance(matrix::Matrix<float, 2, 2> &covariance)
{
	covariance = _covariance;
}

void KalmanFilter::getCovariance(double &cov00, double &cov11)
{
	cov00 = _covariance(0, 0);
	cov11 = _covariance(1, 1);
}

} // namespace precland_beacon_est
