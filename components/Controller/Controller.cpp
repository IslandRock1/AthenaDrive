#include "cmath"

#include "Controller.hpp"

Controller::Controller(const ControllerParams& params)
	: _dReg(params.dRegKp, params.dRegKi), _qReg(params.qRegKp, params.qRegKi) {}

Output Controller::update(
	const float iqRef,
	const float electricalPosition,
	const float motorVelocity,
	const float Ia, const float Ib)
{
	const float sinPos = std::sin(electricalPosition);
	const float cosPos = std::cos(electricalPosition);

	_clarkeTransform(Ia, Ib);
	_parkTransform(sinPos, cosPos);
	_PILoop(iqRef, motorVelocity);
	_parkInverse(sinPos, cosPos);
	_clarkeInverse();
	_subtractAvg();

	return _output;
}


void Controller::_clarkeTransform(const float Ia, const float Ib) {
	_clarkeIa = Ia;
	// Bitshift is faster?
	_clarkeIb = (Ia + 2 * Ib) * INV_SQRT_3;
}

void Controller::_parkTransform(const float sinElectricalPos, const float cosElectricalPos) {

	// Coult be faster with matrix mult?
	// Is just a simple roation matrix.
	_parkId = cosElectricalPos * _clarkeIa + sinElectricalPos * _clarkeIb;
	_parkIq = -sinElectricalPos * _clarkeIa + cosElectricalPos * _clarkeIb;
}

void Controller::_PILoop(const float iqRef, const float motorVelocity) {
	// TODO: test sign.
	float vdPI = _dReg.update(_parkId);
	float vqPI = _qReg.update(_parkIq - iqRef);

	float motor_l = 0.001f; // TODO! Find motor L
	float motor_flux = 0.00067f; // TODO! Find motor flux
	float vd_decouple = - motorVelocity * motor_l * _parkIq;
	float vq_decouple = motorVelocity * motor_l * _parkId + motorVelocity * motor_flux;

	_parkVd = vdPI + vd_decouple;
	_parkVq = vqPI + vq_decouple;
}

void Controller::_parkInverse(const float sinElectricalPos, const float cosElectricalPos) {
	_clarkeVa = cosElectricalPos * _parkVd - sinElectricalPos * _parkVq;
	_clarkeVb = sinElectricalPos * _parkVd + cosElectricalPos * _parkVq;
}

void Controller::_clarkeInverse() {
	_output.phaseA = _clarkeVa;

	// Bitshift faster? Constexpr sqrt(3) / 2?
	const float term0 = - _clarkeVa * 0.5f;
	const float term1 = _clarkeVb * SQRT_3_DIV_2;
	_output.phaseB = term0 + term1;
	_output.phaseC = term0 - term1;
}

void Controller::_subtractAvg() {
	float avg = (_output.phaseA + _output.phaseB + _output.phaseC) * INV_3;
	_output.phaseA -= avg;
	_output.phaseB -= avg;
	_output.phaseC -= avg;
}
