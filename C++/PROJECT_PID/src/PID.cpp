#include "PID.h"

PID::PID(): p_error_(0.0), i_error_(0.0), d_error_(0.0), Kp_(0.0), Ki_(0.0), Kd_(0.0) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;

	p_error_ = 0.0;
	i_error_ = 0.0;
	d_error_ = 0.0;

	speed_alpha_ = 0.0;
	base_speed_ = 0.3;

	prev_cte_ = 0.0;
	cte_recorded_ = false;
}

void PID::UpdateError(double cte) {

	if (!cte_recorded_) {
		prev_cte_ = cte;
		cte_recorded_ = true;
	}

	p_error_ = cte;
	i_error_ += cte;
	d_error_ = cte - prev_cte_;

	prev_cte_ = cte;
}

double PID::TotalError() {
	return -Kp_ * p_error_ - Ki_ * i_error_ - Kd_ * d_error_;
}