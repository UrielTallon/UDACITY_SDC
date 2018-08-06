#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

class MPC {
public:
	/*
	* Class constructor
	*/
	MPC();

	/*
	* Class destructor
	*/
	virtual ~MPC();
	
	/*
	* Wrapper for the Ipopt solver
	*/
	std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

	/*
	* Getter for the previously computed value of delta
	*/
	double getPreviousDelta() {
		return prev_delta_;
	}

	/*
	* Getter for the previously computed value of a (speed)
	*/
	double getPreviousA() {
		return prev_a_;
	}

	/*
	* Next x & y values predicted by the solver
	*/
	std::vector<double> next_x_vals_;
	std::vector<double> next_y_vals_;

private:
	/*
	* Those private attributes are previously computed values
	* for both delta (steering angle) and a (throttle); they are
	* used for latency implementation
	*/
	double prev_delta_; // previously computed value of delta
	double prev_a_; // previously computed value of a
};

#endif // !MPC_H

