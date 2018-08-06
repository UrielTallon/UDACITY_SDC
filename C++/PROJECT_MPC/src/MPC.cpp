#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

/*
* Time steps and durations:
* set to 10 steps for a duration 0f 50 ms each
*/
size_t N = 10;
double dt = 0.05;

/*
* Number of timesteps to skip before actuation;
* for a 100ms latency and a dt of 50ms, this gives
* 0.1 / 0.05 = 2
*/
const int latency_periods = 2;

/*
* Both the reference cross track and orientation errors are 0.
* The reference velocity is set to 40 mph.
*/
const double ref_cte_ = 0.;
const double ref_epsi_ = 0.;
const double ref_v_ = 40.0;

/*
* Parameters for smoothing the driving and minimizing the use of steering.
*/
const double smooth_delta_ = 500.0;
const double smooth_a_ = 1.0;
const double min_delta_ = 200.0;
const double min_a_ = 1.0;

/*
* This value assumes the model presented in the classroom is used. It was
* obtained by measuring the radius formed by running the vehicle in the simulator
* around in a circle with a constant steering angle and velocity on a flat terrain.
*
* Lf was tuned until the radius formed by the simulating model presented in the
* classroom matched the previous radius.
*
* This is the length from front to CoG that has a similar radius.
*/
const double Lf = 2.67;

/*
* Organize the variable and constraint arrays:
*/
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;



class FG_eval {
public:
	Eigen::VectorXd coeffs;
	FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

	typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

	void operator()(ADvector& fg, const ADvector& vars) {
		/*
		* The cost is stored in the first element of `fg`. Any addition
		* to the cost should be added to `fg[0].`
		*/
		fg[0] = 0;

		// Part of the cost based on the reference state:
		for (int i(0); i < N; ++i) {
			fg[0] += CppAD::pow(vars[cte_start + i] - ref_cte_, 2);
			fg[0] += CppAD::pow(vars[epsi_start + i] - ref_epsi_, 2);
			fg[0] += CppAD::pow(vars[v_start + i] - ref_v_, 2);
		}

		// Minimize the use of actuators:
		for (int i(0); i < N - 1; ++i) {
			fg[0] += min_delta_ * CppAD::pow(vars[delta_start + i], 2);
			fg[0] += min_a_ * CppAD::pow(vars[a_start + i], 2);
		}

		// Minimize the value gap between sequential actuations:
		for (int i(0); i < N - 2; ++i) {
			fg[0] += smooth_delta_ * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
			fg[0] += smooth_a_ * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
		}

		/*
		* Setup the model constraints.
		* NOTA: we had 1 to each of the starting indices due to the cost
		* being located @ index 0 of `fg`. This bumps the position of all
		* the other values...
		*/

		// Initial constraints
		fg[1 + x_start] = vars[x_start];
		fg[1 + y_start] = vars[y_start];
		fg[1 + psi_start] = vars[psi_start];
		fg[1 + v_start] = vars[v_start];
		fg[1 + cte_start] = vars[cte_start];
		fg[1 + epsi_start] = vars[epsi_start];

		// The rest of the constraints
		for (int t(0); t < N - 1; ++t) {
			// The state at time t+1
			AD<double> x1 = vars[x_start + t + 1];
			AD<double> y1 = vars[y_start + t + 1];
			AD<double> psi1 = vars[psi_start + t + 1];
			AD<double> v1 = vars[v_start + t + 1];
			AD<double> cte1 = vars[cte_start + t + 1];
			AD<double> epsi1 = vars[epsi_start + t + 1];

			// The state at time t
			AD<double> x0 = vars[x_start + t];
			AD<double> y0 = vars[y_start + t];
			AD<double> psi0 = vars[psi_start + t];
			AD<double> v0 = vars[v_start + t];
			AD<double> cte0 = vars[cte_start + t];
			AD<double> epsi0 = vars[epsi_start + t];

			// only consider the actuation at time t
			AD<double> delta0 = vars[delta_start + t];
			AD<double> a0 = vars[a_start + t];

			// use 3rd order polynomial
			AD<double> f0 = coeffs[0] + coeffs[1] * x0 +
				coeffs[2] * CppAD::pow(x0, 2) + 
				coeffs[3] * CppAD::pow(x0, 3);
			AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0);

			/*
			* Recall the equations for the model:
			* x_{t+1} = x_t + v_t * cos(psi_t) * dt
			* y_{t+1} = y_t + v_t * sin(psi_t) * dt
			* psi_{t+1} = psi_t + v_t / Lf * delta_t * dt
			* v_{t+1} = v_t * a_t * dt
			* cte_{t+1} = f(x_t) - y_t + v_t * sin(epsi_t) * dt
			* epsi_{t+1} = psi_t - psides_t + v_t * delta_t / Lf * dt
			*/
			fg[2 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
			fg[2 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
			fg[2 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
			fg[2 + v_start + t] = v1 - (v0 + a0 * dt);
			fg[2 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
			fg[2 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
		}
	}
};

/*
* MPC Class definition & implementation
*/
MPC::MPC() {
	prev_delta_ = 0.0;
	prev_a_ = 0.0;
	next_x_vals_.resize(N - 1);
	next_y_vals_.resize(N - 1);
}
MPC::~MPC() {}

std::vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
	bool ok = true;
	//size_t i;
	typedef CPPAD_TESTVECTOR(double) Dvector;

	double x = state[0];
	double y = state[1];
	double psi = state[2];
	double v = state[3];
	double cte = state[4];
	double epsi = state[5];

	/*
	* Set the number of model variables:
	* if the state is 4 elements with 2 actuators and 10 timesteps, then it is:
	* 4 * 10 + 2 * 9 (10 - 1 number of actuations)
	*/
	size_t n_vars = N * 6 + 2 * (N - 1);

	/*
	* Set the number of constraints
	*/
	size_t n_constraints = N * 6;

	// Initial values of independent variables (should be 0)
	Dvector vars(n_vars);
	for (int i(0); i < n_vars; ++i) {
		vars[i] = 0;
	}

	// Set the initial variable values
	vars[x_start] = x;
	vars[y_start] = y;
	vars[psi_start] = psi;
	vars[v_start] = v;
	vars[cte_start] = cte;
	vars[epsi_start] = epsi;

	/*
	* Set upper and lower limits for x;
	*/
	Dvector vars_lowerbound(n_vars);
	Dvector vars_upperbound(n_vars);

    /*
    * Sett all non-actuators upper and lower limits
    * to the maximum negative and positive value
    */
	for (int i(0); i < delta_start; ++i) {
		vars_lowerbound[i] = -1.0e19;
		vars_upperbound[i] = 1.0e19;
	}

    /*
    * Upper and lower limits for the steering angle are
    * respectively set to 25 and -25 degree
    */
	for (int i(delta_start); i < a_start; ++i) {
		vars_lowerbound[i] = -0.436332;
		vars_upperbound[i] = 0.436332;
	}

	/*
	* This approach was suggested by my mentor and I
	* find it way more intuitive than everything I have
	* seen on the forums: basically, we force the actuators
	* to keep the same previously computed value for a
	* number of latency periods determined by both the
	* latency duration and the timesteps duration.
	*/
	for (int i(delta_start); i < delta_start + latency_periods; ++i) {
		vars_lowerbound[i] = prev_delta_;
		vars_upperbound[i] = prev_delta_;
	}
    
    /*
    * Upper and lower speed limits are respectively set
    * to -100 mph and 100 mph (NOTA: a negative speed
    * is actually a brake)
    */
    for (int i(a_start); i < n_vars; ++i) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

	for (int i(a_start); i < a_start + latency_periods; ++i) {
		vars_lowerbound[i] = prev_a_;
		vars_upperbound[i] = prev_a_;
	}

	/*
	* Lower and upper limits for the constraints (should be 0 besides initial state)
	*/
	Dvector constraints_lowerbound(n_constraints);
	Dvector constraints_upperbound(n_constraints);
	for (int i(0); i < n_constraints; ++i) {
		constraints_lowerbound[i] = 0;
		constraints_upperbound[i] = 0;
	}
    
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;
    
    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

	/*
	* Objects that computes objective and constraints
	*/
	FG_eval fg_eval(coeffs);

	std::string options;
	// this may not work on windows platform
	//options += "Integer print_level  0\n";
	options += "Sparse  true        forward\n";
	options += "Sparse  true        reverse\n";
	// this may not work on windows platform
	options += "Numeric max_cpu_time          0.05\n";

	/*
	* Place to return solution
	*/
	CppAD::ipopt::solve_result<Dvector> solution;

	/*
	* Solve the problem
	*/
	CppAD::ipopt::solve<Dvector, FG_eval>(
		options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
		constraints_upperbound, fg_eval, solution);

	/*
	* Check some of the solution values
	*/
	ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

	/*
	* Cost
	*/
	auto cost = solution.obj_value;
	std::cout << "Cost: " << cost << std::endl;

	/*
	* Shift the values for the number of latency periods
	*/
	prev_delta_ = solution.x[delta_start + latency_periods];
	prev_a_ = solution.x[a_start + latency_periods];

	/*
	* Values used to draw the green line (computed solutions)
	* in the simulator
	*/
	for (size_t i(1); i < N; ++i) {
		next_x_vals_[i - 1] = solution.x[x_start + i];
		next_y_vals_[i - 1] = solution.x[y_start + i];
	}

	/*
	* Return the first actuator value:
	* {x , y, psi, v, cte, epsi, delta, a}
	*/
	return { solution.x[x_start + 1], solution.x[y_start + 1],
			 solution.x[psi_start + 1], solution.x[v_start + 1],
			 solution.x[cte_start + 1], solution.x[epsi_start + 1],
			 solution.x[delta_start + 1], solution.x[a_start + 1] };
}