#ifndef PID_H_
#define PID_H_

class PID {
public:

	/* 
	* Errors
	*/
	double p_error_;
	double i_error_;
	double d_error_;

	/* 
	* Coefficients
	*/
	double Kp_;
	double Ki_;
	double Kd_;

	/*
	* Base speed
	*/
	double base_speed_;

	/*
	* Speed correction coefficient
	*/
	double speed_alpha_;

	/* 
	* Constructor
	*/
	PID();

	/*
	* Destructor
	*/
	virtual ~PID();

	/*
	* Initialize PID
	*/
	void Init(double Kp, double Ki, double Kd);

	/*
	* Update the PID error variables given cross track error
	*/
	void UpdateError(double cte);

	/*
	* Calculate total PID error
	*/
	double TotalError();

private:

	/*
	* Previous CTE for differential (D) coefficient calculation
	*/
	double prev_cte_;

	/*
	* Has a previous CTE already been recorded?
	*/
	bool cte_recorded_;
};

#endif // !PID_H_
