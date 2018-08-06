// CarNd_MPC.cpp : Defines the entry point for the console application.
//

#define _USE_MATH_DEFINES
#define _CRT_SECURE_NO_WARNINGS
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// for converting back and forth between radians and degrees
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


std::string hasData(std::string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.rfind("}]");
	if (found_null != std::string::npos) {
		return "";
	}
	else if (b1 != std::string::npos && b2 != std::string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

/*
* Evaluate a polynomial
*/
double polyeval(Eigen::VectorXd coeffs, double x) {
	double result = 0.0;
	for (int i(0); i < coeffs.size(); ++i) {
		result += coeffs[i] * pow(x, i);
	}
	return result;
}

/*
* Fit a polynomial; adapted from:
* https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
*/
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
	assert(xvals.size() == yvals.size());
	assert(order >= 1 && order <= xvals.size() - 1);
	Eigen::MatrixXd A(xvals.size(), order + 1);

	for (int i(0); i < xvals.size(); ++i) {
		A(i, 0) = 1.0;
	}

	for (int j(0); j < xvals.size(); ++j) {
		for (int i(0); i < order; ++i) {
			A(j, i + 1) = A(j, i) * xvals(j);
		}
	}

	auto Q = A.householderQr();
	auto result = Q.solve(yvals);
	return result;
}


int main()
{
	uWS::Hub h;

	/*
	* MPC is initialized here
	*/
	MPC mpc;

	// uWebSocket 0.14.2; onMessage() method definition changed
	h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
		/*
		* "42" at the start of the message means there is a websocket message event.
		* "4" means there is a websocket message
		* "2" means there is a websocket event
		*/
		std::string sdata = std::string(data).substr(0, length);
		std::cout << sdata << std::endl;

		if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
			auto s = hasData(sdata);
			if (s != "") {
				auto j = json::parse(s);
				std::string event = j[0].get<std::string>();

				if (event == "telemetry") {
					// j[1] is the JSON object
					std::vector<double> ptsx = j[1]["ptsx"];
					std::vector<double> ptsy = j[1]["ptsy"];
					double px = j[1]["x"];
					double py = j[1]["y"];
					double psi = j[1]["psi"];
					double v = j[1]["speed"];

					/*
					* Calculate steering angle and throttle using MPC.
					* (both are in between [-1, 1])
					*/
					//double steer_value;
					//double throttle_value;

					/*
					* Conversion of waypoints to car coordinates; first center on the
					* car with ptsx[i] - px and ptsy[i] - py, and apply rotation matrix;
					* NOTA: took me a while to figure this one; moreover, the sign of psi
					* must be flipped (opposite in the simulator) so we are actually computing
					* using -psi. Remember cos function is even and sin function is odd, thus:
					* cos(-psi) = psi and sin(-psi) = -sin(psi)
					*/
					std::vector<double> mpc_x_vals(ptsx.size());
					std::vector<double> mpc_y_vals(ptsy.size());
					for (size_t i(0); i < mpc_x_vals.size(); ++i) {
						mpc_x_vals[i] = (ptsx[i] - px) * cos(psi) + (ptsy[i] - py) * sin(psi);
						mpc_y_vals[i] = -(ptsx[i] - px) * sin(psi) + (ptsy[i] - py) * cos(psi);
					}

					/* 
					* Find the coefficients (3rd order polynomial)
					* NOTA: polyfit function requires Eigen::VectorXd
					* as parameters for x and y
					*/
					Eigen::VectorXd x_vals(mpc_x_vals.size());
					Eigen::VectorXd y_vals(mpc_y_vals.size());
					for (size_t k(0); k < mpc_y_vals.size(); ++k) {
						x_vals(k) = mpc_x_vals[k];
						y_vals(k) = mpc_y_vals[k];
					}
					Eigen::VectorXd coeffs = polyfit(x_vals, y_vals, 3);

					/*
					* Compute Cross-Track Error and epsi.
					* Coefficients are evaluated in the car coordinates, which renders
					* computation of thos 2 values slightly more straightforward
					* (e.g. cte is basically the intercept)
					*/
					double cte = polyeval(coeffs, 0);
					double epsi = -atan(coeffs[1]);

					/*
					* Define the state in the vehicle coordinate system;
					* please note as the vehicle becomes the reference,
					* x, y, and psi are always equal to 0.
					*/
					Eigen::VectorXd v_state(6);
					v_state << 0.0, 0.0, 0.0, v, cte, epsi;

					/*
					* Use the solver to compute the next state. The returned
					* vector is { x, y, psi, v, cte, epsi, delta, a }
					*/
					std::vector<double> next_state = mpc.Solve(v_state, coeffs);

					json msgJson;
					
					/*
					* The solver returns the steering angle between -25 degree and
					* 25 degree (in radians); however, the simulator only accepts a
					* non-dimensional arbitrary scale between -1 and 1; thus the
					* necessity to divide by deg2rad(25); and one more thing, the
					* steering in the simulator is the opposite to the angle returned
					* by the solver, so the sign needs to be flipped...
					*/
					msgJson["steering_angle"] = -mpc.getPreviousDelta() / deg2rad(25);
					msgJson["throttle"] = mpc.getPreviousA();

					/*
					* Display the MPC predicted trajectory (green line)
					*/
					msgJson["mpc_x"] = mpc.next_x_vals_;
					msgJson["mpc_y"] = mpc.next_y_vals_;

					/*
					* .. add (x, y) points to list here, points are in reference to the vehicle's
					* coordinate system; the points in the simulator are connected by a yellow line.
					*/
					msgJson["next_x"] = mpc_x_vals;
					msgJson["next_y"] = mpc_y_vals;

					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					std::cout << msg << std::endl;

					/*
					* Latency:
					* The purpose is to mimic real driving conditions where the car does
					* actuate the commands instantly.
					* !THE CAR SHOULD BE ABLE TO DRIVE AROUND THE TRACK WITH 100ms LATENCY!
					*/
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else {
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	/*
	* We don't need this since we don't use HTTPbut if it is removed,
	* the program does not compile :-(
	*/
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		}
		else {
			// I guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	/*
	* Listening to simulator port
	*/
	int port(4567);
    
    if  (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	}
	else {
		std::cerr << "Failed to listen to port " << port << std::endl;
		return -1;
	}

	/*
	* Run the program
	*/
	h.run();
}

