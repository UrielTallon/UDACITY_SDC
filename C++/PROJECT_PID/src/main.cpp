#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#define _USE_MATH_DEFINES
#include <math.h>


// for convenience
using json = nlohmann::json;

// for converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; };
double deg2rad(double x) { return x * pi() / 180; };
double rad2deg(double x) { return x * 180 / pi(); };

void checkArguments(int argc, char* argv[]) {
	std::string usage_instructions = "Usage instructions: ";
	usage_instructions += argv[0];
	usage_instructions += "-m Kp Ki Kd";
	usage_instructions += " -s base_speed [speed_correction_coefficient]";
    

	bool has_valid_args = false;

	if (argc == 1) {
		std::cerr << usage_instructions << std::endl;
	}
	else {
		std::string arg_flag = argv[1];
		if (arg_flag.compare("-m") == 0) {
			if (argc < 7 || argc > 8) {
				std::cerr << "Invalid number of arguments.\n" << usage_instructions << std::endl;
			}
			else {
				// Throw an exception if conversion not possible
				// Always get a runtime error here...
				try {
					std::stod(argv[2]);
					std::stod(argv[3]);
					std::stod(argv[4]);
				}
				catch (std::invalid_argument &e) {
					std::cerr << "Invalid PID parameters" << std::endl;
				}
				// Integrates base speed and correction coefficient
				std::string speed_flag = argv[5];
				if (speed_flag.compare("-s") != 0) {
					std::cerr << "Invalid flag.\n" << usage_instructions << std::endl;
				}
				else {
					if (argc == 7) {
						try {
							std::stod(argv[6]);
							has_valid_args = true;
						}
						catch (std::invalid_argument &e) {
							std::cerr << "Invalid base speed or speed correction factor." << std::endl;
							has_valid_args = false;
						}
					}
					else if (argc == 8) {
						try {
							std::stod(argv[6]);
							std::stod(argv[7]);
							has_valid_args = true;
						}
						catch (std::invalid_argument &e) {
							std::cerr << "Invalid base speed or speed correction factor." << std::endl;
							has_valid_args = false;
						}
					}
				}
			}
		}
		else {
			std::cerr << "Invalid mode flag.\n" << usage_instructions << std::endl;
		}
	}

	if (!has_valid_args) {
		exit(EXIT_FAILURE);
	}
}

// Checks if the SocketIO event has JSON data.
// If there is data, the JSONobject in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_last_of("]");

	if (found_null != std::string::npos) {
		return "";
	}
	else if (b1 != std::string::npos && b2 != std::string::npos) {
		return s.substr(b1, b2 - b1 + 1);
	}
	return "";
}

int main(int argc, char* argv[])
{
	uWS::Hub h;

	PID pid;

	checkArguments(argc, argv);
	std::string mode_flag = argv[1];
	if (mode_flag.compare("-m") == 0) {
		double kp = std::stod(argv[2]);
		double ki = std::stod(argv[3]);
		double kd = std::stod(argv[4]);
		std::cout << "Manual mode engaged!" << std::endl;
		std::cout << "Selected parameters: Kp=" << kp << " Ki=" << ki << " Kd=" << kd << std::endl;
		pid.Init(kp, ki, kd);
		pid.base_speed_ = std::stod(argv[6]);
		std::cout << "Base speed chosen at " << pid.base_speed_ << std::endl;
		if (argc == 7) {
			std::cout << "Speed correction factor alpha not chosen; assumed to be 0.0" << std::endl;
		}
		else if (argc == 8) {
			pid.speed_alpha_ = std::stod(argv[7]);
			std::cout << "Speed correction factor alpha chosen at " << pid.speed_alpha_ << std::endl;
		}
	}

	//TODO implement twiddle
	/*
	else if (mode_flag.compare("-t") == 0) {
		exit(EXIT_SUCCESS);
	}
	else if (mode_flag.compare("-f") == 0) {
		exit(EXIT_SUCCESS);
	}*/

	//pid.Init(0.2, 0.004, 3);

	h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
		// "42" at the start of the message means there is a websocket message event.
		// 4 => websocket message
		// 2 => websocket event
		if (length && length > 2 && data[0] == '4' && data[1] == '2') {
			auto s = hasData(std::string(data).substr(0, length));
			if (s != "") {
				auto j = json::parse(s);
				std::string event = j[0].get<std::string>();

				if (event == "telemetry") {
					// j[1] is the data JSON object
					double cte = std::stod(j[1]["cte"].get<std::string>());
					double speed = std::stod(j[1]["speed"].get<std::string>());
					double angle = std::stod(j[1]["steering_angle"].get<std::string>());
					double max_steering_angle = 1.0;
					
					double steer_value;
					double throttle;

					// Calculate steering angle value
					// NOTA: should be in between [-1, 1].
					pid.UpdateError(cte);
					steer_value = pid.TotalError();
					if (steer_value > max_steering_angle) {
						steer_value = max_steering_angle;
					}
					if (steer_value < -max_steering_angle) {
						steer_value = -max_steering_angle;
					}
					
					throttle = pid.base_speed_ - (pid.speed_alpha_ * (abs(steer_value)));

					// DEBUG
					std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

					json msgJson;
					msgJson["steering_angle"] = steer_value;
					msgJson["throttle"] = throttle;
					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					std::cout << msg << std::endl;
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else {
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it is
	// removed, the program won't compile...
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

	int port(4567);
    
    if (h.listen(port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}

	h.run();
}

