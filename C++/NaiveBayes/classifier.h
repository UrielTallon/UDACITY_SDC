#ifndef CLASSIFIER_H_
#define CLASSIFIER_H_

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>

class GNB {
public:
	std::vector<std::string> possible_labels_ = { "left", "keep", "right" };

	GNB();

	virtual ~GNB();

	void train(std::vector<std::vector<double>> data, std::vector<std::string> labels);

	std::string predict(std::vector<double>);

private:
	float compute_proba_(const double x, const double mu, const double var);

	void process_class_(std::vector<std::vector<double>> data, std::vector<std::string> labels, int label_index);

	std::vector<std::vector<std::vector<double>>> weights_;
};

#endif	// CLASSIFIER_H_