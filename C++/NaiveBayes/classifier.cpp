#define _USE_MATH_DEFINES
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include "classifier.h"

GNB::GNB() {
	
}

GNB::~GNB() {

}

float GNB::compute_proba_(const double x, const double mu, const double std) {
	double expo(exp(-pow((x - mu), 2) / (2 * pow(std, 2))));
	double prop(1.0 / sqrt(2 * M_PI * pow(std, 2)));
	return prop * expo;
}

void GNB::process_class_(std::vector<std::vector<double>> data, std::vector<std::string> labels, int label_index) {
	std::vector<std::vector<double>> fragment;
	std::string label(possible_labels_[label_index]);

	for (size_t i(0); i < labels.size(); ++i) {
		if (labels[i].compare(label) == 0) {
			fragment.push_back(data[i]);
		}
	}

	weights_[label_index].resize(fragment[0].size());
	std::vector<double> vals(weights_[label_index].size(), 0);

	for (size_t i(0); i < fragment.size(); ++i) {
		for (size_t j(0); j < fragment[i].size(); ++j) {
			vals[j] += fragment[i][j];
		}
	}

	for (size_t i(0); i < weights_[label_index].size(); ++i) {
		weights_[label_index][i].push_back(vals[i] / fragment.size());
	}
	
	vals = std::vector<double>(weights_[label_index].size(), 0);

	for (size_t i(0); i < fragment.size(); ++i) {
		for (size_t j(0); j < fragment[i].size(); ++j) {
			vals[j] += pow((fragment[i][j] - weights_[label_index][j][0]), 2);
		}
	}
	
	for (size_t i(0); i < weights_[label_index].size(); ++i) {
		weights_[label_index][i].push_back(sqrt(vals[i] / fragment.size()));
	}

}

void GNB::train(std::vector<std::vector<double>> data, std::vector<std::string> labels) {
	
	weights_.clear();
	weights_.resize(possible_labels_.size());
	
	for (size_t i(0); i < possible_labels_.size(); ++i) {
		process_class_(data, labels, i);
	}

	//std::cout << weights_.size() << std::endl;
	//std::cout << weights_[0].size() << std::endl;
	//std::cout << weights_[0][0].size() << std::endl;
}

std::string GNB::predict(std::vector<double> sample) {
	
	double best(0);
	int idx(0);

	for (size_t i(0); i < possible_labels_.size(); ++i) {
		double pr(1);
		for (size_t j(0); j < sample.size(); j++) {
			pr *= compute_proba_(sample[j], weights_[i][j][0], weights_[i][j][1]);
		}
		if (pr > best) {
			best = pr;
			idx = i;
		}
	}

	return possible_labels_[idx];

}