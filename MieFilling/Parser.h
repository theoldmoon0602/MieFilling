#pragma once

#include <vector>
#include <istream>
#include "Eigen/Core"

class ProblemInput {
public:
	std::vector<Eigen::MatrixX2d> ps;
	std::vector<Eigen::MatrixX2d> waku;

	ProblemInput(const std::vector<Eigen::MatrixX2d>& ps, const std::vector<Eigen::MatrixX2d>& waku) : ps(ps), waku(waku) {}
};

ProblemInput parseInput(std::istream & in);
