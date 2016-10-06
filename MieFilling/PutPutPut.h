#pragma once

#include "Eigen/Core"
#include "BoundingBox.h"
#include "Circle.h"
#include <chrono>
#include <vector>
#include <random>

class PutPutPut
{
protected:
	std::vector<Circles> putCircles;
	std::vector<Circles> circles;

	Eigen::MatrixX2d waku;
	BoundingBox wakuBox;
	
	std::mt19937 rd;

	std::vector<Eigen::Vector2d> vecvec;

public:
	PutPutPut(const std::vector<Circles>& approximatedCircles, const Eigen::MatrixX2d& waku);
	PutPutPut() = delete;

	void solve(std::chrono::milliseconds t);

	const Eigen::MatrixX2d& getWaku() const { return waku; }
	const std::vector<Circles>& getCircles() const { return putCircles; }

protected:
	double evalState();
};

