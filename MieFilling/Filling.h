#pragma once

#include "Circle.h"
#include <vector>
#include <random>
#include "Eigen/Core"
#include "Eigen/StdVector"
#include <chrono>

class Filling
{
private:
	Eigen::MatrixX2d waku;
	std::vector<Circles> circles;
	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> moves;

	std::vector<Eigen::Affine2d> bestState;
	double bestScore;
	double curScore;

	BoundingBox wakuBox;

	std::mt19937 rd;
	std::uniform_real_distribution<double> rd_f;

	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> vectors;

	Eigen::MatrixX3d builtCircles;

public:
	Filling(const Eigen::MatrixX2d& waku, const std::vector<Circles>& circles);
	const Eigen::MatrixX2d& getWaku() const { return waku; }
	const std::vector<Circles>& getCircles() const { return circles; }
	double getScore() const { return curScore; }
	const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> getVectors() const { return vectors; }
	const std::vector<Circles>& applyBest();
	void SetState(std::vector<Circles>& circles, const std::vector<Eigen::Affine2d>& state) const {
		for (int i = 0; i < circles.size(); i++) {
			circles.at(i).setTransformMatrix(state.at(i));
		}
	};
	void copyState(const std::vector<Circles>& circles, std::vector<Eigen::Affine2d>& state) const {
		state.clear();
		for (int i = 0; i < circles.size(); i++) {
			state.push_back(circles.at(i).getTransformMatrix());
		}
	};
	void yakinamasi();
	//private:
	void initState();
	double evalState();
};