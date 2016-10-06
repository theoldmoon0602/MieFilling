#pragma once

#include "BoundingBox.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <vector>

using CircleT = Eigen::Vector3d;

class Circles {
private:
	Eigen::MatrixX3d circles;
	Eigen::MatrixX2d frame;

	Eigen::Affine2d transformMatrix;
public:
	Circles(const Eigen::MatrixX3d& circles, const Eigen::MatrixX2d& frame):circles(circles), frame(frame), transformMatrix(Eigen::Translation2d(0, 0)) {}

	const Eigen::Affine2d& getTransformMatrix() const {
		return transformMatrix;
	}
	void setTransformMatrix(const Eigen::Affine2d& matrix) {
		transformMatrix = matrix;
	}

	const Eigen::MatrixX2d& getFrame() const {
		return frame;
	}
	const Eigen::MatrixX3d& getCircles() const {
		return circles;
	}

	Eigen::Vector2d getCenter() const {
		auto frame = buildFrame();

		double xg = 0;
		double yg = 0;
		double ss = 0;

		for (int i = 0; i < frame.rows(); i++) {
			double s = frame.row(i).x() * frame.row((i + 1) % frame.rows()).y() - frame.row(i).y() * frame.row((i + 1) % frame.rows()).x();
			double cx = (frame.row(i).x() + frame.row((i + 1) % frame.rows()).x()) ;
			double cy = (frame.row(i).y() + frame.row((i + 1) % frame.rows()).y()) ;
			xg += s * cx;
			yg += s * cy;
			ss += s;
		}
		return Eigen::Vector2d(xg / (3* ss), yg /(3* ss));
	}
	const Eigen::Affine2d& transform(const Eigen::Transform<double, 2, Eigen::Affine>& other) {
		return transformMatrix = other * transformMatrix;
	}
	const Eigen::Affine2d& rotate(double theta) {
		return transformMatrix =  Eigen::Translation2d(getCenter()) * Eigen::Rotation2Dd(theta) * Eigen::Translation2d(- getCenter()) * transformMatrix;
	}
	const Eigen::Affine2d& move(const Eigen::Vector2d& d) {
		return transformMatrix = Eigen::Translation2d(d) * transformMatrix;
	}
	Eigen::MatrixX2d buildFrame() const {
		std::vector<double> values;
		for (int i = 0; i < frame.rows(); i++) {
			Eigen::Vector2d row = frame.row(i);
			auto obj = transformMatrix * row;
			values.push_back(obj.x());
			values.push_back(obj.y());
		}
		return Eigen::Map<Eigen::Matrix<double, -1, 2, Eigen::RowMajor>>(&values[0], values.size() / 2, 2);
	}
	Eigen::MatrixX3d buildCircles() const {
		std::vector<double> values;
		for (int i = 0; i < circles.rows(); i++) {
			Eigen::Vector2d row = circles.row(i).block<1, 2>(0, 0).row(0);
			auto obj = transformMatrix * row;
			values.push_back(obj.x());
			values.push_back(obj.y());
			values.push_back(circles.row(i).z());
		}
		return Eigen::Map<Eigen::Matrix<double, -1, 3, Eigen::RowMajor>>(&values[0], values.size() / 3, 3);
	}
	void reset() {
		transformMatrix = Eigen::Translation2d(0, 0);
	}
};

