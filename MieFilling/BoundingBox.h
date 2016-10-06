#pragma once

#include "Eigen/Core"

class BoundingBox
{
public:
	using T = double;

private:

	T top, bottom, left, right;
public:

	T getTop() const {
		return top;
	}
	T getBottom() const {
		return bottom;
	}
	T getLeft() const {
		return left;
	}
	T getRight() const {
		return right;
	}

	T getWidth() const {
		return right - left;
	}
	T getHeight() const {
		return bottom - top;
	}

	BoundingBox() :top(0), bottom(0), left(0), right(0) {}
	BoundingBox(T top, T bottom, T left, T right) : top(top), bottom(bottom), left(left), right(right) {}

	bool isInclude(const BoundingBox& obj) const {
		return (top <= obj.top && bottom >= obj.bottom && left <= obj.left && right >= obj.right);
	}
	bool isInclude(const Eigen::Vector2d& point) const {
		return (top <= point.y() && bottom >= point.y() && left <= point.x() && right >= point.x());
	}
	bool isCollide(const BoundingBox& obj) const {
		return ((top <= obj.top && obj.top <= bottom) || (top <= obj.bottom && obj.bottom <= bottom)) &&
			((left <= obj.left && obj.left <= right) || (left <= obj.right && obj.right <= right));
	}

	static BoundingBox fuse (const BoundingBox& a, const BoundingBox& b) {
		return BoundingBox(
			(a.top < b.top) ? a.top : b.top,
			(a.bottom > b.bottom) ? a.bottom : b.bottom,
			(a.left < b.left) ? a.left : b.left,
			(a.right > a.right) ? a.right : b.right
			);
	}

	static BoundingBox Create(const Eigen::MatrixX2d& ps) {
		
		BoundingBox::T top = ps.col(1).minCoeff();
		BoundingBox::T bottom = ps.col(1).maxCoeff();
		BoundingBox::T left = ps.col(0).minCoeff();
		BoundingBox::T right = ps.col(0).maxCoeff();

		return BoundingBox(top, bottom, left, right);
	}
};



