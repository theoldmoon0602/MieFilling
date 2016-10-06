#include "Filling.h"
#include <iostream>
#include <algorithm>
#include <numeric>


Filling::Filling(const Eigen::MatrixX2d& waku, const std::vector<Circles>& circles):
	waku(waku),
	circles(circles),
	rd(std::random_device()()),
	rd_f(),
	wakuBox(BoundingBox::Create(waku))
{
	moves.clear();
}

const std::vector<Circles> & Filling::applyBest()
{
	SetState(circles, bestState);
	return circles;
}

void Filling::yakinamasi()
{
	if (bestScore <= curScore) {
		//setState(circles, bestState);
	}

	

	for (int i = 0; i < circles.size(); i++) {
		circles.at(i).move(-vectors.at(i) * ((rd() % 100) / 100.f) );
	}
	curScore = evalState();
	// swap and rotate
	{
		int i = rd() % circles.size();
		int j = i;
		while (i == j) {
			j = rd() % circles.size();
		}

		Eigen::Vector2d iMove(circles.at(i).getTransformMatrix().translation());
		Eigen::Vector2d jMove(circles.at(j).getTransformMatrix().translation());

		circles.at(i).move(-iMove);
		circles.at(i).move(jMove);

		circles.at(j).move(-jMove);
		circles.at(j).move(iMove);

		double theta = (rd() % 100) / 1000.f * 2 * 3.1415926535;
		circles.at(i).rotate(theta);
	}
}

void Filling::initState()
{

	for (int i = 0; i < circles.size(); i++) {
		auto frameBox = BoundingBox::Create(circles.at(i).buildFrame());
		long minX = frameBox.getLeft() - wakuBox.getLeft();
		long maxX = wakuBox.getRight() - frameBox.getRight();
		long minY = frameBox.getTop() - wakuBox.getTop();
		long maxY = wakuBox.getBottom() - frameBox.getBottom();
		if (maxX + minX == 0) {
			maxX += 1;
		}
		if (maxY + minY == 0) {
			maxY += 1;
		}
		Eigen::Vector2d d((rd() % (maxX + minX) - minX), ((rd() % (maxY + minY)) - minY));
		circles.at(i).move(d);
	}
	
	copyState(circles, bestState);
	curScore = bestScore = evalState();
}


namespace {
	//
	// Caluclate collision depth and pick longest one
	//
	Eigen::Vector2d getCollisionVector(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
		const auto& bToA = Eigen::Vector2d(b.x() - a.x(), b.y() - a.y());		// A to B vector
		double sumOfRadius = a.z() + b.z();
		double d = sumOfRadius - bToA.norm();

		if (d <= 0) {
			return Eigen::Vector2d(0, 0);
		}

		return bToA;
	}
	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> getCollisionVectors(const Eigen::MatrixX3d& a, const Eigen::MatrixX3d& b) {
		double longestDepth = 0;			// longest length of collision depth
		std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> vecs;
		//
		// Loop about approximation circles
		//
		for (int i = 0; i < a.rows(); i++) {
			for (int j = 0; j < b.rows(); j++) {
				vecs.push_back(getCollisionVector(a.row(i), b.row(j)));
			}
		}

		return vecs;
	}

	//
	// Calculate vector a pointer to line
	Eigen::Vector2d getPointToLineVector(const Eigen::Vector2d& p, const Eigen::Vector2d& s, const Eigen::Vector2d& e) {
		double eps = 1e-5;
		const double pi = 3.141592653589793238;

		if ((e - s).dot(p - s) < eps) {
			return s - p;
		}
		if ((s - e).dot(p - e) < eps) {
			return e - p;
		}
		Eigen::Vector2d sp = p - s;
		Eigen::Vector2d se = e - s;
		auto vec = s + sp.dot(se) / se.squaredNorm() * se;
		return vec - p;
	}

	//
	// calculate vector piece to waku, over waku
	//
	Eigen::Vector2d getOutOfWakuVector(const Circles& a, const Eigen::MatrixX2d& waku, const BoundingBox& wakuBox) {

		auto frameBox = BoundingBox::Create(a.buildFrame());

		//
		// Completely in the waku
		//
		if (wakuBox.isInclude(frameBox)) {
			return Eigen::Vector2d::Zero();
		}

		Eigen::Vector2d result(0, 0);

		//
		// get distance of each lines with each circles
		// 
		auto circles = a.buildCircles();
		for (int i = 0; i < circles.rows(); i++) {
			Eigen::Vector2d p(circles.row(i).block<1, 2>(0, 0));
			double r = circles.row(i).z();
			bool isIn = wakuBox.isInclude(p);

			if (isIn) {
				Eigen::Vector2d longestVec(0, 0);
				bool updated = false;
				for (int j = 0; j < waku.rows(); j++) {
					auto vec = getPointToLineVector(p, waku.row(j), waku.row((j + 1) % waku.rows()));		// a vector p to waku line
					if (longestVec.squaredNorm() < vec.squaredNorm()) {
						updated = true;
						longestVec = vec;
					}
				}

				if (updated) {
					if (r*r > longestVec.squaredNorm()) {
						double d = (r - longestVec.norm()) * (-1);
						if (d * d > result.squaredNorm()) {
							result = longestVec.normalized() * d;
						}
					}
				}
				
			}
			else {
				Eigen::Vector2d shortestVec(1e10, 1e10);
				bool updated = false;
				for (int j = 0; j < waku.rows(); j++) {
					auto vec = getPointToLineVector(p, waku.row(j), waku.row((j + 1) % waku.rows()));		// a vector p to waku line	
					if (vec.squaredNorm() < shortestVec.squaredNorm()) {
						updated = true;
						shortestVec = vec;
					}
				}
				if (updated) {
					double d = 0;
					if (r*r > shortestVec.squaredNorm()) {
						d = r + shortestVec.norm();
					}
					else {
						d = 2 * r + shortestVec.norm();
					}

					if (d*d > result.squaredNorm()) {
						result = shortestVec.normalized() * d;
					}
				}
			}

		}
		

		return result;
	}
}

double Filling::evalState()
{
	//
	// Penalties
	//
	double collisionPenalty = 0;
	double outOfWakuPenalty = 0;

	vectors.resize(0);
	vectors.assign(circles.size(), Eigen::Vector2d::Zero());

	//
	// Compute depth of out of waku
	//
	for (int a = 0; a < circles.size(); a++) {
		Eigen::Vector2d vec = getOutOfWakuVector(circles.at(a), waku, wakuBox);
		if (vectors.at(a).isZero() || vectors.at(a).squaredNorm() < vec.squaredNorm()) {
			vectors.at(a) = -vec;
		}
		outOfWakuPenalty += vec.squaredNorm();
	}


	//
	// Compute depth of circle collide a with b
	//
	for (int a = 0; a < circles.size(); a++) {
		auto aCircles = circles.at(a).buildCircles();		// vector of circles of A
		for (int b = 0; b < circles.size(); b++) {
			auto bCircles = circles.at(b).buildCircles();	// vector of circles of B
			auto vecs = getCollisionVectors(aCircles, bCircles);
			for (int i = 0; i < vecs.size(); i++) {
				if (vectors.at(a).squaredNorm() < vecs.at(i).squaredNorm()) {
					vectors.at(a) = vecs.at(i);
				}
				collisionPenalty += vecs.at(i).squaredNorm();
			}
		}
	}

	return collisionPenalty + outOfWakuPenalty;
}




