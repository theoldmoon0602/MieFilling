#include "PutPutPut.h"
#include <cmath>
#include <iostream>

using namespace std::chrono;

PutPutPut::PutPutPut(const std::vector<Circles>& approximatedCircles, const Eigen::MatrixX2d & waku):
	putCircles(),
	circles(approximatedCircles),
	waku(waku),
	wakuBox(BoundingBox::Create(waku)),
	rd(std::random_device()())
{
}

//
// Put pieces to waku in specified time
//
void PutPutPut::solve(std::chrono::milliseconds t)
{
	auto curT = system_clock::now();
	auto endT = curT + t;

	const double pi = 3.141592653589793238;

	//
	// shuffle pieces
	//
	std::shuffle(circles.begin(), circles.end(), std::mt19937());

	//
	// put loop (put a piece per loop)
	//
	for (int i = 0; i < circles.size(); i++) {
		{
			//
			// determine random position for new piece
			//
			Eigen::Vector2d d(rd() % static_cast<long>(wakuBox.getRight()) - wakuBox.getLeft(), rd() % static_cast<long>(wakuBox.getBottom()) - wakuBox.getTop());
			double rotation = (rd() % 10) / 1000.f * 2 * pi;

			//
			// put new piece
			//
			putCircles.push_back(circles.at(i));
			putCircles.back().rotate(rotation);
			putCircles.back().move(d);
		}

		//
		// until evaluated value become zero
		//
		while (std::abs(evalState()) > 0.1) {
			if ((curT = system_clock::now()) > endT) {
				break;
			}

			for (int j = 0; j < putCircles.size(); j++) {
				Eigen::Vector2d vec = Eigen::Vector2d(rd(), rd()).normalized() * (rd() % 10);
				double rotation = (rd() % 10) / 1000.f * 2 * pi;

				putCircles.at(j).rotate(rotation);
				putCircles.at(j).move(vec);
			}
		}

		if ((curT = system_clock::now()) > endT) {
			break;
		}
	}
}

namespace {
	//
	// Caluclate collision depth and pick longest one
	//
	Eigen::Vector2d getCollisionVectors(const Eigen::MatrixX3d& a, const Eigen::MatrixX3d& b) {
		double longestDepth = 0;			// longest length of collision depth
		Eigen::Vector2d longestVector(0, 0);		// vector of point to point which has longest length

		//
		// Loop about approximation circles
		//
		for (int i = 0; i < a.rows(); i++) {
			for (int j = 0; j < b.rows(); j++) {
				//
				// Prepare values
				//
				const auto& bToA = b.row(j).block<1, 2>(0, 0) - a.row(i).block<1, 2>(0, 0);		// A to B vector
				double sumOfRadius = a.row(i).z() + b.row(j).z();
				double d = sumOfRadius - bToA.norm();

				//
				// Judge collision and Update
				//
				if (d > 0) { // true -> Collide
					if (longestDepth < d) {
						longestDepth = d;			// update
						longestVector = bToA;
					}
				}
			}
		}

		return longestVector;
	}

	//
	// Calculate vector a pointer to line
	Eigen::Vector2d getPointToLineVector(const Eigen::Vector2d& p, const Eigen::Vector2d& lineStart, const Eigen::Vector2d& lineEnd) {
		auto a = lineEnd - lineStart;
		auto b = p - lineStart;
		
		double r = a.dot(b) / a.squaredNorm();
		if (r <= 0) {
			return lineStart;
		}
		else if (r >= 1) {
			return lineEnd;
		}
		return Eigen::Vector2d(lineStart.x() + r * a.x(), lineStart.y() + r * a.y());
	}

	//
	// calculate vector piece to waku, over waku
	//
	Eigen::Vector2d getOutOfWakuVector(const Circles& a, const Eigen::MatrixX2d& waku, const BoundingBox& wakuBox) {

		std::cout << a.buildFrame() << std::endl;

		auto frameBox = BoundingBox::Create(a.buildFrame());

		//
		// Completely in the waku
		//
		if (wakuBox.isInclude(frameBox)) {
			return Eigen::Vector2d::Zero();
		}

		double longestDistance = 0;
		Eigen::Vector2d longestVector(0, 0);

		//
		// get distance of each lines with each circles
		// 
		auto circles = a.buildCircles();
		for (int i = 0; i < circles.rows(); i++) {
			
			Eigen::Vector2d p(circles.row(i).block<1, 2>(0, 0));
			double r = circles.row(i).z();
			bool isIn = wakuBox.isInclude(p);
			
			for (int j = 0; j < waku.rows(); j++) {
				auto vec = getPointToLineVector(p, waku.row(j), waku.row((j + 1) % waku.rows()));		// a vector p to waku line
				double d = 0;
				
				//
				// Caluclate distance of circle with waku line
				//
				if (isIn && vec.squaredNorm() >= r*r) {		// circle in the waku
					continue;
				}
				if (isIn && vec.squaredNorm() < r*r) {			// a part of cirle is out of the waku
					d = r - vec.norm();
				}
				if (!isIn && vec.squaredNorm() <= r*r) {		// over half of circle is out of the waku
					d = r + (r - vec.norm());
				}
				if (!isIn && vec.squaredNorm() > r*r) {		// circle is out of waku completely
					d = 2 * r + vec.norm();
				}

				//
				// update
				//
				if (d > longestDistance) {
					longestDistance = d;
					longestVector = vec;
				}
			}
		}
		return longestVector;
	}
}

double PutPutPut::evalState()
{
	//
	// Penalties
	//
	double collisionPenalty = 0;
	double outOfWakuPenalty = 0;

	/*std::vector<int> cnts;
	cnts.assign(putCircles.size(), 0);

	vecvec.clear();
	for (int i = 0; i < putCircles.size(); i++) {
		vecvec.push_back(Eigen::Vector2d(0, 0));
	}*/

	//
	// Compute depth of out of waku
	//
	for (int a = 0; a < putCircles.size(); a++) {
		auto vec = getOutOfWakuVector(putCircles.at(a), waku, wakuBox);
		/*vecvec.at(a) += vec;
		cnts.at(a)++;*/
		outOfWakuPenalty += vec.squaredNorm();
	}

	//
	// Compute depth of circle collide a with b
	//
	for (int a = 0; a < putCircles.size(); a++) {
		auto aCircles = putCircles.at(a).buildCircles();		// vector of circles of A
		for (int b = 0; b < putCircles.size(); b++) {
			auto bCircles = putCircles.at(b).buildCircles();	// vector of circles of B
			auto vec = getCollisionVectors(aCircles, bCircles);
			/*vecvec.at(a) += vec;
			cnts.at(a)++;*/

			collisionPenalty += vec.squaredNorm();
		}
	}

	/*for (int i = 0; i < vecvec.size(); i++) {
		vecvec.at(i) = vecvec.at(i) / cnts.at(i);
	}*/

	std::cout << collisionPenalty + outOfWakuPenalty << std::endl;

	return collisionPenalty + outOfWakuPenalty;
}