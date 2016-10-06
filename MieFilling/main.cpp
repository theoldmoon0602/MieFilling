#define EIGEN_NO_DEBUG


#include "Eigen/Core"
#include "Eigen/LU"
#include "Eigen/Geometry"
#include "Eigen/StdVector"
#include <vector>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Affine2d);

#include "BoundingBox.h"
#include "Parser.h"
#include "Circle.h"
#include <string>
#include <sstream>
#include <random>
#include <SFML/Graphics.hpp>
#include <iostream>
#include "Filling.h"
#include "PutPutPut.h"


const double pi = 3.141592653589793238;

extern std::string INPUT;

bool isPieceIncludingPoint(const Eigen::MatrixX2d& frame, const Eigen::Vector2d& p) {
	int cnt = 0;
	for (int i = 0; i < frame.rows(); i++) {
		if ((frame.row(i).y() <= p.y() && frame.row((i + 1) % frame.rows()).y() > p.y()) ||
			(frame.row(i).y() > p.y() && frame.row((i + 1) % frame.rows()).y() <= p.y())) {
			double vt = (p.y() - frame.row(i).y()) / (frame.row((i + 1) % frame.rows()).y() - frame.row(i).y());
			if (p.x() < (frame.row(i).x() + (vt * (frame.row((i + 1) % frame.rows()).x() - frame.row(i).x())))) {
				++cnt;
			}
		}
	}
	return (cnt % 2 == 1);
	
}

double distancePointToLine(const Eigen::Vector2d& p, const Eigen::Vector2d& s, const Eigen::Vector2d& e)
{
	double eps = 1e-6;
	if ((e - s).dot(p - s) < eps) {
		return std::sqrt((p - s).squaredNorm());
	}
	if ((s - e).dot(p - e) < eps) {
		return std::sqrt((p - e).squaredNorm());
	}
	return std::fabs((e - s).x()*(p - s).y() - (e - s).y()*(p - s).x()) / std::sqrt((s - e).squaredNorm());
}

double shapeSpace(const Eigen::MatrixX2d& shape) {
	double s = 0;
	for (int i = 0; i < shape.rows(); i++) {
		s += (shape.row(i).x() - shape.row((i + 1) % shape.rows()).x()) * (shape.row(i).y() + shape.row((i + 1) % shape.rows()).y());
	}
	return std::fabs(s) / 2;
}


void circleApproximate(const Eigen::MatrixX2d& ps, Eigen::MatrixX3d& circles, int circleNum = 100)
{
	double eps = 1e-6;

	BoundingBox box = BoundingBox::Create(ps);
	double ds = std::sqrt(shapeSpace(ps) / circleNum);

	std::vector<double> values;
	
	for (double y = box.getHeight(); y > 0; y -= ds) {
		for (double x = 0; x < box.getWidth(); x += ds) {
			Eigen::Vector2d here(x, y);
			
			if (! isPieceIncludingPoint(ps, here)) {
				continue;
			}
			// Point in Shape
			
			double distance = 1e10;
			for (int i = 0; i < ps.rows(); i++) {
				double d = distancePointToLine(here, ps.row(i), ps.row((i + 1) % ps.rows()));
				if (distance > d) {
					distance = d;
				}
			}

			values.push_back(here.x());
			values.push_back(here.y());
			values.push_back(distance);
		}
	}
	circles = Eigen::Map<Eigen::Matrix<double, -1, 3, Eigen::RowMajor>>(&values[0], values.size() / 3, 3);
}

void drawFrame(sf::RenderWindow& window, const Eigen::MatrixX2d& ps, float rate = 1.0) {
	sf::VertexArray varray(sf::LinesStrip, ps.rows() + 1);
	for (int i = 0; i < ps.rows() + 1; i++) {
		varray[i].position = sf::Vector2f(ps.row(i % ps.rows()).x(), ps.row(i % ps.rows()).y());
		varray[i].position = varray[i].position / rate;
		varray[i].color = sf::Color::Magenta;
	}

	window.draw(varray);
}

Eigen::Vector2d ffff(const Eigen::Vector2d& p, const Eigen::Vector2d& s, const Eigen::Vector2d& e) {
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

void drawCircles(sf::RenderWindow& window, const Eigen::MatrixX2d& waku, const std::vector<Circles>& circles, const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& vecs, float rate = 1.0) {
	drawFrame(window, waku, rate);
	for (int i = 0; i < circles.size(); i++) {
		auto frame = circles.at(i).buildFrame();
		drawFrame(window, frame, rate);
		sf::VertexArray line(sf::Lines, 2);
		auto center = circles.at(i).getCenter();
		line[0] = sf::Vertex(sf::Vector2f((center.x() + vecs.at(i).x()) / rate, (center.y() + vecs.at(i).y())/rate), sf::Color::Blue);
		line[1] = sf::Vertex(sf::Vector2f(center.x()/rate, center.y()/rate));
		window.draw(line);
		/*const auto & cs = circles.at(i).buildCircles();

		for (int j = 0; j < cs.rows(); j++) {
			sf::CircleShape circle;
			circle.setFillColor(sf::Color::Transparent);
			circle.setOutlineColor(sf::Color::Cyan);
			circle.setOutlineThickness(1);
			circle.setRadius(cs.row(j).z());
			circle.setRadius(circle.getRadius() / rate);
			circle.setPosition(cs.row(j).x() - circle.getRadius(), cs.row(j).y() - circle.getRadius());
			circle.setPosition(circle.getPosition() / rate);

			window.draw(circle);
		}*/
	}
}


int circleNum = 49;


int main() {
	std::stringstream input;
	input << INPUT;
	auto parseResult = parseInput(input);


	std::vector<Circles> approximates;
	for (int i = 0; i < parseResult.ps.size(); i++) {
		Eigen::MatrixX3d circles;
		circleApproximate(parseResult.ps.at(i), circles, circleNum);
		approximates.push_back(Circles(circles, parseResult.ps.at(i)));
	}

	Filling filling(parseResult.waku.at(0), approximates);
	filling.initState();

	sf::RenderWindow window(sf::VideoMode(1024, 768), "MieFilling");

	sf::Font f;
	f.loadFromFile("fonts/GN-Kin-iro_SansSerif.ttf");

	sf::Text t("", f, 20);
	t.setColor(sf::Color::Blue);
	t.setPosition(0, 600);

	float rate = 10.f;
	while (window.isOpen()) {

		filling.yakinamasi();

		sf::Event e;
		while (window.pollEvent(e))
		{
			if (e.type == sf::Event::Closed) {
				window.close();
			}
			if (e.type == sf::Event::MouseWheelMoved) {
				rate += e.mouseWheel.delta ;
			}
		}

		sf::RectangleShape score;
		score.setPosition(sf::Vector2f(0, 600));
		score.setSize(sf::Vector2f(filling.getScore()/1000000, 10));
		score.setFillColor(sf::Color(200, 0, 0, 100));

		window.clear(sf::Color::White);
		drawCircles(window, filling.getWaku(), filling.getCircles(), filling.getVectors(), rate);
		window.draw(score);
		window.display();
	}

	auto circles = filling.applyBest();
	std::cout << circles.size() << std::endl;
	for (int i = 0; i < circles.size(); i++) {
		auto frame = circles.at(i).buildFrame();
		std::cout << i + 1<< std::endl;
		std::cout << frame.rows() << std::endl;
		std::cout << frame << std::endl;
	}

}

std::string INPUT{ R"(
25
1
6
309 0
90 61
159 280
0 375
43 476
382 356
2
4
66 0
0 323
289 377
403 85
3
8
188 0
148 133
31 112
0 223
112 263
96 338
254 382
352 48
4
4
9 0
0 251
206 269
216 11
5
8
112 0
0 331
277 423
300 357
192 317
233 194
344 228
389 100
6
6
12 31
0 198
245 219
257 23
76 0
63 37
7
4
160 0
0 85
83 244
244 160
8
4
480 310
332 0
0 28
24 316
9
4
49 0
0 334
374 378
294 47
10
4
76 0
0 265
319 357
415 67
11
6
267 0
0 7
0 110
102 130
108 360
324 339
12
4
178 0
0 203
194 375
367 177
13
4
12 0
0 448
174 458
209 7
14
7
211 0
120 283
101 299
51 287
0 423
334 550
486 150
15
6
74 0
48 108
120 124
0 467
167 516
304 71
16
4
220 0
0 14
27 494
225 482
17
8
40 0
0 594
204 611
216 559
177 546
223 124
264 123
275 32
18
6
49 0
0 339
307 378
336 159
121 118
149 20
19
6
15 0
0 322
147 335
159 233
198 232
215 11
20
4
38 0
0 318
252 352
289 41
21
4
25 0
0 249
274 279
299 25
22
4
0 0
22 1535
265 1462
197 25
23
4
51 0
0 361
385 357
391 22
24
6
208 0
0 1071
180 1106
184 1141
232 1151
386 42
25
4
214 0
10 12
0 1149
230 1126
1
1
8
0 1730
1737 1746
1753 21
1455 14
1434 171
608 164
569 6
18 0
)" };