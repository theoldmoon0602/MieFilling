#include "Parser.h"



ProblemInput parseInput(std::istream &in)
{
	std::vector<Eigen::MatrixX2d> pieces;
	int n;
	in >> n;
	for (int i = 0; i < n; i++) {
		int pn;
		in >> pn;		// piece id
		in >> pn;
		std::vector<double> piece_values;
		for (int j = 0; j < pn; j++) {
			int x, y;
			in >> x >> y;

			piece_values.push_back(x);
			piece_values.push_back(y);
		}
		pieces.push_back(Eigen::Map<Eigen::Matrix<double, -1, 2, Eigen::RowMajor>, Eigen::RowMajor>(&piece_values[0], piece_values.size() / 2, 2));
	}

	in >> n;
	std::vector<Eigen::MatrixX2d> wakus;
	for (int i = 0; i < n; i++) {
		std::vector<double> waku_values;
		int wn;
		in >> wn;		// waku id
		in >> wn;
		for (int j = 0; j < wn; j++) {
			int x, y;
			in >> x >> y;

			waku_values.push_back(x);
			waku_values.push_back(y);
		}
		wakus.push_back(Eigen::Map<Eigen::Matrix<double, -1, 2, Eigen::RowMajor>, Eigen::RowMajor>(&waku_values[0], waku_values.size() / 2, 2));
	}

	return ProblemInput(pieces, wakus);
}
