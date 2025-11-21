#pragma once
#include "DiagnoalMatrix.hpp"
// Options 1: solving continuing Problem
template <typename Scalar, int DIMISIONS>
class CholeskyDecomposition
{
public:
	CholeskyDecomposition() = default;
	~CholeskyDecomposition() = default;
	bool Decomposition(const Matrix<Scalar, DIMISIONS, DIMISIONS> &A)
	{
		mat_ = A;
		constexpr int n = DIMISIONS;

		for (int i = 0; i < n; i++) {
			for (int j = i; j < n; j++) {
				Scalar sum = mat_(i, j);

				for (int k = i - 1; k >= 0; k--) {
					sum -= mat_(i, k) * mat_(j, k);
				}

				if (i == j) {
					if (sum <= 0) {
						return false;
					}

					mat_(i, i) = std::sqrt(sum);

				} else {
					mat_(j, i) = sum / mat_(i, i);
				}
			}

			for (int k = i + 1; k < n; k++) {
				mat_(i, k) = mat_(k, i);
			}
		}

		is_diagnol_ = false;
		return true;
	}

	bool Decomposition(const DiagnoalMatrix<Scalar, DIMISIONS> &A)
	{
		mat_ = A;
		constexpr int n = DIMISIONS;

		for (int i = 0; i < n; i++) {
			mat_(i, i) = std::sqrt(mat_(i, i));
		}

		is_diagnol_ = true;
		return true;
	}

	/* Solve L * y = b */
	inline void ForwardElimination(Vector<Scalar, DIMISIONS> &y, const Vector<Scalar, DIMISIONS> &b)
	{
		constexpr int n = DIMISIONS;

		if (!is_diagnol_) {
			y(0) = b(0) / mat_(0, 0);

			for (int i = 1; i < n; i++) {
				y(i) = b(i);

				for (int j = 0; j < i; j++) {
					y(i) -= mat_(i, j) * y(j);
				}

				y(i) = y(i) / mat_(i, i);
			}

		} else {
			for (int i = 0; i < n; ++i) {
				y(i) = b(i) / mat_(i, i);
			}
		}

	}
	/* Solve L^T * x = y */
	inline void BackwardElimination(Vector<Scalar, DIMISIONS> &x, const Vector<Scalar, DIMISIONS> &y)
	{
		constexpr int n = DIMISIONS;

		if (!is_diagnol_) {
			x(n - 1) = y(n - 1) / mat_(n - 1, n - 1);

			for (int i = n - 2; i >= 0; i--) {
				x(i) = y(i);

				for (int j = i + 1; j < n; j++) {
					x(i) -= mat_(i, j) * x(j);
				}

				x(i) = x(i) / mat_(i, i);
			}

		} else {
			for (int i = 0; i < n; ++i) {
				x(i) = y(i) / mat_(i, i);
			}
		}

	}
	// solve L*L^T*x = b
	void Solve(Vector<Scalar, DIMISIONS> &x, const Vector<Scalar, DIMISIONS> &b)
	{
		Vector<Scalar, DIMISIONS> y;

		/* Solve L * y = b */
		ForwardElimination(y, b);
		/* Solve L^T * x = y */
		BackwardElimination(x, y);
	}

	Matrix<Scalar, DIMISIONS, DIMISIONS> GetDecompositionResult()
	{
		return mat_;
	}

	Matrix<Scalar, DIMISIONS, DIMISIONS> GetMatrixL()
	{
		Matrix<Scalar, DIMISIONS, DIMISIONS> L;

		for (int i = 0; i < DIMISIONS; ++i) {
			for (int j = 0; j <= i; ++j) {
				L(i, j) = mat_(i, j);
			}

			for (int j = i + 1; j < DIMISIONS; ++j) {
				L(i, j) = 0;
			}
		}

		return L;
	}
	Matrix<Scalar, DIMISIONS, DIMISIONS> GetMatrixLT()
	{
		Matrix<Scalar, DIMISIONS, DIMISIONS> LT;

		for (int i = 0; i < DIMISIONS; ++i) {
			for (int j = 0; j < i; ++j) {
				LT(i, j) = 0;
			}

			for (int j = i; j < DIMISIONS; ++j) {
				LT(i, j) = mat_(i, j);
			}
		}

		return LT;
	}
	Matrix<Scalar, DIMISIONS, DIMISIONS> InverseL()
	{
		Matrix<Scalar, DIMISIONS, DIMISIONS> inv_L;
		Vector<Scalar, DIMISIONS> y;
		Vector<Scalar, DIMISIONS> b;
		b.setZero();

		// L^-1
		for (int i = 0; i < DIMISIONS; ++i) {
			b(i) = 1;
			this->ForwardElimination(y, b);
			inv_L.col(i) = y;
			b(i) = 0;
		}

		return inv_L;
	}
	Matrix<Scalar, DIMISIONS, DIMISIONS> Inverse()
	{
		Matrix<Scalar, DIMISIONS, DIMISIONS> inv_L = InverseL();

		Matrix<Scalar, DIMISIONS, DIMISIONS> inv;
		Vector<Scalar, DIMISIONS> x;
		Vector<Scalar, DIMISIONS> y;

		// L^T*X=Y
		for (int i = 0; i < DIMISIONS; ++i) {
			y = inv_L.col(i);
			this->BackwardElimination(x, y);
			inv.col(i) = x;
		}

		return inv;
	}

private:
	bool is_diagnol_ = false;
	Matrix<Scalar, DIMISIONS, DIMISIONS> mat_; // L*L^T decompsition result
};
