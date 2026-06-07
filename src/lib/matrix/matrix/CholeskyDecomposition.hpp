#pragma once
#include "Matrix.hpp"
#include "Vector.hpp"
// Options 1: solving continuing Problem
template <typename Scalar, int DIMISIONS, typename Derived>
class CholeskyDecompositionCommon {
 public:
  CholeskyDecompositionCommon() = default;
  ~CholeskyDecompositionCommon() = default;

  // solve L*L^T*x = b
  void Solve(matrix::Vector<Scalar, DIMISIONS>& x,
             const matrix::Vector<Scalar, DIMISIONS>& b) const {
    matrix::Vector<Scalar, DIMISIONS> y;

    /* Solve L * y = b */
    derived().ForwardElimination(y, b);
    /* Solve L^T * x = y */
    derived().BackwardElimination(x, y);
  }
  // solve L*L^T*X = B
  template <int RHS_DIMISIONS>
  void Solve(matrix::Matrix<Scalar, DIMISIONS, RHS_DIMISIONS>& X,
             const matrix::Matrix<Scalar, DIMISIONS, RHS_DIMISIONS>& B) const {
    matrix::Matrix<Scalar, DIMISIONS, RHS_DIMISIONS> Y;

    /* Solve L * Y = B */
    derived().template ForwardElimination<RHS_DIMISIONS>(Y, B);
    /* Solve L^T * X = Y */
    derived().template BackwardElimination<RHS_DIMISIONS>(X, Y);
  }

  matrix::Matrix<Scalar, DIMISIONS, DIMISIONS> GetDecompositionResult() const {
    matrix::Matrix<Scalar, DIMISIONS, DIMISIONS> result;
    for (int i = 0; i < DIMISIONS; ++i) {
      result(i, i) = mat_(i, i);
      for (int j = 0; j < i; ++j) {
        result(i, j) = mat_(i, j);
        result(j, i) = mat_(i, j);
      }
    }
    return result;
  }

  matrix::Matrix<Scalar, DIMISIONS, DIMISIONS> GetMatrixL() const {
    matrix::Matrix<Scalar, DIMISIONS, DIMISIONS> L;

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
  matrix::Matrix<Scalar, DIMISIONS, DIMISIONS> GetMatrixLT() const {
    matrix::Matrix<Scalar, DIMISIONS, DIMISIONS> LT;

    for (int i = 0; i < DIMISIONS; ++i) {
      for (int j = 0; j < i; ++j) {
        LT(i, j) = 0;
      }
      for (int j = i; j < DIMISIONS; ++j) {
        LT(i, j) = mat_(j, i);
      }
    }
    return LT;
  }
  matrix::Matrix<Scalar, DIMISIONS, DIMISIONS> InverseL() const {
    matrix::Matrix<Scalar, DIMISIONS, DIMISIONS> inv_L;
    matrix::Vector<Scalar, DIMISIONS> y;
    matrix::Vector<Scalar, DIMISIONS> b;
    b.setZero();
    // L^-1
    for (int i = 0; i < DIMISIONS; ++i) {
      b(i) = 1;
      derived().ForwardElimination(y, b);
      inv_L.col(i) = y;
      b(i) = 0;
    }
    return inv_L;
  }
  matrix::Matrix<Scalar, DIMISIONS, DIMISIONS> Inverse() const {
    matrix::Matrix<Scalar, DIMISIONS, DIMISIONS> inv_L = InverseL();

    matrix::Matrix<Scalar, DIMISIONS, DIMISIONS> inv;
    matrix::Vector<Scalar, DIMISIONS> x;
    matrix::Vector<Scalar, DIMISIONS> y;
    // L^T*X=Y
    for (int i = 0; i < DIMISIONS; ++i) {
      y = inv_L.col(i);
      derived().BackwardElimination(x, y);
      inv.col(i) = x;
    }

    return inv;
  }

 protected:
  matrix::Matrix<Scalar, DIMISIONS, DIMISIONS>
      mat_;  // L*L^T decompsition result

 private:
  const Derived& derived() const { return static_cast<const Derived&>(*this); }
};

template <typename Scalar, int DIMISIONS>
class CholeskyDecomposition
    : public CholeskyDecompositionCommon<
          Scalar, DIMISIONS, CholeskyDecomposition<Scalar, DIMISIONS>> {
 public:
  CholeskyDecomposition() = default;
  ~CholeskyDecomposition() = default;
  bool Decomposition(const matrix::Matrix<Scalar, DIMISIONS, DIMISIONS>& A) {
    this->mat_ = A;
    constexpr int n = DIMISIONS;

    for (int i = 0; i < n; i++) {
      for (int j = i; j < n; j++) {
        Scalar sum = this->mat_(i, j);
        for (int k = i - 1; k >= 0; k--)
          sum -= this->mat_(i, k) * this->mat_(j, k);
        if (i == j) {
          if (sum <= 0) {
            return false;
          }
          this->mat_(i, i) = std::sqrt(sum);
        } else
          this->mat_(j, i) = sum / this->mat_(i, i);
      }
    }
    return true;
  }
  /* Solve L * y = b */
  inline void ForwardElimination(
      matrix::Vector<Scalar, DIMISIONS>& y,
      const matrix::Vector<Scalar, DIMISIONS>& b) const {
    constexpr int n = DIMISIONS;

    y(0) = b(0) / this->mat_(0, 0);
    for (int i = 1; i < n; i++) {
      y(i) = b(i);
      for (int j = 0; j < i; j++) y(i) -= this->mat_(i, j) * y(j);
      y(i) = y(i) / this->mat_(i, i);
    }
  }
  /* Solve L * Y = B */
  template <int RHS_DIMISIONS>
  inline void ForwardElimination(
      matrix::Matrix<Scalar, DIMISIONS, RHS_DIMISIONS>& Y,
      const matrix::Matrix<Scalar, DIMISIONS, RHS_DIMISIONS>& B) const {
    constexpr int n = DIMISIONS;

    for (int col = 0; col < RHS_DIMISIONS; ++col) {
      Y(0, col) = B(0, col) / this->mat_(0, 0);
    }

    for (int i = 1; i < n; i++) {
      for (int col = 0; col < RHS_DIMISIONS; ++col) {
        Y(i, col) = B(i, col);
      }
      for (int j = 0; j < i; j++) {
        for (int col = 0; col < RHS_DIMISIONS; ++col)
          Y(i, col) -= this->mat_(i, j) * Y(j, col);
      }
      for (int col = 0; col < RHS_DIMISIONS; ++col) {
        Y(i, col) = Y(i, col) / this->mat_(i, i);
      }
    }
  }
  /* Solve L^T * x = y */
  inline void BackwardElimination(
      matrix::Vector<Scalar, DIMISIONS>& x,
      const matrix::Vector<Scalar, DIMISIONS>& y) const {
    constexpr int n = DIMISIONS;

    x(n - 1) = y(n - 1) / this->mat_(n - 1, n - 1);
    for (int i = n - 2; i >= 0; i--) {
      x(i) = y(i);
      for (int j = i + 1; j < n; j++) x(i) -= this->mat_(j, i) * x(j);
      x(i) = x(i) / this->mat_(i, i);
    }
  }
  /* Solve L^T * X = Y */
  template <int RHS_DIMISIONS>
  inline void BackwardElimination(
      matrix::Matrix<Scalar, DIMISIONS, RHS_DIMISIONS>& X,
      const matrix::Matrix<Scalar, DIMISIONS, RHS_DIMISIONS>& Y) const {
    constexpr int n = DIMISIONS;

    for (int col = 0; col < RHS_DIMISIONS; ++col) {
      X(n - 1, col) = Y(n - 1, col) / this->mat_(n - 1, n - 1);
    }

    for (int i = n - 2; i >= 0; i--) {
      for (int col = 0; col < RHS_DIMISIONS; ++col) {
        X(i, col) = Y(i, col);
      }
      for (int j = i + 1; j < n; j++) {
        for (int col = 0; col < RHS_DIMISIONS; ++col)
          X(i, col) -= this->mat_(j, i) * X(j, col);
      }
      for (int col = 0; col < RHS_DIMISIONS; ++col) {
        X(i, col) = X(i, col) / this->mat_(i, i);
      }
    }
  }
};

template <typename Scalar>
class CholeskyDecomposition<Scalar, 3>
    : public CholeskyDecompositionCommon<Scalar, 3,
                                         CholeskyDecomposition<Scalar, 3>> {
 public:
  CholeskyDecomposition() = default;
  ~CholeskyDecomposition() = default;
  bool Decomposition(const matrix::Matrix<Scalar, 3, 3>& A) {
    this->mat_ = A;
    Scalar sum = A(0, 0);
    if (sum <= 0) {
      return false;
    }
    this->mat_(0, 0) = std::sqrt(sum);

    this->mat_(1, 0) = A(0, 1) / this->mat_(0, 0);
    this->mat_(2, 0) = A(0, 2) / this->mat_(0, 0);

    sum = A(1, 1) - this->mat_(1, 0) * this->mat_(1, 0);
    if (sum <= 0) {
      return false;
    }
    this->mat_(1, 1) = std::sqrt(sum);

    this->mat_(2, 1) =
        (A(1, 2) - this->mat_(1, 0) * this->mat_(2, 0)) / this->mat_(1, 1);

    sum = A(2, 2) - this->mat_(2, 0) * this->mat_(2, 0) -
          this->mat_(2, 1) * this->mat_(2, 1);
    if (sum <= 0) {
      return false;
    }
    this->mat_(2, 2) = std::sqrt(sum);

    return true;
  }

  inline void ForwardElimination(matrix::Vector<Scalar, 3>& y,
                                 const matrix::Vector<Scalar, 3>& b) const {
    y(0) = b(0) / this->mat_(0, 0);
    y(1) = (b(1) - this->mat_(1, 0) * y(0)) / this->mat_(1, 1);
    y(2) = (b(2) - this->mat_(2, 0) * y(0) - this->mat_(2, 1) * y(1)) /
           this->mat_(2, 2);
  }

  template <int RHS_DIMISIONS>
  inline void ForwardElimination(
      matrix::Matrix<Scalar, 3, RHS_DIMISIONS>& Y,
      const matrix::Matrix<Scalar, 3, RHS_DIMISIONS>& B) const {
    for (int col = 0; col < RHS_DIMISIONS; ++col) {
      Y(0, col) = B(0, col) / this->mat_(0, 0);
      Y(1, col) = (B(1, col) - this->mat_(1, 0) * Y(0, col)) / this->mat_(1, 1);
      Y(2, col) = (B(2, col) - this->mat_(2, 0) * Y(0, col) -
                   this->mat_(2, 1) * Y(1, col)) /
                  this->mat_(2, 2);
    }
  }

  inline void BackwardElimination(matrix::Vector<Scalar, 3>& x,
                                  const matrix::Vector<Scalar, 3>& y) const {
    x(2) = y(2) / this->mat_(2, 2);
    x(1) = (y(1) - this->mat_(2, 1) * x(2)) / this->mat_(1, 1);
    x(0) = (y(0) - this->mat_(1, 0) * x(1) - this->mat_(2, 0) * x(2)) /
           this->mat_(0, 0);
  }

  template <int RHS_DIMISIONS>
  inline void BackwardElimination(
      matrix::Matrix<Scalar, 3, RHS_DIMISIONS>& X,
      const matrix::Matrix<Scalar, 3, RHS_DIMISIONS>& Y) const {
    for (int col = 0; col < RHS_DIMISIONS; ++col) {
      X(2, col) = Y(2, col) / this->mat_(2, 2);
      X(1, col) = (Y(1, col) - this->mat_(2, 1) * X(2, col)) / this->mat_(1, 1);
      X(0, col) = (Y(0, col) - this->mat_(1, 0) * X(1, col) -
                   this->mat_(2, 0) * X(2, col)) /
                  this->mat_(0, 0);
    }
  }
};
