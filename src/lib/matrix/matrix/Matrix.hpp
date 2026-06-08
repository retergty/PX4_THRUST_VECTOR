/**
 * @file Matrix.hpp
 *
 * A simple matrix template library.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

#include "matrix/Slice.hpp"
#include "matrix/helper_functions.hpp"

namespace matrix {

constexpr int Infinity = -1;

template <typename Type, int M, int N>
class Matrix {
  static_assert(M >= 0, "Matrix row count must be non-negative");
  static_assert(N >= 0, "Matrix column count must be non-negative");

  Type _data[M][N]{};

  void assignFromInitializerList(std::initializer_list<Type> values) {
    assert(values.size() == M * N);

    size_t index = 0;

    for (const Type& value : values) {
      if (index >= M * N) {
        break;
      }

      _data[index / N][index % N] = value;
      ++index;
    }
  }

  void assignFromInitializerList(
      std::initializer_list<std::initializer_list<Type>> values) {
    assert(values.size() == M);

    size_t i = 0;

    for (const auto& row : values) {
      if (i >= M) {
        break;
      }

      assert(row.size() == N);

      size_t j = 0;

      for (const Type& value : row) {
        if (j >= N) {
          break;
        }

        _data[i][j] = value;
        ++j;
      }

      ++i;
    }
  }

 public:
  // Constructors
  Matrix() = default;

  Matrix(std::initializer_list<Type> values) {
    assignFromInitializerList(values);
  }

  Matrix(std::initializer_list<std::initializer_list<Type>> values) {
    assignFromInitializerList(values);
  }

  explicit Matrix(const Type data_[M * N]) {
    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        _data[i][j] = data_[N * i + j];
      }
    }
  }

  explicit Matrix(const Type data_[M][N]) {
    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        _data[i][j] = data_[i][j];
      }
    }
  }

  Matrix(const Matrix& other) {
    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        _data[i][j] = other(i, j);
      }
    }
  }

  template <int P, int Q>
  Matrix(const Slice<Type, M, N, P, Q>& in_slice) {
    Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        self(i, j) = in_slice(i, j);
      }
    }
  }

  template <int P, int Q>
  Matrix(const ConstSlice<Type, M, N, P, Q>& in_slice) {
    Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        self(i, j) = in_slice(i, j);
      }
    }
  }

  static Matrix<Type, M, N> Zero() {
    Matrix<Type, M, N> result;
    result.setZero();
    return result;
  }

  static Matrix<Type, M, N> Ones() {
    Matrix<Type, M, N> result;
    result.setOne();
    return result;
  }

  static Matrix<Type, M, N> Constant(Type value) {
    Matrix<Type, M, N> result;
    result.setAll(value);
    return result;
  }

  static Matrix<Type, M, N> Identity() {
    Matrix<Type, M, N> result;
    result.setIdentity();
    return result;
  }

  static Matrix<Type, M, N> Random() {
    Matrix<Type, M, N> result;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        result(i, j) = Type(2) * Type(std::rand()) / Type(RAND_MAX) - Type(1);
      }
    }

    return result;
  }

  /**
   * Accessors/ Assignment etc.
   */

  inline const Type& operator()(size_t i, size_t j) const {
    assert(i < M);
    assert(j < N);

    return _data[i][j];
  }

  inline Type& operator()(size_t i, size_t j) {
    assert(i < M);
    assert(j < N);

    return _data[i][j];
  }

  Matrix<Type, M, N>& operator=(const Matrix<Type, M, N>& other) {
    if (this != &other) {
      Matrix<Type, M, N>& self = *this;

      for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
          self(i, j) = other(i, j);
        }
      }
    }

    return (*this);
  }

  Matrix<Type, M, N>& operator=(std::initializer_list<Type> values) {
    assignFromInitializerList(values);
    return (*this);
  }

  Matrix<Type, M, N>& operator=(
      std::initializer_list<std::initializer_list<Type>> values) {
    assignFromInitializerList(values);
    return (*this);
  }

  void copyTo(Type dst[M * N]) const {
    const Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        dst[N * i + j] = self(i, j);
      }
    }
  }

  void copyToColumnMajor(Type dst[M * N]) const {
    const Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        dst[i + (j * M)] = self(i, j);
      }
    }
  }

  /**
   * Matrix Operations
   */

  // this might use a lot of programming memory
  // since it instantiates a class for every
  // required mult pair, but it provides
  // compile time size checking
  template <int P>
  Matrix<Type, M, P> operator*(const Matrix<Type, N, P>& other) const {
    const Matrix<Type, M, N>& self = *this;
    Matrix<Type, M, P> res{};

    for (size_t i = 0; i < M; i++) {
      for (size_t k = 0; k < P; k++) {
        for (size_t j = 0; j < N; j++) {
          res(i, k) += self(i, j) * other(j, k);
        }
      }
    }

    return res;
  }

  // Using this function reduces the number of temporary variables needed to
  // compute A * B.T
  template <int P>
  Matrix<Type, M, P> multiplyByTranspose(
      const Matrix<Type, P, N>& other) const {
    Matrix<Type, M, P> res;
    const Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t k = 0; k < P; k++) {
        for (size_t j = 0; j < N; j++) {
          res(i, k) += self(i, j) * other(k, j);
        }
      }
    }

    return res;
  }

  // Compute A.T * B without constructing A.T.
  template <int P>
  Matrix<Type, N, P> transposeMultiply(const Matrix<Type, M, P>& other) const {
    Matrix<Type, N, P> res{};
    const Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < N; i++) {
      for (size_t k = 0; k < P; k++) {
        for (size_t j = 0; j < M; j++) {
          res(i, k) += self(j, i) * other(j, k);
        }
      }
    }

    return res;
  }

  // Accumulate this += lhs.T * rhs without constructing lhs.T or lhs.T * rhs.
  template <int LhsRows>
  void addTransposeProduct(const Matrix<Type, LhsRows, M>& lhs,
                           const Matrix<Type, LhsRows, N>& rhs) {
    Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t k = 0; k < N; k++) {
        Type sum = self(i, k);
        for (size_t j = 0; j < LhsRows; j++) {
          sum += lhs(j, i) * rhs(j, k);
        }
        self(i, k) = sum;
      }
    }
  }

  // Element-wise multiplication
  Matrix<Type, M, N> emult(const Matrix<Type, M, N>& other) const {
    Matrix<Type, M, N> res;
    const Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        res(i, j) = self(i, j) * other(i, j);
      }
    }

    return res;
  }

  // Element-wise division
  Matrix<Type, M, N> edivide(const Matrix<Type, M, N>& other) const {
    Matrix<Type, M, N> res;
    const Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        res(i, j) = self(i, j) / other(i, j);
      }
    }

    return res;
  }

  Matrix<Type, M, N> operator+(const Matrix<Type, M, N>& other) const {
    Matrix<Type, M, N> res;
    const Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        res(i, j) = self(i, j) + other(i, j);
      }
    }

    return res;
  }

  Matrix<Type, M, N> operator-(const Matrix<Type, M, N>& other) const {
    Matrix<Type, M, N> res;
    const Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        res(i, j) = self(i, j) - other(i, j);
      }
    }

    return res;
  }

  // unary minus
  Matrix<Type, M, N> operator-() const {
    Matrix<Type, M, N> res;
    const Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        res(i, j) = -self(i, j);
      }
    }

    return res;
  }

  void operator+=(const Matrix<Type, M, N>& other) {
    Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        self(i, j) += other(i, j);
      }
    }
  }

  void operator-=(const Matrix<Type, M, N>& other) {
    Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        self(i, j) -= other(i, j);
      }
    }
  }

  template <int P>
  void operator*=(const Matrix<Type, N, P>& other) {
    Matrix<Type, M, N>& self = *this;
    self = self * other;
  }

  /**
   * Scalar Operations
   */

  Matrix<Type, M, N> operator*(Type scalar) const {
    Matrix<Type, M, N> res;
    const Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        res(i, j) = self(i, j) * scalar;
      }
    }

    return res;
  }

  inline Matrix<Type, M, N> operator/(Type scalar) const {
    return (*this) * (1 / scalar);
  }

  Matrix<Type, M, N> operator+(Type scalar) const {
    Matrix<Type, M, N> res;
    const Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        res(i, j) = self(i, j) + scalar;
      }
    }

    return res;
  }

  inline Matrix<Type, M, N> operator-(Type scalar) const {
    return (*this) + (-1 * scalar);
  }

  void operator*=(Type scalar) {
    Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        self(i, j) *= scalar;
      }
    }
  }

  void operator/=(Type scalar) {
    Matrix<Type, M, N>& self = *this;
    self *= (Type(1) / scalar);
  }

  inline void operator+=(Type scalar) {
    Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        self(i, j) += scalar;
      }
    }
  }

  inline void operator-=(Type scalar) {
    Matrix<Type, M, N>& self = *this;
    self += (-scalar);
  }

  bool operator==(const Matrix<Type, M, N>& other) const {
    return isEqual(*this, other);
  }

  bool isApprox(const Matrix<Type, M, N>& other,
                const Type eps = Type(1e-4)) const {
    return isEqual(*this, other, eps);
  }

  bool isZero(const Type eps = Type(1e-4)) const {
    return isApprox(Matrix<Type, M, N>::Zero(), eps);
  }

  bool operator!=(const Matrix<Type, M, N>& other) const {
    const Matrix<Type, M, N>& self = *this;
    return !(self == other);
  }

  template <int P>
  Type lpNorm() const {
    static_assert(P == Infinity,
                  "Only lpNorm<matrix::Infinity>() is supported.");

    Type max_val(0);
    const Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        const Type abs_val = Type(std::fabs(self(i, j)));
        if (abs_val > max_val) {
          max_val = abs_val;
        }
      }
    }

    return max_val;
  }

  /**
   * Misc. Functions
   */

  void write_string(char* buf, size_t n) const {
    buf[0] = '\0';  // make an empty string to begin with (we need the '\0' for
                    // strlen to work)
    const Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        snprintf(buf + strlen(buf), n - strlen(buf), "\t%8.8g",
                 double(self(i, j)));  // directly append to the string buffer
      }

      snprintf(buf + strlen(buf), n - strlen(buf), "\n");
    }
  }

  void print(float eps = 1e-9) const {
    // print column numbering
    if (N > 1) {
      printf("  ");

      for (unsigned i = 0; i < N; i++) {
        printf("|%2u      ", i);
      }

      printf("\n");
    }

    const Matrix<Type, M, N>& self = *this;
    bool is_prev_symmetric = true;  // assume symmetric until one element is not

    for (unsigned i = 0; i < M; i++) {
      printf("%2u|", i);  // print row numbering

      for (unsigned j = 0; j < N; j++) {
        double d = static_cast<double>(self(i, j));

        // if symmetric don't print upper triangular elements
        if (is_prev_symmetric && (M == N) && (j > i) && (i < N) && (j < M) &&
            (fabs(d - static_cast<double>(self(j, i))) < (double)eps)) {
          // print empty space
          printf("         ");

        } else {
          // avoid -0.0 for display
          if (fabs(d - 0.0) < (double)eps) {
            // print fixed width zero
            printf(" 0       ");

          } else if ((fabs(d) < 1e-4) || (fabs(d) >= 10.0)) {
            printf("% .1e ", d);

          } else {
            printf("% 6.5f ", d);
          }

          is_prev_symmetric = false;  // not symmetric if once inside here
        }
      }

      printf("\n");
    }
  }

  Matrix<Type, N, M> transpose() const {
    Matrix<Type, N, M> res;
    const Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        res(j, i) = self(i, j);
      }
    }

    return res;
  }

  // tranpose alias
  inline Matrix<Type, N, M> T() const { return transpose(); }

  template <int P, int Q>
  ConstSlice<Type, P, Q, M, N> slice(size_t x0, size_t y0) const {
    return {x0, y0, this};
  }

  template <int P, int Q>
  Slice<Type, P, Q, M, N> slice(size_t x0, size_t y0) {
    return {x0, y0, this};
  }

  template <int R, int C>
  ConstSlice<Type, R, C, M, N> topLeftCorner() const {
    static_assert(R <= M, "Corner rows bigger than matrix");
    static_assert(C <= N, "Corner cols bigger than matrix");
    return slice<R, C>(0, 0);
  }

  template <int R, int C>
  Slice<Type, R, C, M, N> topLeftCorner() {
    static_assert(R <= M, "Corner rows bigger than matrix");
    static_assert(C <= N, "Corner cols bigger than matrix");
    return slice<R, C>(0, 0);
  }

  template <int R, int C>
  ConstSlice<Type, R, C, M, N> topRightCorner() const {
    static_assert(R <= M, "Corner rows bigger than matrix");
    static_assert(C <= N, "Corner cols bigger than matrix");
    return slice<R, C>(0, N - C);
  }

  template <int R, int C>
  Slice<Type, R, C, M, N> topRightCorner() {
    static_assert(R <= M, "Corner rows bigger than matrix");
    static_assert(C <= N, "Corner cols bigger than matrix");
    return slice<R, C>(0, N - C);
  }

  template <int R, int C>
  ConstSlice<Type, R, C, M, N> bottomLeftCorner() const {
    static_assert(R <= M, "Corner rows bigger than matrix");
    static_assert(C <= N, "Corner cols bigger than matrix");
    return slice<R, C>(M - R, 0);
  }

  template <int R, int C>
  Slice<Type, R, C, M, N> bottomLeftCorner() {
    static_assert(R <= M, "Corner rows bigger than matrix");
    static_assert(C <= N, "Corner cols bigger than matrix");
    return slice<R, C>(M - R, 0);
  }

  template <int R, int C>
  ConstSlice<Type, R, C, M, N> bottomRightCorner() const {
    static_assert(R <= M, "Corner rows bigger than matrix");
    static_assert(C <= N, "Corner cols bigger than matrix");
    return slice<R, C>(M - R, N - C);
  }

  template <int R, int C>
  Slice<Type, R, C, M, N> bottomRightCorner() {
    static_assert(R <= M, "Corner rows bigger than matrix");
    static_assert(C <= N, "Corner cols bigger than matrix");
    return slice<R, C>(M - R, N - C);
  }

  template <int R>
  ConstSlice<Type, R, N, M, N> topRows() const {
    static_assert(R <= M, "Rows bigger than matrix");
    return slice<R, N>(0, 0);
  }

  template <int R>
  Slice<Type, R, N, M, N> topRows() {
    static_assert(R <= M, "Rows bigger than matrix");
    return slice<R, N>(0, 0);
  }

  template <int R>
  ConstSlice<Type, R, N, M, N> bottomRows() const {
    static_assert(R <= M, "Rows bigger than matrix");
    return slice<R, N>(M - R, 0);
  }

  template <int R>
  Slice<Type, R, N, M, N> bottomRows() {
    static_assert(R <= M, "Rows bigger than matrix");
    return slice<R, N>(M - R, 0);
  }

  ConstSlice<Type, 1, N, M, N> row(size_t i) const { return slice<1, N>(i, 0); }

  Slice<Type, 1, N, M, N> row(size_t i) { return slice<1, N>(i, 0); }

  ConstSlice<Type, M, 1, M, N> col(size_t j) const { return slice<M, 1>(0, j); }

  Slice<Type, M, 1, M, N> col(size_t j) { return slice<M, 1>(0, j); }

  void setRow(size_t i, const Matrix<Type, N, 1>& row_in) {
    slice<1, N>(i, 0) = row_in.transpose();
  }

  void setRow(size_t i, Type val) { slice<1, N>(i, 0) = val; }

  void setCol(size_t j, const Matrix<Type, M, 1>& column) {
    slice<M, 1>(0, j) = column;
  }

  void setCol(size_t j, Type val) { slice<M, 1>(0, j) = val; }

  void setZero() { memset(_data, 0, sizeof(_data)); }

  inline void zero() { setZero(); }

  void setAll(Type val) {
    Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        self(i, j) = val;
      }
    }
  }

  inline void setOne() { setAll(1); }

  inline void setNaN() { setAll(NAN); }

  void setIdentity() {
    setZero();
    Matrix<Type, M, N>& self = *this;

    const size_t min_i = M > N ? N : M;

    for (size_t i = 0; i < min_i; i++) {
      self(i, i) = 1;
    }
  }

  inline void identity() { setIdentity(); }

  void swap(Matrix<Type, M, N>& other) {
    Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        Type tmp = self(i, j);
        self(i, j) = other(i, j);
        other(i, j) = tmp;
      }
    }
  }

  inline void swapRows(size_t a, size_t b) {
    assert(a < M);
    assert(b < M);

    if (a == b) {
      return;
    }

    Matrix<Type, M, N>& self = *this;

    for (size_t j = 0; j < N; j++) {
      Type tmp = self(a, j);
      self(a, j) = self(b, j);
      self(b, j) = tmp;
    }
  }

  inline void swapCols(size_t a, size_t b) {
    assert(a < N);
    assert(b < N);

    if (a == b) {
      return;
    }

    Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      Type tmp = self(i, a);
      self(i, a) = self(i, b);
      self(i, b) = tmp;
    }
  }

  Matrix<Type, M, N> abs() const {
    Matrix<Type, M, N> r;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        r(i, j) = Type(std::fabs((*this)(i, j)));
      }
    }

    return r;
  }

  Type squaredNorm() const {
    Type accum(0);
    const Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        accum += self(i, j) * self(i, j);
      }
    }

    return accum;
  }

  Type norm() const { return Type(std::sqrt(squaredNorm())); }

  Type max() const {
    Type max_val = (*this)(0, 0);

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        Type val = (*this)(i, j);

        if (val > max_val) {
          max_val = val;
        }
      }
    }

    return max_val;
  }

  Type min() const {
    Type min_val = (*this)(0, 0);

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        Type val = (*this)(i, j);

        if (val < min_val) {
          min_val = val;
        }
      }
    }

    return min_val;
  }

  bool isAllNan() const {
    const Matrix<Type, M, N>& self = *this;
    bool result = true;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        result = result && std::isnan(self(i, j));
      }
    }

    return result;
  }

  bool isAllFinite() const {
    const Matrix<Type, M, N>& self = *this;

    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        if (!std::isfinite(self(i, j))) {
          return false;
        }
      }
    }

    return true;
  }
};

template <typename Type, int M, int N>
Matrix<Type, M, N> zeros() {
  Matrix<Type, M, N> m;
  m.setZero();
  return m;
}

template <typename Type, int M, int N>
Matrix<Type, M, N> ones() {
  Matrix<Type, M, N> m;
  m.setOne();
  return m;
}

template <int M, int N>
Matrix<float, M, N> nans() {
  Matrix<float, M, N> m;
  m.setNaN();
  return m;
}

template <typename Type, int M, int N>
Matrix<Type, M, N> operator*(Type scalar, const Matrix<Type, M, N>& other) {
  return other * scalar;
}

template <typename Type, int M, int N>
bool isEqual(const Matrix<Type, M, N>& x, const Matrix<Type, M, N>& y,
             const Type eps = Type(1e-4)) {
  for (size_t i = 0; i < M; i++) {
    for (size_t j = 0; j < N; j++) {
      if (!isEqualF(x(i, j), y(i, j), eps)) {
        return false;
      }
    }
  }

  return true;
}

namespace typeFunction {
template <typename Type>
Type min(const Type x, const Type y) {
  bool x_is_nan = std::isnan(x);
  bool y_is_nan = std::isnan(y);

  // take the non-nan value if there is one
  if (x_is_nan || y_is_nan) {
    if (x_is_nan && !y_is_nan) {
      return y;
    }

    // either !x_is_nan && y_is_nan or both are NAN anyways
    return x;
  }

  return (x < y) ? x : y;
}

template <typename Type>
Type max(const Type x, const Type y) {
  bool x_is_nan = std::isnan(x);
  bool y_is_nan = std::isnan(y);

  // take the non-nan value if there is one
  if (x_is_nan || y_is_nan) {
    if (x_is_nan && !y_is_nan) {
      return y;
    }

    // either !x_is_nan && y_is_nan or both are NAN anyways
    return x;
  }

  return (x > y) ? x : y;
}

template <typename Type>
Type constrain(const Type x, const Type lower_bound, const Type upper_bound) {
  if (lower_bound > upper_bound) {
    return NAN;

  } else if (std::isnan(x)) {
    return NAN;

  } else {
    return typeFunction::max(lower_bound, typeFunction::min(upper_bound, x));
  }
}
}  // namespace typeFunction

template <typename Type, int M, int N>
Matrix<Type, M, N> min(const Matrix<Type, M, N>& x,
                       const Type scalar_upper_bound) {
  Matrix<Type, M, N> m;

  for (size_t i = 0; i < M; i++) {
    for (size_t j = 0; j < N; j++) {
      m(i, j) = typeFunction::min(x(i, j), scalar_upper_bound);
    }
  }

  return m;
}

template <typename Type, int M, int N>
Matrix<Type, M, N> min(const Type scalar_upper_bound,
                       const Matrix<Type, M, N>& x) {
  return min(x, scalar_upper_bound);
}

template <typename Type, int M, int N>
Matrix<Type, M, N> min(const Matrix<Type, M, N>& x1,
                       const Matrix<Type, M, N>& x2) {
  Matrix<Type, M, N> m;

  for (size_t i = 0; i < M; i++) {
    for (size_t j = 0; j < N; j++) {
      m(i, j) = typeFunction::min(x1(i, j), x2(i, j));
    }
  }

  return m;
}

template <typename Type, int M, int N>
Matrix<Type, M, N> max(const Matrix<Type, M, N>& x,
                       const Type scalar_lower_bound) {
  Matrix<Type, M, N> m;

  for (size_t i = 0; i < M; i++) {
    for (size_t j = 0; j < N; j++) {
      m(i, j) = typeFunction::max(x(i, j), scalar_lower_bound);
    }
  }

  return m;
}

template <typename Type, int M, int N>
Matrix<Type, M, N> max(const Type scalar_lower_bound,
                       const Matrix<Type, M, N>& x) {
  return max(x, scalar_lower_bound);
}

template <typename Type, int M, int N>
Matrix<Type, M, N> max(const Matrix<Type, M, N>& x1,
                       const Matrix<Type, M, N>& x2) {
  Matrix<Type, M, N> m;

  for (size_t i = 0; i < M; i++) {
    for (size_t j = 0; j < N; j++) {
      m(i, j) = typeFunction::max(x1(i, j), x2(i, j));
    }
  }

  return m;
}

template <typename Type, int M, int N>
Matrix<Type, M, N> constrain(const Matrix<Type, M, N>& x,
                             const Type scalar_lower_bound,
                             const Type scalar_upper_bound) {
  Matrix<Type, M, N> m;

  if (scalar_lower_bound > scalar_upper_bound) {
    m.setNaN();

  } else {
    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < N; j++) {
        m(i, j) = typeFunction::constrain(x(i, j), scalar_lower_bound,
                                          scalar_upper_bound);
      }
    }
  }

  return m;
}

template <typename Type, int M, int N>
Matrix<Type, M, N> constrain(const Matrix<Type, M, N>& x,
                             const Matrix<Type, M, N>& x_lower_bound,
                             const Matrix<Type, M, N>& x_upper_bound) {
  Matrix<Type, M, N> m;

  for (size_t i = 0; i < M; i++) {
    for (size_t j = 0; j < N; j++) {
      m(i, j) = typeFunction::constrain(x(i, j), x_lower_bound(i, j),
                                        x_upper_bound(i, j));
    }
  }

  return m;
}

template <typename OStream, typename Type, int M, int N>
OStream& operator<<(OStream& os, const matrix::Matrix<Type, M, N>& matrix) {
  os << "\n";
  // element: tab, point, 8 digits, 4 scientific notation chars; row: newline;
  // string: \0 end
  static const size_t n = 15 * N * M + M + 1;
  char string[n];
  matrix.write_string(string, n);
  os << string;
  return os;
}

}  // namespace matrix
