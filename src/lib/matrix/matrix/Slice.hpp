/**
 * @file Slice.hpp
 *
 * A simple matrix template library.
 *
 * @author Julian Kent < julian@auterion.com >
 */

#pragma once

#include <cassert>
#include <cmath>
#include <cstdio>

#include "matrix/helper_functions.hpp"

namespace matrix {

template <typename Type, int M, int N>
class Matrix;

template <typename Type, int M>
class Vector;

template <typename MatrixT, typename Type, int P, int Q, int M, int N>
class SliceT {
  static_assert(P >= 0, "Slice row count must be non-negative");
  static_assert(Q >= 0, "Slice column count must be non-negative");
  static_assert(M >= 0, "Backing matrix row count must be non-negative");
  static_assert(N >= 0, "Backing matrix column count must be non-negative");

 public:
  using Self = SliceT<MatrixT, Type, P, Q, M, N>;

  SliceT(size_t x0, size_t y0, MatrixT* data) : _x0(x0), _y0(y0), _data(data) {
    static_assert(P <= M, "Slice rows bigger than backing matrix");
    static_assert(Q <= N, "Slice cols bigger than backing matrix");
    assert(x0 + P <= M);
    assert(y0 + Q <= N);
  }

  SliceT(const Self& other) = default;

  const Type& operator()(size_t i, size_t j) const {
    assert(i < P);
    assert(j < Q);

    return (*_data)(_x0 + i, _y0 + j);
  }

  Type& operator()(size_t i, size_t j) {
    assert(i < P);
    assert(j < Q);

    return (*_data)(_x0 + i, _y0 + j);
  }

  // Separate function needed otherwise the default copy constructor matches
  // before the deep copy implementation
  Self& operator=(const Self& other) { return this->operator= <M, N>(other); }

  template <int MM, int NN>
  Self& operator=(
      const SliceT<Matrix<Type, MM, NN>, Type, P, Q, MM, NN>& other) {
    Self& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        self(i, j) = other(i, j);
      }
    }

    return self;
  }

  template <int MM, int NN>
  SliceT<MatrixT, Type, P, Q, M, N>& operator=(
      const SliceT<const Matrix<Type, MM, NN>, Type, P, Q, MM, NN>& other) {
    SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        self(i, j) = other(i, j);
      }
    }

    return self;
  }

  SliceT<MatrixT, Type, P, Q, M, N>& operator=(
      const Matrix<Type, P, Q>& other) {
    SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        self(i, j) = other(i, j);
      }
    }

    return self;
  }

  SliceT<MatrixT, Type, P, Q, M, N>& operator=(const Type& other) {
    SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        self(i, j) = other;
      }
    }

    return self;
  }

  template <int MM, int NN>
  Matrix<Type, P, Q> operator-(
      const SliceT<const Matrix<Type, MM, NN>, Type, P, Q, MM, NN>& other) {
    Matrix<Type, P, Q> result;
    const SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        result(i, j) = self(i, j) - other(i, j);
      }
    }

    return result;
  }

  Matrix<Type, P, Q> operator-(const Matrix<Type, P, Q>& other) {
    Matrix<Type, P, Q> result;
    const SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        result(i, j) = self(i, j) - other(i, j);
      }
    }

    return result;
  }

  Matrix<Type, P, Q> operator-(const Type& other) {
    Matrix<Type, P, Q> result;
    const SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        result(i, j) = self(i, j) - other;
      }
    }

    return result;
  }

  template <int MM, int NN>
  Matrix<Type, P, Q> operator+(
      const SliceT<const Matrix<Type, MM, NN>, Type, P, Q, MM, NN>& other) {
    Matrix<Type, P, Q> result;
    const SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        result(i, j) = self(i, j) + other(i, j);
      }
    }

    return result;
  }

  Matrix<Type, P, Q> operator+(const Matrix<Type, P, Q>& other) {
    Matrix<Type, P, Q> result;
    const SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        result(i, j) = self(i, j) + other(i, j);
      }
    }

    return result;
  }

  Matrix<Type, P, Q> operator+(const Type& other) {
    Matrix<Type, P, Q> result;
    const SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        result(i, j) = self(i, j) + other;
      }
    }

    return result;
  }

  // allow assigning vectors to a slice that are in the axis
  template <int DUMMY = 1>  // make this a template function since it only
                            // exists for some instantiations
  SliceT<MatrixT, Type, 1, Q, M, N>& operator=(const Vector<Type, Q>& other) {
    SliceT<MatrixT, Type, 1, Q, M, N>& self = *this;

    for (size_t j = 0; j < Q; j++) {
      self(0, j) = other(j);
    }

    return self;
  }

  template <int MM, int NN>
  SliceT<MatrixT, Type, P, Q, M, N>& operator+=(
      const SliceT<MatrixT, Type, P, Q, MM, NN>& other) {
    SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        self(i, j) += other(i, j);
      }
    }

    return self;
  }

  SliceT<MatrixT, Type, P, Q, M, N>& operator+=(
      const Matrix<Type, P, Q>& other) {
    SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        self(i, j) += other(i, j);
      }
    }

    return self;
  }

  SliceT<MatrixT, Type, P, Q, M, N>& operator+=(const Type& other) {
    SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        self(i, j) += other;
      }
    }

    return self;
  }

  template <int MM, int NN>
  SliceT<MatrixT, Type, P, Q, M, N>& operator-=(
      const SliceT<MatrixT, Type, P, Q, MM, NN>& other) {
    SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        self(i, j) -= other(i, j);
      }
    }

    return self;
  }

  SliceT<MatrixT, Type, P, Q, M, N>& operator-=(
      const Matrix<Type, P, Q>& other) {
    SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        self(i, j) -= other(i, j);
      }
    }

    return self;
  }

  SliceT<MatrixT, Type, P, Q, M, N>& operator-=(const Type& other) {
    SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        self(i, j) -= other;
      }
    }

    return self;
  }

  SliceT<MatrixT, Type, P, Q, M, N>& operator*=(const Type& other) {
    SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        self(i, j) *= other;
      }
    }

    return self;
  }

  SliceT<MatrixT, Type, P, Q, M, N>& operator/=(const Type& scalar) {
    return operator*=(Type(1) / scalar);
  }

  void setAll(Type val) { (*this) = val; }

  void setZero() { setAll(Type(0)); }

  void setIdentity() {
    setZero();
    SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    const size_t min_i = P > Q ? Q : P;

    for (size_t i = 0; i < min_i; i++) {
      self(i, i) = Type(1);
    }
  }

  Matrix<Type, P, Q> operator*(const Type& scalar) const {
    Matrix<Type, P, Q> result;
    const SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        result(i, j) = self(i, j) * scalar;
      }
    }

    return result;
  }

  Matrix<Type, P, Q> operator/(const Type& scalar) const {
    return (*this) * (1 / scalar);
  }

  template <int R>
  Matrix<Type, P, R> operator*(const Matrix<Type, Q, R>& other) const {
    Matrix<Type, P, R> result;
    const SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t k = 0; k < R; k++) {
        for (size_t j = 0; j < Q; j++) {
          result(i, k) += self(i, j) * other(j, k);
        }
      }
    }

    return result;
  }

  template <int R, int S>
  const SliceT<MatrixT, Type, R, S, M, N> slice(size_t x0, size_t y0) const {
    return SliceT<MatrixT, Type, R, S, M, N>(x0 + _x0, y0 + _y0, _data);
  }

  template <int R, int S>
  SliceT<MatrixT, Type, R, S, M, N> slice(size_t x0, size_t y0) {
    return SliceT<MatrixT, Type, R, S, M, N>(x0 + _x0, y0 + _y0, _data);
  }

  void copyTo(Type dst[P * Q]) const {
    const SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        dst[i * N + j] = self(i, j);
      }
    }
  }

  void copyToColumnMajor(Type dst[P * Q]) const {
    const SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        dst[i + (j * M)] = self(i, j);
      }
    }
  }

  Vector < Type, P<Q ? P : Q> diag() const {
    const SliceT<MatrixT, Type, P, Q, M, N>& self = *this;
    Vector < Type, P<Q ? P : Q> res;

    for (size_t j = 0; j < (P < Q ? P : Q); j++) {
      res(j) = self(j, j);
    }

    return res;
  }

  Type norm_squared() const {
    const SliceT<MatrixT, Type, P, Q, M, N>& self = *this;
    Type accum(0);

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        accum += self(i, j) * self(i, j);
      }
    }

    return accum;
  }

  Type squaredNorm() const { return norm_squared(); }

  Type norm() const { return std::sqrt(norm_squared()); }

  bool isZero(const Type eps = Type(1e-4f)) const {
    const SliceT<MatrixT, Type, P, Q, M, N>& self = *this;

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        if (!isEqualF(self(i, j), Type(0), eps)) {
          return false;
        }
      }
    }

    return true;
  }

  bool longerThan(Type testVal) const {
    return norm_squared() > testVal * testVal;
  }

  Type max() const {
    Type max_val = (*this)(0, 0);

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
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

    for (size_t i = 0; i < P; i++) {
      for (size_t j = 0; j < Q; j++) {
        Type val = (*this)(i, j);

        if (val < min_val) {
          min_val = val;
        }
      }
    }

    return min_val;
  }

 private:
  size_t _x0, _y0;
  MatrixT* _data;
};

template <typename Type, int P, int Q, int M, int N>
using Slice = SliceT<Matrix<Type, M, N>, Type, P, Q, M, N>;

template <typename Type, int P, int Q, int M, int N>
using ConstSlice = SliceT<const Matrix<Type, M, N>, Type, P, Q, M, N>;

}  // namespace matrix
