/**
 * @file Vector.hpp
 *
 * Vector class.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "matrix/Matrix.hpp"

namespace matrix {

template <typename Type, int M>
class Vector : public Matrix<Type, M, 1> {
  static_assert(M >= 0, "Vector dimension must be non-negative");

 public:
  using MatrixM1 = Matrix<Type, M, 1>;
  using MatrixM1::MatrixM1;
  using MatrixM1::operator=;
  using MatrixM1::operator*;

  Vector() = default;

  Vector(const MatrixM1& other) : MatrixM1(other) {}

  explicit Vector(const Type data_[M]) : MatrixM1(data_) {}

  template <int P, int Q>
  Vector(const Slice<Type, M, 1, P, Q>& slice_in)
      : Matrix<Type, M, 1>(slice_in) {}

  template <int P, int Q, int DUMMY = 1>
  Vector(const Slice<Type, 1, M, P, Q>& slice_in) {
    Vector& self(*this);

    for (size_t i = 0; i < M; i++) {
      self(i) = slice_in(0, i);
    }
  }

  template <int P, int Q>
  Vector(const ConstSlice<Type, M, 1, P, Q>& slice_in)
      : Matrix<Type, M, 1>(slice_in) {}

  template <int P, int Q>
  Vector& operator=(const Slice<Type, M, 1, P, Q>& slice_in) {
    Vector& self(*this);

    for (size_t i = 0; i < M; i++) {
      self(i) = slice_in(i, 0);
    }

    return self;
  }

  template <int P, int Q>
  Vector& operator=(const ConstSlice<Type, M, 1, P, Q>& slice_in) {
    Vector& self(*this);

    for (size_t i = 0; i < M; i++) {
      self(i) = slice_in(i, 0);
    }

    return self;
  }

  template <int P, int Q, int DUMMY = 1>
  Vector& operator=(const Slice<Type, 1, M, P, Q>& slice_in) {
    Vector& self(*this);

    for (size_t i = 0; i < M; i++) {
      self(i) = slice_in(0, i);
    }

    return self;
  }

  template <int P, int Q, int DUMMY = 1>
  Vector& operator=(const ConstSlice<Type, 1, M, P, Q>& slice_in) {
    Vector& self(*this);

    for (size_t i = 0; i < M; i++) {
      self(i) = slice_in(0, i);
    }

    return self;
  }

  template <int P, int Q, int DUMMY = 1>
  Vector(const ConstSlice<Type, 1, M, P, Q>& slice_in) {
    Vector& self(*this);

    for (size_t i = 0; i < M; i++) {
      self(i) = slice_in(0, i);
    }
  }

  static Vector Zero() {
    Vector result;
    result.setZero();
    return result;
  }

  static Vector Ones() {
    Vector result;
    result.setOne();
    return result;
  }

  static Vector Constant(Type value) {
    Vector result;
    result.setAll(value);
    return result;
  }

  static Vector Identity() {
    Vector result;
    result.setIdentity();
    return result;
  }

  static Vector Random() {
    Vector result;

    for (size_t i = 0; i < M; i++) {
      result(i) = Type(2) * Type(std::rand()) / Type(RAND_MAX) - Type(1);
    }

    return result;
  }

  template <int N>
  ConstSlice<Type, N, 1, M, 1> head() const {
    static_assert(N <= M, "Head size bigger than vector");
    return this->template slice<N, 1>(0, 0);
  }

  template <int N>
  Slice<Type, N, 1, M, 1> head() {
    static_assert(N <= M, "Head size bigger than vector");
    return this->template slice<N, 1>(0, 0);
  }

  template <int N>
  ConstSlice<Type, N, 1, M, 1> tail() const {
    static_assert(N <= M, "Tail size bigger than vector");
    return this->template slice<N, 1>(M - N, 0);
  }

  template <int N>
  Slice<Type, N, 1, M, 1> tail() {
    static_assert(N <= M, "Tail size bigger than vector");
    return this->template slice<N, 1>(M - N, 0);
  }

  inline const Type& operator()(size_t i) const {
    assert(i < M);

    const MatrixM1& v = *this;
    return v(i, 0);
  }

  inline Type& operator()(size_t i) {
    assert(i < M);

    MatrixM1& v = *this;
    return v(i, 0);
  }

  Type dot(const MatrixM1& b) const {
    const Vector& a(*this);
    Type r(0);

    for (size_t i = 0; i < M; i++) {
      r += a(i) * b(i, 0);
    }

    return r;
  }

  inline Type operator*(const Vector& b) const {
    const Vector& a(*this);
    return a.dot(b);
  }

  inline Vector operator*(Type b) const {
    return Vector(MatrixM1::operator*(b));
  }

  Type norm() const {
    const Vector& a(*this);
    return Type(std::sqrt(a.dot(a)));
  }

  Type norm_squared() const {
    const Vector& a(*this);
    return a.dot(a);
  }

  Type squaredNorm() const { return norm_squared(); }

  inline Type length() const { return norm(); }

  inline void normalize() { (*this) /= norm(); }

  Vector unit() const { return (*this) / norm(); }

  Vector unit_or_zero(const Type eps = Type(1e-5)) const {
    const Type n = norm();

    if (n > eps) {
      return (*this) / n;
    }

    return Vector();
  }

  inline Vector normalized() const { return unit(); }

  bool longerThan(Type testVal) const {
    return norm_squared() > testVal * testVal;
  }

  Vector sqrt() const {
    const Vector& a(*this);
    Vector r;

    for (size_t i = 0; i < M; i++) {
      r(i) = Type(std::sqrt(a(i)));
    }

    return r;
  }

  void print() const { (*this).transpose().print(); }

  static size_t size() { return M; }
};

template <typename OStream, typename Type, int M>
OStream& operator<<(OStream& os, const matrix::Vector<Type, M>& vector) {
  os << "\n";
  // element: tab, point, 8 digits, 4 scientific notation chars; row: newline;
  // string: \0 end
  static const size_t n = 15 * M * 1 + 1 + 1;
  char string[n];
  vector.transpose().write_string(string, n);
  os << string;
  return os;
}

}  // namespace matrix
