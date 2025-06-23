#pragma once
#include "CholeskyDecomposition.hpp"
/**
The problem is in the form:

min 0.5 * x G x + g0^T x
s.t.
    CE^T x + ce0 = 0

 The matrix and vectors dimensions are as follows:
     G: n * n
        g0: n

        CE: n * p rank(CE) = p
     ce0: p

     x: n

use KKT Method To Solve Equal Constrains quadritic problem
(G      CE)(x)   ( -g0)
(         )( ) = (    )
(CE^T   0 )(r)   (-ce0)

CE^T * G^-1 * CE * r = -CE^T * G^-1 * g0 + ce0
G * x = -g0 - CE * r
 */
template <typename Scalar, int DIMISIONS, int EQST>
class KKTMethod
{
public:
    KKTMethod(const Scalar error_bound = FLT_EPSILON) : error_bound_(error_bound) {};
    ~KKTMethod() = default;
    void InitProblemMatrix(const Matrix<Scalar, DIMISIONS, EQST> &CE, const Vector<Scalar, DIMISIONS> &g0, const Matrix<Scalar, DIMISIONS, DIMISIONS> &G_inv)
    {
        A_.Decomposition(CE.transpose() * G_inv * CE);
        B_ = -CE.transpose() * G_inv * g0;
        C_ = -G_inv * g0;
        D_ = -G_inv * CE;
    }
    void Solve(const Vector<Scalar, EQST> &ce0)
    {
        Vector<Scalar, EQST> y = B_ + ce0;
        Vector<Scalar, EQST> r;
        A_.Solve(r, y);

        x_ = C_ + D_ * r;
    }
    inline Vector<Scalar, DIMISIONS> GetOptimalVector()
    {
        return x_;
    }

private:
    Scalar error_bound_;

    // A * r = B + ce0
    // x = C + D * r
    CholeskyDecomposition<Scalar, EQST> A_; // A = CE^T * G^-1 * CE
    Vector<Scalar, EQST> B_;                // B = -CE^T * G^-1 * g0
    Vector<Scalar, DIMISIONS> C_;           // C = -G^-1 * g0
    Matrix<Scalar, DIMISIONS, EQST> D_;     // D = -G^-1 * CE

    Vector<Scalar, DIMISIONS> x_;
};

template <typename Scalar, int DIMISIONS>
class KKTMethod<Scalar, DIMISIONS, 0>
{
public:
    KKTMethod(const Scalar error_bound = FLT_EPSILON) : error_bound_(error_bound) {};
    ~KKTMethod() = default;
    void InitProblemMatrix(const Vector<Scalar, DIMISIONS> &g0, const Matrix<Scalar, DIMISIONS, DIMISIONS> &G_inv)
    {
        x_ = -G_inv * g0;
    }
    void Solve()
    {
        return;
    }
    inline Vector<Scalar, DIMISIONS> GetOptimalVector()
    {
        return x_;
    }

private:
    Scalar error_bound_;

    Vector<Scalar, DIMISIONS> x_;
};
