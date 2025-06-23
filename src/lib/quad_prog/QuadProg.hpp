#pragma once

#include <limits.h>

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

template <typename Type, int Rows, int Cols>
using Matrix = matrix::Matrix<Type, Rows, Cols>;
template <typename Type, int Rows>
using Vector = matrix::Vector<Type, Rows>;

#include "CholeskyDecomposition.hpp"
#include "KKTMethod.hpp"
/**
The problem is in the form:

min 0.5 * x G x + g0^T x
s.t.
    CE^T x + ce0 = 0
    CI^T x + ci0 >= 0

 The matrix and vectors dimensions are as follows:
     G: n * n
        g0: n

     CE: n * p
     ce0: p

     CI: n * m

     ci0: m

     x: n
 */
template <typename Scalar, int DIMISIONS, int EQST, int IEQST>
class QuadProg
{
public:
	enum class QuadProgState {
		Infeasible,
		Optimal,
		Init,
		UnconstrainMin, // step 0
		EqualConstrainMin,
		ChooseViolateConstrain,   // step 1
		CheckFeasibility,         // step 2
		DetermineStepDirection,   // step 2(a)
		ComputePartialStepLength, // step 2(b) i
		ComputeFullStepLength,    // step 2(b) ii
		DetermineStepLength,      // step 2(b) iii
		DetermineSPairTakeStep,   // step 2(c)
		NoStep,                   // step 2(c) i
		StepInDualSpace,          // step 2(c) ii
		StepInPrimalAndDualSpace  // step 2(c) iii
	};

	QuadProg(const Scalar error_bound = FLT_EPSILON) : error_bound_(error_bound), kkt_(error_bound)
	{
	}
	void init(const Matrix<Scalar, DIMISIONS, DIMISIONS> &G, const Vector<Scalar, DIMISIONS> &g0,
		  const Matrix<Scalar, DIMISIONS, EQST> &CE, const Vector<Scalar, EQST> &ce0,
		  const Matrix<Scalar, DIMISIONS, IEQST> &CI, const Vector<Scalar, IEQST> &ci0)
	{
		g0_ = g0;
		CE_ = CE;
		ce0_ = ce0;
		CI_ = CI;
		ci0_ = ci0;

		State_ = QuadProgState::Init;
		equal_constrians_calculate_ = false;

		InitCholeskyMatrix(G);
		InitKKTMatrix(G_.Inverse());
	}
	bool Solve()
	{
		Vector<Scalar, DIMISIONS> z;
		Vector<Scalar, DIMISIONS> r;
		Vector<Scalar, DIMISIONS> d;
		Vector<Scalar, DIMISIONS> np;
		Vector<Scalar, DIMISIONS> x_old;
		Vector<Scalar, DIMISIONS> u_old;
		Vector < Scalar, EQST + IEQST > u;
		Vector < Scalar, EQST + IEQST > s;
		Vector < int, EQST + IEQST > A; // active set index
		Vector < int, EQST + IEQST > A_old;
		Vector < int, EQST + IEQST > iai;
		Vector < bool, EQST + IEQST > iaexcl;

		int iq = 0;
		iter_ = 0;
		int ip = 0; /* ip will be the index of the chosen violated constraint */
		constexpr Scalar inf = 1e20f;
		int l = 0;
		x_old.setZero();
		d.setZero();

		Scalar step_length = 0;
		Scalar p_length = 0;
		Scalar f_length = 0;

		while (State_ != QuadProgState::Infeasible && State_ != QuadProgState::Optimal) {
			switch (State_) {
			case QuadProgState::Init:
			case QuadProgState::UnconstrainMin: {
					ComputeEqualConstrainMin();

					iq = EQST;

					for (int i = 0; i < EQST; ++i) {
						A(i) = -i - 1;
					}

					/* set iai = K \ A */
					for (int i = 0; i < IEQST; i++) {
						iai(i) = i;
					}

					State_ = QuadProgState::ChooseViolateConstrain;
				}
				break;

			case QuadProgState::ChooseViolateConstrain: {
					iter_++;

					/* step 1: choose a violated constraint */
					for (int i = EQST; i < iq; i++) {
						ip = A(i);
						iai(ip) = -1;
					}

					/* compute s[x] = ci^T * x + ci0 for all elements of K \ A */
					Scalar psi = 0; /* this value will contain the sum of all infeasibilities */
					ip = 0;           /* ip will be the index of the chosen violated constraint */

					for (int i = 0; i < IEQST; i++) {
						iaexcl(i) = true;
						Vector<Scalar, DIMISIONS> col_vec = CI_.col(i);
						s(i) = col_vec.dot(x_) + ci0_(i);
						psi += math::min<Scalar>(0, s(i));
					}

					if (fabsf(psi) <= IEQST * error_bound_ * cond1_ * cond2_ * 100) {
						/* numerically there are not infeasibilities anymore */
						State_ = QuadProgState::Optimal;

					} else {
						/* save old values for u and A */
						for (int i = 0; i < iq; i++) {
							u_old(i) = u(i);
							A_old(i) = A(i);
						}

						x_old = x_;
						State_ = QuadProgState::CheckFeasibility;
					}
				}
				break;

			case QuadProgState::CheckFeasibility: {
					Scalar ss = 0;

					for (int i = 0; i < IEQST; i++) {
						if (s(i) < ss && iai(i) != -1 && iaexcl(i)) {
							ss = s(i);
							ip = i;
						}
					}

					if (ss >= static_cast<Scalar>(0)) {
						State_ = QuadProgState::Optimal;

					} else {
						/* set np = n[ip] */
						np = CI_.col(ip);

						/* set u = [u 0]^T */
						u(iq) = 0;
						/* add ip to the active set A */
						A(iq) = ip;
						State_ = QuadProgState::DetermineStepDirection;
					}
				}
				break;

			case QuadProgState::DetermineStepDirection: {
					/* compute z = H np: the step direction in the primal space (through J, see the paper) */
					ComputeStepDirection(d, J_, np);
					ComputeStepDirectionInPrimalSpace(z, J_, d, iq);
					/* compute N* np (if q > 0): the negative of the step direction in the dual space */
					ComputeStepDirectionInDualSpace(r, R_, d, iq);
					l = 0;
					State_ = QuadProgState::ComputePartialStepLength;
				}
				break;

			case QuadProgState::ComputePartialStepLength: {
					/* Compute t1: partial step length (maximum step in dual space without violating dual feasibility */
					p_length = inf; /* +inf */

					/* find the index l s.t. it reaches the minimum of u+[x] / r */
					for (int k = EQST; k < iq; k++) {
						if (r(k) > 0) {
							if (u(k) / r(k) < p_length) {
								p_length = u(k) / r(k);
								l = A(k);
							}
						}
					}

					State_ = QuadProgState::ComputeFullStepLength;
				}
				break;

			case QuadProgState::ComputeFullStepLength: {
					/* Compute t2: full step length (minimum step in primal space such that the constraint ip becomes feasible */
					if (fabsf(z.dot(z)) > error_bound_) { // i.e. z != 0
						f_length = -s(ip) / z.dot(np);

						if (f_length < 0) { // patch suggested by Takano Akio for handling numerical inconsistencies
							f_length = inf;
						}

					} else {
						f_length = inf;        /* +inf */
					}

					State_ = QuadProgState::DetermineStepLength;
				}
				break;

			case QuadProgState::DetermineStepLength: {
					/* the step is chosen as the minimum of t1 and t2 */
					step_length = math::min(p_length, f_length);

					State_ = QuadProgState::DetermineSPairTakeStep;
				}
				break;

			case QuadProgState::DetermineSPairTakeStep: {
					/* Step 2c: determine new S-pair and take step: */

					/* case (i): no step in primal or dual space */
					if (step_length >= inf) {
						State_ = QuadProgState::NoStep;
					}

					/* case (ii): step in dual space */
					else if (f_length >= inf) {
						State_ = QuadProgState::StepInDualSpace;

					} else {
						/* case (iii): step in primal and dual space */
						State_ = QuadProgState::StepInPrimalAndDualSpace;
					}
				}
				break;

			case QuadProgState::NoStep: {
					State_ = QuadProgState::Infeasible;
				}
				break;

			case QuadProgState::StepInDualSpace: {
					/* set u = u +  t * [-r 1] and drop constraint l from the active set A */
					for (int k = 0; k < iq; k++) {
						u(k) -= step_length * r(k);
					}

					u(iq) += step_length;
					iai(l) = l;
					DeleteConstraint(A, u, iq, l);
					State_ = QuadProgState::DetermineStepDirection;
				}
				break;

			case QuadProgState::StepInPrimalAndDualSpace: {
					/* set x = x + t * z */
					x_ += step_length * z;
					/* update the solution value */
					f_value_ += step_length * z.dot(np) * (static_cast<Scalar>(0.5) * step_length + u(iq));

					/* u = u + t * [-r 1] */
					for (int k = 0; k < iq; k++) {
						u(k) -= step_length * r(k);
					}

					u(iq) += step_length;

					if (fabsf(step_length - f_length) < error_bound_) {
						/* full step has taken */
						/* add constraint ip to the active set*/
						if (!AddConstraint(d, iq)) {
							iaexcl(ip) = false;
							DeleteConstraint(A, u, iq, ip);

							for (int i = 0; i < IEQST; i++) {
								iai(i) = i;
							}

							for (int i = EQST; i < iq; i++) {
								A(i) = A_old(i);
								u(i) = u_old(i);
								iai(A(i)) = -1;
							}

							x_ = x_old;
							State_ = QuadProgState::CheckFeasibility;

						} else {
							iai(ip) = -1;
							State_ = QuadProgState::ChooseViolateConstrain;
						}

					} else {
						/* a patial step has taken */
						/* drop constraint l */
						iai(l) = l;
						DeleteConstraint(A, u, iq, l);

						/* update s[ip] = CI * x + ci0 */
						Vector<Scalar, DIMISIONS> ci_col = CI_.col(ip);
						s(ip) = ci_col.dot(x_) + ci0_(ip);
						State_ = QuadProgState::DetermineStepDirection;
					}
				}
				break;

			default:
				return false;
				break;
			}
		}

		if (State_ == QuadProgState::Infeasible) {
			return false;
		}

		return true;
	}
	void UpdateConstrains(const Matrix<Scalar, DIMISIONS, EQST> &CE, const Vector<Scalar, EQST> &ce0,
			      const Matrix<Scalar, DIMISIONS, IEQST> &CI, const Vector<Scalar, IEQST> &ci0)
	{
		CE_ = CE;
		ce0_ = ce0;
		CI_ = CI;
		ci0_ = ci0;

		InitKKTMatrix(G_.Inverse());
		equal_constrians_calculate_ = false;

		State_ = QuadProgState::UnconstrainMin;
	}
	void UpdateVectorConstrains(const Vector<Scalar, EQST> &ce0, const Vector<Scalar, IEQST> &ci0)
	{
		ce0_ = ce0;
		ci0_ = ci0;

		State_ = QuadProgState::UnconstrainMin;
	}
	Vector<Scalar, DIMISIONS> GetOptimalVector() const
	{
		return x_;
	}
	Scalar GetOptimalValue() const
	{
		return f_value_;
	}
	QuadProgState GetResultState() const
	{
		return State_;
	}
	inline int GetIterationCount() const
	{
		return iter_;
	}
	~QuadProg() = default;

private:
	void InitKKTMatrix(const Matrix<Scalar, DIMISIONS, DIMISIONS> G_inv)
	{
		kkt_.InitProblemMatrix(CE_, g0_, G_inv);
	}
	void ComputeEqualConstrainMin()
	{
		if (!equal_constrians_calculate_) {
			/* Add equality constraints to the working set A */
			int iq = 0;
			Vector<Scalar, DIMISIONS> d;
			Vector<Scalar, DIMISIONS> np;

			for (int i = 0; i < EQST; i++) {
				np = CE_.col(i);

				ComputeStepDirection(d, J_, np);

				if (!AddConstraint(d, iq)) {
					return;
				}
			}

			equal_constrain_J_ = J_;
			equal_constrain_R_ = R_;
			equal_constrain_R_norm = R_norm_;
			equal_constrians_calculate_ = true;

		} else {
			J_ = equal_constrain_J_;
			R_ = equal_constrain_R_;
			equal_constrain_R_norm = R_norm_;
		}

		kkt_.Solve(ce0_);

		x_ = kkt_.GetOptimalVector();
		Vector<Scalar, DIMISIONS> ltx = G_LT_ * x_;
		f_value_ = static_cast<Scalar>(0.5) * ltx.dot(ltx) + g0_.dot(x_);
	}
	inline void ComputeUnconstrainMin()
	{
		/*
		 * Find the unconstrained minimizer of the quadratic form 0.5 * x G x + g0 x
		 * this is a feasible point in the dual space
		 * x = G^-1 * g0
		 */
		G_.Solve(x_, g0_);
		x_ = -x_;
		f_value_ = static_cast<Scalar>(0.5) * g0_.dot(x_);
	}
	inline void InitCholeskyMatrix(const Matrix<Scalar, DIMISIONS, DIMISIONS> &G)
	{
		cond1_ = 0;

		for (int i = 0; i < DIMISIONS; ++i) {
			cond1_ += G(i, i);
		}

		G_.Decomposition(G);
		G_LT_ = G_.GetMatrixLT();

		Vector<Scalar, DIMISIONS> b;
		b.setZero();
		Vector<Scalar, DIMISIONS> y;

		constexpr int n = DIMISIONS;

		/* compute the inverse of the factorized matrix G^-1, this is the initial value for H */
		cond2_ = 0;

		for (int i = 0; i < n; i++) {
			b(i) = 1;
			G_.ForwardElimination(y, b);

			for (int j = 0; j < n; j++) {
				J_(i, j) = y(j);
			}

			cond2_ += y(i);
			b(i) = 0;
		}

		R_.setZero();
		R_norm_ = 1;
	}
	// compute d
	inline void ComputeStepDirection(Vector<Scalar, DIMISIONS> &d, const Matrix<Scalar, DIMISIONS, DIMISIONS> &J,
					 const Vector<Scalar, DIMISIONS> &np)
	{
		constexpr int n = DIMISIONS;

		// J.transpose()*np
		for (int i = 0; i < n; ++i) {
			d(i) = 0;

			for (int j = 0; j < n; ++j) {
				d(i) += J(j, i) * np(j);
			}
		}
	}

	// compute z
	inline void ComputeStepDirectionInPrimalSpace(Vector<Scalar, DIMISIONS> &z,
			const Matrix<Scalar, DIMISIONS, DIMISIONS> &J,
			const Vector<Scalar, DIMISIONS> &d, const int iq)
	{
		constexpr int n = DIMISIONS;

		for (int i = 0; i < n; ++i) {
			z(i) = 0;

			for (int j = iq; j < n; ++j) {
				z(i) += J(i, j) * d(j);
			}
		}
	}

	// compute r
	inline void ComputeStepDirectionInDualSpace(Vector<Scalar, DIMISIONS> &r, const Matrix<Scalar, DIMISIONS, DIMISIONS> &R,
			const Vector<Scalar, DIMISIONS> &d, const int iq)
	{
		/* setting of r = R^-1 d */
		for (int i = iq - 1; i >= 0; i--) {
			Scalar sum = 0;

			for (int j = i + 1; j < iq; j++) {
				sum += R(i, j) * r(j);
			}

			r(i) = (d(i) - sum) / R(i, i);
		}
	}

	bool AddConstraint(Vector<Scalar, DIMISIONS> &d, int &iq)
	{
		constexpr int n = DIMISIONS;

		/* we have to find the Givens rotation which will reduce the element
		  d[j] to zero.
		  if it is already zero we don't have to do anything, except of
		  decreasing j */
		for (int j = n - 1; j >= iq + 1; j--) {
			/* The Givens rotation is done with the matrix (cc cs, cs -cc).
			If cc is one, then element (j) of d is zero compared with element
			(j - 1). Hence we don't have to do anything.
			If cc is zero, then we just have to switch column (j) and column (j - 1)
			of J. Since we only switch columns in J, we have to be careful how we
			update d depending on the sign of gs.
			Otherwise we have to apply the Givens rotation to these columns.
			The i - 1 element of d has to be updated to h. */
			Scalar cc = d(j - 1);
			Scalar ss = d(j);
			Scalar h = Distance(cc, ss);

			if (fabsf(h) < error_bound_) { // h == 0
				continue;
			}

			d(j) = 0;
			ss = ss / h;
			cc = cc / h;

			if (cc < 0) {
				cc = -cc;
				ss = -ss;
				d(j - 1) = -h;

			} else {
				d(j - 1) = h;
			}

			Scalar xny = ss / (1 + cc);

			for (int k = 0; k < n; k++) {
				Scalar t1 = J_(k, j - 1);
				Scalar t2 = J_(k, j);
				J_(k, j - 1) = t1 * cc + t2 * ss;
				J_(k, j) = xny * (t1 + J_(k, j - 1)) - t2;
			}
		}

		/* update the number of constraints added*/
		iq++;

		/* To update R we have to put the iq components of the d vector
		  into column iq - 1 of R
		  */
		for (int i = 0; i < iq; i++) {
			R_(i, iq - 1) = d(i);
		}

		if (fabsf(d(iq - 1)) <= error_bound_ * R_norm_) {
			// problem degenerate
			return false;
		}

		R_norm_ = math::max(R_norm_, fabsf(d(iq - 1)));
		return true;
	}

	void DeleteConstraint(Vector < int, EQST + IEQST > &A, Vector < Scalar, EQST + IEQST > &u, int &iq, const int l)
	{
		constexpr int n = DIMISIONS;
		int qq = 0;

		bool found = false;

		/* Find the index qq for active constraint l to be removed */
		for (int i = EQST; i < iq; i++)
			if (A(i) == l) {
				qq = i;
				found = true;
				break;
			}

		if (!found) {
			return;
		}

		/* remove the constraint from the active set and the duals */
		for (int i = qq; i < iq - 1; i++) {
			A(i) = A(i + 1);
			u(i) = u(i + 1);

			for (int j = 0; j < n; j++) {
				R_(j, i) = R_(j, i + 1);
			}
		}

		A(iq - 1) = A(iq);
		u(iq - 1) = u(iq);
		A(iq) = 0;
		u(iq) = 0;

		for (int j = 0; j < iq; j++) {
			R_(j, iq - 1) = 0;
		}

		/* constraint has been fully removed */
		iq--;

		if (iq == 0) {
			return;
		}

		for (int j = qq; j < iq; j++) {
			Scalar cc = R_(j, j);
			Scalar ss = R_(j + 1, j);
			Scalar h = Distance(cc, ss);

			if (fabsf(h) < error_bound_) { // h == 0
				continue;
			}

			cc = cc / h;
			ss = ss / h;
			R_(j + 1, j) = 0;

			if (cc < 0) {
				R_(j, j) = -h;
				cc = -cc;
				ss = -ss;

			} else {
				R_(j, j) = h;
			}

			Scalar xny = ss / (1 + cc);

			for (int k = j + 1; k < iq; k++) {
				Scalar t1 = R_(j, k);
				Scalar t2 = R_(j + 1, k);
				R_(j, k) = t1 * cc + t2 * ss;
				R_(j + 1, k) = xny * (t1 + R_(j, k)) - t2;
			}

			for (int k = 0; k < n; k++) {
				Scalar t1 = J_(k, j);
				Scalar t2 = J_(k, j + 1);
				J_(k, j) = t1 * cc + t2 * ss;
				J_(k, j + 1) = xny * (J_(k, j) + t1) - t2;
			}
		}
	}
	inline Scalar Distance(const Scalar a, const Scalar b)
	{
		return sqrtf(a * a + b * b);
	}

private:
	Scalar error_bound_;
	int iter_;

	Vector<Scalar, DIMISIONS> x_;
	Scalar f_value_;

	// saved equal constrain matrix, used by continuing calculation
	bool equal_constrians_calculate_;
	Matrix<Scalar, DIMISIONS, DIMISIONS> equal_constrain_J_;
	Matrix<Scalar, DIMISIONS, DIMISIONS> equal_constrain_R_;
	Scalar equal_constrain_R_norm;

	KKTMethod<Scalar, DIMISIONS, EQST> kkt_;

	// problems matrix
	CholeskyDecomposition<Scalar, DIMISIONS> G_; // positive defined save in L*L^T decompsition
	Matrix<Scalar, DIMISIONS, DIMISIONS> G_LT_;
	Vector<Scalar, DIMISIONS> g0_;
	Matrix<Scalar, DIMISIONS, EQST> CE_;
	Vector<Scalar, EQST> ce0_;
	Matrix<Scalar, DIMISIONS, IEQST> CI_;
	Vector<Scalar, IEQST> ci0_;

	Scalar cond1_; // cond(G)
	Scalar cond2_; // cond(G)

	Matrix<Scalar, DIMISIONS, DIMISIONS> J_; // G choksky decompsition
	Matrix<Scalar, DIMISIONS, DIMISIONS> R_; // QR decompsition
	Scalar R_norm_;                          // this variable will hold the norm of the matrix R

	QuadProgState State_;
};
