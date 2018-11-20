#include "onedpath2_ip.h"

#include "draw.h"

#include <GL/glut.h>

#undef min
#undef max

#include <Eigen/Dense>
#include <algorithm>

using namespace Eigen;

enum V2
{
	// variables

	vel1X,
	duration0,
	duration1,

	// constraint multipliers (Lagrange multipliers)

	c0, // segment 0 initial acceleration
	c1, // segment 0 final acceleration
	c2, // segment 1 initial acceleration
	c3, // segment 1 final acceleration

	// constants

	pos0X,
	vel0X,
	pos1X,
	pos2X,
	vel2X,

	M2
};

static const size_t numVars = 3;

static const char * varName[numVars] =
{
	"v1",
	"t0",
	"t1",
};

struct Trajectory2
{
	double var[M2]; // variables and constants
};

static Trajectory2 g_trajectory;

const double accelerationLimit = 100.0;

typedef void (ConstraintFirstDerivFunc)(const Trajectory2 &, double & error, Matrix<double, numVars, 1> & deriv);
typedef void (ConstraintSecondDerivFunc)(const Trajectory2 &, Matrix<double, numVars, numVars> &);

static ConstraintFirstDerivFunc evalConstraint0;
static ConstraintFirstDerivFunc evalConstraint1;
static ConstraintFirstDerivFunc evalConstraint2;
static ConstraintFirstDerivFunc evalConstraint3;

static ConstraintSecondDerivFunc evalConstraintSecondDeriv0;
static ConstraintSecondDerivFunc evalConstraintSecondDeriv1;
static ConstraintSecondDerivFunc evalConstraintSecondDeriv2;
static ConstraintSecondDerivFunc evalConstraintSecondDeriv3;

static ConstraintFirstDerivFunc * constraints[] =
{
	evalConstraint0,
	evalConstraint1,
	evalConstraint2,
	evalConstraint3,
};

static ConstraintSecondDerivFunc * constraintSecondDerivs[] =
{
	evalConstraintSecondDeriv0,
	evalConstraintSecondDeriv1,
	evalConstraintSecondDeriv2,
	evalConstraintSecondDeriv3,
};

static const size_t numConstraints = sizeof(constraints) / sizeof(constraints[0]);

static void evalAccelInit(
	double x0,
	double v0,
	double x1,
	double v1,
	double t,
	double & accel,
	double & dAdT,
	double & dAdV0,
	double & dAdV1);
static void evalAccelSecondDerivInit(
	double x0,
	double v0,
	double x1,
	double v1,
	double t,
	double & sTT,
	double & sTV0,
	double & sTV1);
static void evalAccelFinal(
	double x0,
	double v0,
	double x1,
	double v1,
	double t,
	double & accel,
	double & dAdT,
	double & dAdV0,
	double & dAdV1);
static void evalAccelSecondDerivFinal(
	double x0,
	double v0,
	double x1,
	double v1,
	double t,
	double & sTT,
	double & sTV0,
	double & sTV1);
static void evalConstraints(
	const Trajectory2 &,
	Matrix<double, numConstraints, 1> & error,
	Matrix<double, numConstraints, numVars> & deriv);

static void moveTowardFeasibility(Trajectory2 &);
static void trajectoryStep(const Trajectory2 &, const Matrix<double, numVars + numConstraints, 1> & dir, double scale, Trajectory2 &);
static bool constraintsSatisfied(const Trajectory2 &);
static void residual(const Trajectory2 &, double perturbation, Matrix<double, numVars + numConstraints, 1> &);
static double residualNorm(const Trajectory2 &, double perturbation);
static double surrogateDualityGap(const Trajectory2 &);
static void moveInteriorPoint(Trajectory2 &);

static void printConstraints(const Trajectory2 &);
static void printState(const Trajectory2 &);
static void plotTrajectory(const Trajectory2 &);
static void plotAcceleration(const Trajectory2 &);

inline double sqr(double x)
{
	return x * x;
}

inline double cube(double x)
{
	return x * x * x;
}

OneDPath2InteriorPoint::OneDPath2InteriorPoint()
{
}

OneDPath2InteriorPoint::~OneDPath2InteriorPoint()
{
}

static void initDefault(Trajectory2 & traj)
{
	traj.var[pos0X] = 0;
	traj.var[vel0X] = 0;

	traj.var[pos1X] = 200;
	traj.var[vel1X] = 0;

	traj.var[pos2X] = 400;
	traj.var[vel2X] = 0;

	traj.var[duration0] = 3.5;
	traj.var[duration1] = 3.5;

#if 1
	traj.var[c0] = 1.0;
	traj.var[c1] = 1.0;
	traj.var[c2] = 1.0;
	traj.var[c3] = 1.0;
#else
	// Constraint multipliers start at 1/error

	for (size_t i = 0; i < numConstraints; ++i)
	{
		double error;
		Matrix<double, numVars, 1> grad;
		(*constraints[i])(traj, error, grad);
		traj.var[c0 + i] = -1.0 / error;
	}
#endif
}

void OneDPath2InteriorPoint::init()
{
	initDefault(g_trajectory);
}

void OneDPath2InteriorPoint::onActivate()
{
	printf(
		"\n1D path: Interior point with squared distance constraints\n\n"
		"Space      Move toward feasibility\n"
		"I          Reinitialize\n"
		"N          Take an interior-point step\n"
		"S          Print current state\n"
		"Home/End   Increment/Decrement segment 0 duration\n"
		"PgUp/PgDn  Increment/Decrement segment 1 duration\n"
		"Right/Left Increment/Decrement midpoint velocity\n"
		"Up/Down    Increment/Decrement midpoint position\n");
}

void OneDPath2InteriorPoint::onKey(unsigned char key)
{
	switch (key)
	{
	case ' ':
		moveTowardFeasibility(g_trajectory);
		repaint();
		break;

	case 'i':
		initDefault(g_trajectory);
		repaint();
		break;

	case 'n':
		moveInteriorPoint(g_trajectory);
		repaint();
		break;

	case 's':
		printState(g_trajectory);
		break;
	}
}

void OneDPath2InteriorPoint::onSpecialKey(int key)
{
	switch (key)
	{
	case GLUT_KEY_END:
		g_trajectory.var[duration0] -= 0.1;
		repaint();
		break;

	case GLUT_KEY_HOME:
		g_trajectory.var[duration0] += 0.1;
		repaint();
		break;

	case GLUT_KEY_PAGE_DOWN:
		g_trajectory.var[duration1] -= 0.1;
		repaint();
		break;

	case GLUT_KEY_PAGE_UP:
		g_trajectory.var[duration1] += 0.1;
		repaint();
		break;

	case GLUT_KEY_LEFT:
		g_trajectory.var[vel1X] -= 1;
		repaint();
		break;

	case GLUT_KEY_RIGHT:
		g_trajectory.var[vel1X] += 1;
		repaint();
		break;

	case GLUT_KEY_UP:
		g_trajectory.var[pos1X] += 10;
		repaint();
		break;

	case GLUT_KEY_DOWN:
		g_trajectory.var[pos1X] -= 10;
		repaint();
		break;
	}
}

static void unitSquare(double xMin, double xMax, double yMin, double yMax)
{
	const double sx = windowSizeX();
	const double sy = windowSizeY();

	glLoadIdentity();
	glTranslated(-1.0 + 2.0 * xMin / sx, -1.0 + 2.0 * yMin / sy, 0);
	glScaled(2.0 * (xMax - xMin) / sx, 2.0 * (yMax - yMin) / sy, 1);
}

void OneDPath2InteriorPoint::onDraw()
{
	const double margin = 10.0;
	const double sx = windowSizeX();
	const double sy = windowSizeY();

	glClear(GL_COLOR_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glMatrixMode(GL_MODELVIEW);

	// Plot trajectory

	unitSquare(margin, sx - margin, (sy + margin) / 2.0, sy - margin);
	plotTrajectory(g_trajectory);

	// Plot acceleration graph

	unitSquare(margin, sx - margin, margin, (sy - margin) / 2.0);
	plotAcceleration(g_trajectory);
}

void OneDPath2InteriorPoint::onMouseMove(int x, int y)
{
}

void OneDPath2InteriorPoint::onMouseDown()
{
}

void OneDPath2InteriorPoint::onMouseUp()
{
}

void evalAccelInit(
	double x0,
	double v0,
	double x1,
	double v1,
	double t,
	double & accel,
	double & dAdT,
	double & dAdV0,
	double & dAdV1)
{
	const double dX = x1 - x0;

	accel = (dX * 6.0 / t + v0 * -4.0 + v1 * -2.0) / t;

	// -12 * dX * t^-3 + 4 * v0 * t^-2 + 2 * t^-2

	dAdT = (dX * -12.0 / t + v0 * 4.0 + v1 * 2.0) / sqr(t);
	dAdV0 = -4.0 / t;
	dAdV1 = -2.0 / t;
}

void evalAccelSecondDerivInit(
	double x0,
	double v0,
	double x1,
	double v1,
	double t,
	double & sTT,
	double & sTV0,
	double & sTV1)
{
	const double dX = x1 - x0;

	// 36 * dX * t^-4 - 8 * v0 * t^-3 - 4 * v1 * t^-3

	sTT = (dX * 36.0 / t - v0 * 8.0 - v1 * 4.0) / cube(t);
	sTV0 = 4 / sqr(t);
	sTV1 = 2 / sqr(t);
}

void evalAccelFinal(
	double x0,
	double v0,
	double x1,
	double v1,
	double t,
	double & accel,
	double & dAdT,
	double & dAdV0,
	double & dAdV1)
{
	const double dX = x1 - x0;

	accel = (dX * -6.0 / t + v0 * 2.0 + v1 * 4.0) / t;

	// 12 * dX * t^-3 - 2 * v0 * t^-2 - 4 * v1 * t^-2

	dAdT = (dX * 12.0 / t + v0 * -2.0 + v1 * -4.0) / sqr(t);
	dAdV0 = 2.0 / t;
	dAdV1 = 4.0 / t;
}

void evalAccelSecondDerivFinal(
	double x0,
	double v0,
	double x1,
	double v1,
	double t,
	double & sTT,
	double & sTV0,
	double & sTV1)
{
	const double dX = x1 - x0;

	// -36 * dX * t^-4 + 4 * v0 * t^-3 + 8 * v1 * t^-3

	sTT = (dX * -36.0 / t + v0 * 4.0 + v1 * 8.0) / cube(t);
	sTV0 = -2.0 / sqr(t);
	sTV1 = -4.0 / sqr(t);
}

void evalConstraint0(const Trajectory2 & traj, double & error, Matrix<double, numVars, 1> & deriv)
{
	double a, dAdT, dAdV0, dAdV1;
	evalAccelInit(traj.var[pos0X], traj.var[vel0X], traj.var[pos1X], traj.var[vel1X], traj.var[duration0], a, dAdT, dAdV0, dAdV1);

	error = (sqr(a) - sqr(accelerationLimit)) / 2.0;

	deriv[duration0] = a * dAdT;
	deriv[duration1] = 0;
	deriv[vel1X] = a * dAdV1;
}

void evalConstraintSecondDeriv0(const Trajectory2 & traj, Matrix<double, numVars, numVars> & secondDeriv)
{
	/*

	1/2 * f(x, y, z)^2

	dx(x, y, z) = f(x, y, z) * dx(f(x, y, z))
	dy(x, y, z) = f(x, y, z) * dy(f(x, y, z))

	dx_dx(x, y, z) = dx(f(x, y, z))^2 + f(x, y, z) * dx_dx(f(x, y, z))
	dx_dy(x, y, z) = dy(f(x, y, z)) * dx(f(x, y, z)) + f(x, y, z) * dx_dy(f(x, y, z))

	*/

	double a, dAdT, dAdV0, dAdV1;
	evalAccelInit(traj.var[pos0X], traj.var[vel0X], traj.var[pos1X], traj.var[vel1X], traj.var[duration0], a, dAdT, dAdV0, dAdV1);

	double sTT, sTV0, sTV1;
	evalAccelSecondDerivInit(traj.var[pos0X], traj.var[vel0X], traj.var[pos1X], traj.var[vel1X], traj.var[duration0], sTT, sTV0, sTV1);

	secondDeriv.setZero();
	secondDeriv(duration0, duration0) = sqr(dAdT) + a * sTT;
	secondDeriv(duration0, vel1X) = secondDeriv(vel1X, duration0) = dAdT * dAdV1 + a * sTV1;
}

void evalConstraint1(const Trajectory2 & traj, double & error, Matrix<double, numVars, 1> & deriv)
{
	double a, dAdT, dAdV0, dAdV1;
	evalAccelFinal(traj.var[pos0X], traj.var[vel0X], traj.var[pos1X], traj.var[vel1X], traj.var[duration0], a, dAdT, dAdV0, dAdV1);

	error = (sqr(a) - sqr(accelerationLimit)) / 2.0;

	deriv[duration0] = a * dAdT;
	deriv[duration1] = 0;
	deriv[vel1X] = a * dAdV1;
}

void evalConstraintSecondDeriv1(const Trajectory2 & traj, Matrix<double, numVars, numVars> & secondDeriv)
{
	double a, dAdT, dAdV0, dAdV1;
	evalAccelFinal(traj.var[pos0X], traj.var[vel0X], traj.var[pos1X], traj.var[vel1X], traj.var[duration0], a, dAdT, dAdV0, dAdV1);

	double sTT, sTV0, sTV1;
	evalAccelSecondDerivFinal(traj.var[pos0X], traj.var[vel0X], traj.var[pos1X], traj.var[vel1X], traj.var[duration0], sTT, sTV0, sTV1);

	secondDeriv.setZero();
	secondDeriv(duration0, duration0) = sqr(dAdT) + a * sTT;
	secondDeriv(duration0, vel1X) = secondDeriv(vel1X, duration0) = dAdT * dAdV1 + a * sTV1;
}

void evalConstraint2(const Trajectory2 & traj, double & error, Matrix<double, numVars, 1> & deriv)
{
	double a, dAdT, dAdV0, dAdV1;
	evalAccelInit(traj.var[pos1X], traj.var[vel1X], traj.var[pos2X], traj.var[vel2X], traj.var[duration1], a, dAdT, dAdV0, dAdV1);

	error = (sqr(a) - sqr(accelerationLimit)) / 2.0;

	deriv[duration0] = 0;
	deriv[duration1] = a * dAdT;
	deriv[vel1X] = a * dAdV0;
}

void evalConstraintSecondDeriv2(const Trajectory2 & traj, Matrix<double, numVars, numVars> & secondDeriv)
{
	double a, dAdT, dAdV0, dAdV1;
	evalAccelInit(traj.var[pos1X], traj.var[vel1X], traj.var[pos2X], traj.var[vel2X], traj.var[duration1], a, dAdT, dAdV0, dAdV1);

	double sTT, sTV0, sTV1;
	evalAccelSecondDerivInit(traj.var[pos1X], traj.var[vel1X], traj.var[pos2X], traj.var[vel2X], traj.var[duration1], sTT, sTV0, sTV1);

	secondDeriv.setZero();
	secondDeriv(duration1, duration1) = sqr(dAdT) + a * sTT;
	secondDeriv(duration1, vel1X) = secondDeriv(vel1X, duration1) = dAdT * dAdV0 + a * sTV0;
}

void evalConstraint3(const Trajectory2 & traj, double & error, Matrix<double, numVars, 1> & deriv)
{
	double a, dAdT, dAdV0, dAdV1;
	evalAccelFinal(traj.var[pos1X], traj.var[vel1X], traj.var[pos2X], traj.var[vel2X], traj.var[duration1], a, dAdT, dAdV0, dAdV1);

	error = (sqr(a) - sqr(accelerationLimit)) / 2.0;

	deriv[duration0] = 0;
	deriv[duration1] = a * dAdT;
	deriv[vel1X] = a * dAdV0;
}

void evalConstraintSecondDeriv3(const Trajectory2 & traj, Matrix<double, numVars, numVars> & secondDeriv)
{
	double a, dAdT, dAdV0, dAdV1;
	evalAccelFinal(traj.var[pos1X], traj.var[vel1X], traj.var[pos2X], traj.var[vel2X], traj.var[duration1], a, dAdT, dAdV0, dAdV1);

	double sTT, sTV0, sTV1;
	evalAccelSecondDerivFinal(traj.var[pos1X], traj.var[vel1X], traj.var[pos2X], traj.var[vel2X], traj.var[duration1], sTT, sTV0, sTV1);

	secondDeriv.setZero();
	secondDeriv(duration1, duration1) = sqr(dAdT) + a * sTT;
	secondDeriv(duration1, vel1X) = secondDeriv(vel1X, duration1) = dAdT * dAdV0 + a * sTV0;
}

void evalConstraints(const Trajectory2 & traj, Matrix<double, numConstraints, 1> & error, Matrix<double, numConstraints, numVars> & deriv)
{
	for (size_t i = 0; i < numConstraints; ++i)
	{
		Matrix<double, numVars, 1> grad;
		(*constraints[i])(traj, error[i], grad);
		deriv.row(i) = grad;
	}
}

void moveTowardFeasibility(Trajectory2 & traj)
{
	// Collect violated constraints

	Matrix<double, numConstraints, 1> constraintError;
	Matrix<double, numConstraints, numVars> constraintGradient;
	evalConstraints(traj, constraintError, constraintGradient);

	size_t n = 0;
	size_t constraintIndex[numConstraints];
	for (size_t i = 0; i < numConstraints; ++i)
	{
		if (constraintError[i] > 0)
		{
			constraintIndex[n] = i;
			++n;
		}
	}

	// Get the errors and gradients of the violated constraints

	double cm[numConstraints];
	memset(cm, 0, sizeof(cm));

	VectorXd dX(numVars);
	dX.setZero();

	if (n > 0)
	{
		MatrixXd g(n, numVars);
		VectorXd err(n);
		for (size_t j = 0; j < n; ++j)
		{
			size_t i = constraintIndex[j];
			err(j) = constraintError[i];
			for (size_t k = 0; k < numVars; ++k)
			{
				g(j, k) = constraintGradient(i, k);
			}
		}

		// Compute multipliers for the gradients of the violated constraints that will add up to remove the error

		MatrixXd a = g * g.transpose();

		VectorXd m = a.colPivHouseholderQr().solve(err);
		assert(m.size() == n);

		dX = g.transpose() * -m;

		for (size_t j = 0; j < n; ++j)
			cm[constraintIndex[j]] = m[j];
	}

	printf("\nConstraints:\n");

	for (size_t i = 0; i < numConstraints; ++i)
	{
		printf("%2u:", i);
		for (size_t j = 0; j < numVars; ++j)
		{
			printf(" %g", constraintGradient(i, j));
		}
		printf(" | %g x %g\n", constraintError[i], cm[i]);
	}

	printf("   ");
	for (size_t i = 0; i < numVars; ++i)
		printf(" %g", dX[i]);
	printf("\n");

	for (size_t i = 0; i < numVars; ++i)
		traj.var[i] += dX[i];
}

void trajectoryStep(const Trajectory2 & trajOriginal, const Matrix<double, numVars + numConstraints, 1> & dir, double scale, Trajectory2 & trajNew)
{
	const size_t c = numVars + numConstraints;

	for (size_t i = 0; i < c; ++i)
	{
		trajNew.var[i] = trajOriginal.var[i] + dir(i) * scale;
	}

	for (size_t i = c; i < M2; ++i)
	{
		trajNew.var[i] = trajOriginal.var[i];
	}
}

bool constraintsSatisfied(const Trajectory2 & traj)
{
	for (size_t i = 0; i < numConstraints; ++i)
	{
		double error;
		Matrix<double, numVars, 1> grad;
		(*constraints[i])(traj, error, grad);

		if (error > 0.0)
			return false;
	}

	return true;
}

void residual(const Trajectory2 & traj, double perturbation, Matrix<double, numVars + numConstraints, 1> & r)
{
	const size_t c = numVars + numConstraints;

	r.setZero();

	// Plug the objective gradient into the residual

	r(duration0) = 1;
	r(duration1) = 1;

	for (size_t i = 0; i < numConstraints; ++i)
	{
		// Current Lagrange multiplier for this constraint

		const double scale = traj.var[c0 + i];

		// Evaluate constraint i

		double error;
		Matrix<double, numVars, 1> grad;
		(*constraints[i])(traj, error, grad);

		for (size_t j = 0; j < numVars; ++j)
		{
			r(j) += scale * grad(j);
		}

		r(numVars + i) = scale * error + perturbation;
	}
}

double residualNorm(const Trajectory2 & traj, double perturbation)
{
	Matrix<double, numVars + numConstraints, 1> r;

	residual(traj, perturbation, r);

	return r.squaredNorm();
}

double surrogateDualityGap(const Trajectory2 & traj)
{
	double mu = 0;

	for (size_t i = 0; i < numConstraints; ++i)
	{
		double error;
		Matrix<double, numVars, 1> grad;
		(*constraints[i])(traj, error, grad);

		mu -= error * traj.var[c0 + i];
	}

	return mu;
}

void moveInteriorPoint(Trajectory2 & traj)
{
	double perturbation = 0.01; // surrogateDualityGap(traj) / (numConstraints * 10.0);

	const size_t c = numVars + numConstraints;

	Matrix<double, c, c> m; // rate of change in residual component as each variable is varied
	m.setZero();

	Matrix<double, c, 1> r; // residual
	r.setZero();

	// Plug the objective gradient into the residual

	r(duration0) = 1;
	r(duration1) = 1;

	for (size_t i = 0; i < numConstraints; ++i)
	{
		// Current Lagrange multiplier for this constraint

		const double scale = traj.var[c0 + i];

		// Evaluate constraint i

		double error;
		Matrix<double, numVars, 1> grad;
		Matrix<double, numVars, numVars> secondDeriv;
		(*constraints[i])(traj, error, grad);
		(*constraintSecondDerivs[i])(traj, secondDeriv);

		// Plug the constraint into the matrix and residual vector

		for (size_t row = 0; row < numVars; ++row)
		{
			for (size_t col = 0; col < numVars; ++col)
			{
				m(row, col) += secondDeriv(row, col) * scale;
			}
		}

		for (size_t j = 0; j < numVars; ++j)
		{
			m(j, numVars + i) = grad(j);
			m(numVars + i, j) = scale * grad(j);

			r(j) += scale * grad(j);
		}

		m(numVars + i, numVars + i) = error;
		r(numVars + i) = scale * error + perturbation;
	}

	// Print the final result

	printf("\nFull matrix:\n");
	for (size_t row = 0; row < c; ++row)
	{
		Matrix<double, 1, c> x = m.row(row);
		double s = 1.0 / x.norm();
		x *= s;
		double rhs = r(row) * s;

		for (size_t col = 0; col < c; ++col)
		{
			int y = std::min(9, int(ceil(fabs(x(col)) * 10.0)));
			printf("%c ", (y == 0) ? ' ' : (y + '0'));
		}

		printf("| ");

		printf("%g\n", rhs);
	}

	// Solve the system for the step to try to eliminate the residual

	Matrix<double, c, 1> d;
	d = m.colPivHouseholderQr().solve(-r);

	printf("Current state:\n");
	for (size_t i = 0; i < c; ++i)
	{
		printf("%s%g", (i > 0) ? " " : "", traj.var[i]);
	}
	printf("\nStep for perturbation %g:\n", perturbation);
	for (size_t i = 0; i < c; ++i)
	{
		printf("%s%g", (i > 0) ? " " : "", d(i));
	}
	printf("\n");

	// Backtrack to a point where none of the Lagrange multipliers are hitting zero

	double s = 1.0;

	for (size_t i = 0; i < numConstraints; ++i)
	{
		double dLm = d(numVars + i);
		if (dLm < 0.0)
		{
			double lmOld = traj.var[numVars + i];
			s = std::min(s, -lmOld / dLm);
		}
	}

	s *= 0.99;

	// Backtrack while the step is violating a constraint

	for (size_t i = 0; i < 100; ++i)
	{
		Trajectory2 trajNew;
		trajectoryStep(traj, d, s, trajNew);

		if (constraintsSatisfied(trajNew))
			break;

		s *= 0.5;
	}

	// Backtrack while the magnitude of the residual is not decreasing

	double residualNormOriginal = residualNorm(traj, perturbation);

	for (size_t i = 0; i < 100; ++i)
	{
		Trajectory2 trajNew;
		trajectoryStep(traj, d, s, trajNew);

		double residualNormNew = residualNorm(trajNew, perturbation);

		if (residualNormNew <= residualNormOriginal * (1.0 - 0.01 * s))
			break;

		s *= 0.5;
	}

	// Take the step

	d *= s;

	for (size_t i = 0; i < c; ++i)
		traj.var[i] += d(i);
}

void printConstraints(const Trajectory2 & traj)
{
	Matrix<double, numVars, 1> obj;
	obj << 0, -1, -1;

	printf("[");
	for (size_t i = 0; i < numVars; ++i)
		printf("%s%s", (i > 0) ? " " : "", varName[i]);

	printf("] [");
	for (size_t i = 0; i < numVars; ++i)
	{
		printf("%s[", (i > 0) ? " " : "");

		for (size_t j = 0; j < numVars; ++j)
		{
			printf("%s%s/%s", (j > 0) ? " " : "", varName[i], varName[j]);
		}

		printf("]");
	}
	printf("]\n");

	for (size_t i = 0; i < numConstraints; ++i)
	{
		double error;
		Matrix<double, numVars, 1> deriv;
		Matrix<double, numVars, numVars> seconds;
		(*constraints[i])(traj, error, deriv);
		(*constraintSecondDerivs[i])(traj, seconds);

		double d = obj.dot(deriv);

		printf("%c%u:", (error > 0) ? '*' : ' ', i);

		printf(" error=%g derivs=[", error);

		for (size_t j = 0; j < numVars; ++j)
		{
			printf("%s%g", (j > 0) ? " " : "", deriv(j));
		}

		printf("] second=[");
		
		for (size_t j = 0; j < numVars; ++j)
		{
			printf("%s[", (j > 0) ? " " : "");

			for (size_t k = 0; k < numVars; ++k)
			{
				printf("%s%g", (k > 0) ? " " : "", seconds(j, k));
			}

			printf("]");
		}

		printf("] dot=%g\n", d);
	}
}

void printState(const Trajectory2 & traj)
{
	printf("\nNode 0: pos=%g vel=%g\n", traj.var[pos0X], traj.var[vel0X]);
	printf("Node 1: pos=%g vel=%g\n", traj.var[pos1X], traj.var[vel1X]);
	printf("Node 2: pos=%g vel=%g\n", traj.var[pos2X], traj.var[vel2X]);
	printf("Duration 0: %g\n", traj.var[duration0]);
	printf("Duration 1: %g\n", traj.var[duration1]);
	printf("Constraint Multipliers:");
	for (size_t i = 0; i < numConstraints; ++i)
	{
		printf(" %g", traj.var[c0 + i]);
	}
	printf("\n");
	printf("Surrogate gap: %g\n", surrogateDualityGap(traj));
	printf("Constraints:\n");
	printConstraints(traj);
}

void plotAcceleration(const Trajectory2 & traj)
{
	double s = 1.0 / (2.0 * accelerationLimit);

	double tTotal = traj.var[duration0] + traj.var[duration1];

	double u0 = 0;
	double u1 = traj.var[duration0] / tTotal;
	double u2 = 1;

	double a0 = ((traj.var[pos1X] - traj.var[pos0X]) *  6.0 / traj.var[duration0] + traj.var[vel0X] * -4.0 + traj.var[vel1X] * -2.0) / traj.var[duration0];
	double a1 = ((traj.var[pos1X] - traj.var[pos0X]) * -6.0 / traj.var[duration0] + traj.var[vel0X] *  2.0 + traj.var[vel1X] *  4.0) / traj.var[duration0];
	double a2 = ((traj.var[pos2X] - traj.var[pos1X]) *  6.0 / traj.var[duration1] + traj.var[vel1X] * -4.0 + traj.var[vel2X] * -2.0) / traj.var[duration1];
	double a3 = ((traj.var[pos2X] - traj.var[pos1X]) * -6.0 / traj.var[duration1] + traj.var[vel1X] *  2.0 + traj.var[vel2X] *  4.0) / traj.var[duration1];

	glColor3d(0.1, 0.1, 0.1);
	glBegin(GL_QUADS);
	glVertex2d(0, 0);
	glVertex2d(1, 0);
	glVertex2d(1, 1);
	glVertex2d(0, 1);
	glEnd();

	glColor3d(0.25, 0.25, 0.25);
	glBegin(GL_LINE_LOOP);
	glVertex2d(0, 0);
	glVertex2d(1, 0);
	glVertex2d(1, 1);
	glVertex2d(0, 1);
	glEnd();

	glBegin(GL_LINES);

	glVertex2d(0, 0.5);
	glVertex2d(1, 0.5);

	glVertex2d(u1, 0);
	glVertex2d(u1, 1);

	glColor3d(1, 1, 0);
	glVertex2d(u0, 0.5 + s * a0);
	glVertex2d(u1, 0.5 + s * a1);

	glColor3d(0, 1, 1);
	glVertex2d(u1, 0.5 + s * a2);
	glVertex2d(u2, 0.5 + s * a3);

	glEnd();
}

static void drawSegment(double x0, double v0, double x1, double v1, double h, double r, double g, double b)
{
	glColor3d(r, g, b);

	double acc0 = (x1 - x0) * (6.0 / sqr(h)) - (v0 * 4.0 + v1 * 2.0) / h;
	double jrk0 = (v1 - v0) * (2.0 / sqr(h)) - acc0 * (2.0 / h);

	glBegin(GL_LINE_STRIP);

	glVertex2d(0, x0);

	for (size_t j = 1; j < 32; ++j)
	{
		double t = h * double(j) / 32.0;

		double pos = x0 + (v0 + (acc0 + jrk0 * (t / 3.0f)) * (t / 2.0f)) * t;

		glVertex2d(t, pos);
	}

	glVertex2d(h, x1);

	glEnd();
}

void plotTrajectory(const Trajectory2 & traj)
{
	glColor3d(0.1, 0.1, 0.1);
	glBegin(GL_QUADS);
	glVertex2d(0, 0);
	glVertex2d(1, 0);
	glVertex2d(1, 1);
	glVertex2d(0, 1);
	glEnd();

	glColor3d(0.25, 0.25, 0.25);
	glBegin(GL_LINE_LOOP);
	glVertex2d(0, 0);
	glVertex2d(1, 0);
	glVertex2d(1, 1);
	glVertex2d(0, 1);
	glEnd();

	double tTotal = traj.var[duration0] + traj.var[duration1];
	double u1 = traj.var[duration0] / tTotal;

	glBegin(GL_LINES);
	glVertex2d(u1, 0);
	glVertex2d(u1, 1);

	glVertex2d(0, traj.var[pos1X] / 400.0);
	glVertex2d(1, traj.var[pos1X] / 400.0);
	glEnd();

	// Draw the curve

	glPushMatrix();
	glScaled(1.0 / tTotal, 1.0 / 400.0, 1.0);

	drawSegment(
		traj.var[pos0X],
		traj.var[vel0X],
		traj.var[pos1X],
		traj.var[vel1X],
		traj.var[duration0],
		1, 1, 0
	);

	glPushMatrix();
	glTranslated(traj.var[duration0], 0, 0);

	drawSegment(
		traj.var[pos1X],
		traj.var[vel1X],
		traj.var[pos2X],
		traj.var[vel2X],
		traj.var[duration1],
		0, 1, 1
	);

	glPopMatrix();

	glPopMatrix();
}
