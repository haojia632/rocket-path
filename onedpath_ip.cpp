#include "onedpath_ip.h"

#include "draw.h"

#include <windows.h>
#include <gl/gl.h>
#include <gl/glu.h>

#undef min
#undef max

#include <Eigen/Dense>
#include <algorithm>

using namespace Eigen;
using std::max;

enum V
{
	// variables

	vel1X,
	duration0,
	duration1,

	// constraint multipliers (Lagrange multipliers)

	c0, // segment 0 initial acceleration minimum
	c1, // segment 0 initial acceleration maximum
	c2, // segment 0 final acceleration minimum
	c3, // segment 0 final acceleration maximum
	c4, // segment 1 initial acceleration minimum
	c5, // segment 1 initial acceleration maximum
	c6, // segment 1 final acceleration minimum
	c7, // segment 1 final acceleration maximum

	// constants

	pos0X,
	vel0X,
	pos1X,
	pos2X,
	vel2X,

	M
};

static const size_t numVars = 3;

struct Trajectory
{
	double var[M]; // variables and constants
};

static Trajectory g_trajectory;
static double g_slack = 0.5;

const double accelerationLimit = 100.0;

typedef void (ConstraintFunc) (const Trajectory &, double & error, Matrix<double, numVars, 1> & deriv, Matrix<double, numVars, numVars> & secondDeriv);

static ConstraintFunc evalConstraint0;
static ConstraintFunc evalConstraint1;
static ConstraintFunc evalConstraint2;
static ConstraintFunc evalConstraint3;
static ConstraintFunc evalConstraint4;
static ConstraintFunc evalConstraint5;
static ConstraintFunc evalConstraint6;
static ConstraintFunc evalConstraint7;

static ConstraintFunc * constraints[] =
{
	evalConstraint0,
	evalConstraint1,
	evalConstraint2,
	evalConstraint3,
	evalConstraint4,
	evalConstraint5,
	evalConstraint6,
	evalConstraint7,
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
	double & dAdV1,
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
	double & dAdV1,
	double & sTT,
	double & sTV0,
	double & sTV1);
static void evalConstraints(
	const Trajectory &,
	Matrix<double, numConstraints, 1> & error,
	Matrix<double, numConstraints, numVars> & deriv);

static void computeLagrangeMultipliers(Trajectory &);
static void moveTowardFeasibility(Trajectory &);
static void moveInConstrainedGradientDir(Trajectory &);
static void moveInteriorPoint(Trajectory &, double slack);
static void fixupConstraint(Trajectory &, ConstraintFunc * constraint);

static void printConstraints(const Matrix<double, numConstraints, 1> & error, const Matrix<double, numConstraints, numVars> & deriv);
static void printState(const Trajectory &);
static void plotTrajectory(const Trajectory &);
static void plotAcceleration(const Trajectory &);

inline double sqr(double x)
{
	return x * x;
}

inline double cube(double x)
{
	return x * x * x;
}

OneDPathInteriorPoint::OneDPathInteriorPoint()
{
}

OneDPathInteriorPoint::~OneDPathInteriorPoint()
{
}

void OneDPathInteriorPoint::init()
{
	g_slack = 0.5;

	memset(&g_trajectory, 0, sizeof(g_trajectory));

	g_trajectory.var[pos0X] = 0;
	g_trajectory.var[vel0X] = 0;

	g_trajectory.var[pos1X] = 200;
	g_trajectory.var[vel1X] = 0;

	g_trajectory.var[pos2X] = 400;
	g_trajectory.var[vel2X] = 0;

	g_trajectory.var[duration0] = 3.4641;
	g_trajectory.var[duration1] = 3.4641;

	// Start with all the constraints having multipliers of 1?

	for (size_t i = 0; i < numConstraints; ++i)
	{
		g_trajectory.var[c0 + i] = 1;
	}
}

void OneDPathInteriorPoint::onKey(unsigned int key)
{
	switch (key)
	{
	case VK_SPACE:
		moveTowardFeasibility(g_trajectory);
		repaint();
		break;

	case VK_END:
		g_trajectory.var[duration0] -= 0.1;
		repaint();
		break;

	case VK_HOME:
		g_trajectory.var[duration0] += 0.1;
		repaint();
		break;

	case VK_NEXT:
		g_trajectory.var[duration1] -= 0.1;
		repaint();
		break;

	case VK_PRIOR:
		g_trajectory.var[duration1] += 0.1;
		repaint();
		break;

	case VK_LEFT:
		g_trajectory.var[vel1X] -= 1;
		repaint();
		break;

	case VK_RIGHT:
		g_trajectory.var[vel1X] += 1;
		repaint();
		break;

	case VK_UP:
		g_trajectory.var[pos1X] += 10;
		repaint();
		break;

	case VK_DOWN:
		g_trajectory.var[pos1X] -= 10;
		repaint();
		break;

	case 'I':
		init();
		repaint();
		break;

	case 'M':
		{
			computeLagrangeMultipliers(g_trajectory);
			repaint();
		}
		break;

	case 'N':
		moveInteriorPoint(g_trajectory, g_slack);
		repaint();
		break;

	case VK_OEM_MINUS:
		g_slack /= 10.0;
		debug_printf("Slack is now %g\n", g_slack);
		break;

	case VK_OEM_PLUS:
		g_slack *= 10.0;
		debug_printf("Slack is now %g\n", g_slack);
		break;

	case 'S':
		printState(g_trajectory);
		break;

	case 'Z':
		moveInConstrainedGradientDir(g_trajectory);
		repaint();
		break;

	case '1':
		fixupConstraint(g_trajectory, constraints[0]);
		fixupConstraint(g_trajectory, constraints[1]);
		repaint();
		break;

	case '2':
		fixupConstraint(g_trajectory, constraints[2]);
		fixupConstraint(g_trajectory, constraints[3]);
		repaint();
		break;

	case '3':
		fixupConstraint(g_trajectory, constraints[4]);
		fixupConstraint(g_trajectory, constraints[5]);
		repaint();
		break;

	case '4':
		fixupConstraint(g_trajectory, constraints[6]);
		fixupConstraint(g_trajectory, constraints[7]);
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

void OneDPathInteriorPoint::onDraw()
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

void OneDPathInteriorPoint::onMouseMove(int x, int y)
{
}

void OneDPathInteriorPoint::onMouseDown()
{
}

void OneDPathInteriorPoint::onMouseUp()
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
	double & dAdV1,
	double & sTT,
	double & sTV0,
	double & sTV1)
{
	const double dX = x1 - x0;

	accel = (dX * 6.0 / t + v0 * -4.0 + v1 * -2.0) / t;

	// -12 * dX * t^-3 + 4 * v0 * t^-2 + 2 * t^-2

	dAdT = (dX * -12.0 / t + v0 * 4.0 + v1 * 2.0) / sqr(t);
	dAdV0 = -4.0 / t;
	dAdV1 = -2.0 / t;

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
	double & dAdV1,
	double & sTT,
	double & sTV0,
	double & sTV1)
{
	const double dX = x1 - x0;

	accel = (dX * -6.0 / t + v0 * 2.0 + v1 * 4.0) / t;

	// 12 * dX * t^-3 - 2 * v0 * t^-2 - 4 * v1 * t^-2

	dAdT = (dX * 12.0 / t + v0 * -2.0 + v1 * -4.0) / sqr(t);
	dAdV0 = 2.0 / t;
	dAdV1 = 4.0 / t;

	// -36 * dX * t^-4 + 4 * v0 * t^-3 + 8 * v1 * t^-3

	sTT = (dX * -36.0 / t + v0 * 4.0 + v1 * 8.0) / cube(t);
	sTV0 = -2.0 / sqr(t);
	sTV1 = -4.0 / sqr(t);
}

void evalConstraint0(const Trajectory & traj, double & error, Matrix<double, numVars, 1> & deriv, Matrix<double, numVars, numVars> & secondDeriv)
{
	double a, dAdT, dAdV0, dAdV1;
	double sTT, sTV0, sTV1;
	evalAccelInit(traj.var[pos0X], traj.var[vel0X], traj.var[pos1X], traj.var[vel1X], traj.var[duration0], a, dAdT, dAdV0, dAdV1, sTT, sTV0, sTV1);

	error = -a - accelerationLimit;

	deriv[duration0] = -dAdT;
	deriv[duration1] = 0;
	deriv[vel1X] = -dAdV1;

	secondDeriv.setZero();
	secondDeriv(duration0, duration0) = -sTT;
	secondDeriv(duration0, vel1X) = -sTV1;
	secondDeriv(vel1X, duration0) = -sTV1;
}

void evalConstraint1(const Trajectory & traj, double & error, Matrix<double, numVars, 1> & deriv, Matrix<double, numVars, numVars> & secondDeriv)
{
	double a, dAdT, dAdV0, dAdV1;
	double sTT, sTV0, sTV1;
	evalAccelInit(traj.var[pos0X], traj.var[vel0X], traj.var[pos1X], traj.var[vel1X], traj.var[duration0], a, dAdT, dAdV0, dAdV1, sTT, sTV0, sTV1);

	error = a - accelerationLimit;

	deriv[duration0] = dAdT;
	deriv[duration1] = 0;
	deriv[vel1X] = dAdV1;

	secondDeriv.setZero();
	secondDeriv(duration0, duration0) = sTT;
	secondDeriv(duration0, vel1X) = sTV1;
	secondDeriv(vel1X, duration0) = sTV1;
}

void evalConstraint2(const Trajectory & traj, double & error, Matrix<double, numVars, 1> & deriv, Matrix<double, numVars, numVars> & secondDeriv)
{
	double a, dAdT, dAdV0, dAdV1;
	double sTT, sTV0, sTV1;
	evalAccelFinal(traj.var[pos0X], traj.var[vel0X], traj.var[pos1X], traj.var[vel1X], traj.var[duration0], a, dAdT, dAdV0, dAdV1, sTT, sTV0, sTV1);

	error = -a - accelerationLimit;

	deriv[duration0] = -dAdT;
	deriv[duration1] = 0;
	deriv[vel1X] = -dAdV1;

	secondDeriv.setZero();
	secondDeriv(duration0, duration0) = -sTT;
	secondDeriv(duration0, vel1X) = -sTV1;
	secondDeriv(vel1X, duration0) = -sTV1;
}

void evalConstraint3(const Trajectory & traj, double & error, Matrix<double, numVars, 1> & deriv, Matrix<double, numVars, numVars> & secondDeriv)
{
	double a, dAdT, dAdV0, dAdV1;
	double sTT, sTV0, sTV1;
	evalAccelFinal(traj.var[pos0X], traj.var[vel0X], traj.var[pos1X], traj.var[vel1X], traj.var[duration0], a, dAdT, dAdV0, dAdV1, sTT, sTV0, sTV1);

	error = a - accelerationLimit;

	deriv[duration0] = dAdT;
	deriv[duration1] = 0;
	deriv[vel1X] = dAdV1;

	secondDeriv.setZero();
	secondDeriv(duration0, duration0) = sTT;
	secondDeriv(duration0, vel1X) = sTV1;
	secondDeriv(vel1X, duration0) = sTV1;
}

void evalConstraint4(const Trajectory & traj, double & error, Matrix<double, numVars, 1> & deriv, Matrix<double, numVars, numVars> & secondDeriv)
{
	double a, dAdT, dAdV0, dAdV1;
	double sTT, sTV0, sTV1;
	evalAccelInit(traj.var[pos1X], traj.var[vel1X], traj.var[pos2X], traj.var[vel2X], traj.var[duration1], a, dAdT, dAdV0, dAdV1, sTT, sTV0, sTV1);

	error = -a - accelerationLimit;

	deriv[duration0] = 0;
	deriv[duration1] = -dAdT;
	deriv[vel1X] = -dAdV0;

	secondDeriv.setZero();
	secondDeriv(duration1, duration1) = -sTT;
	secondDeriv(duration1, vel1X) = -sTV0;
	secondDeriv(vel1X, duration1) = -sTV0;
}

void evalConstraint5(const Trajectory & traj, double & error, Matrix<double, numVars, 1> & deriv, Matrix<double, numVars, numVars> & secondDeriv)
{
	double a, dAdT, dAdV0, dAdV1;
	double sTT, sTV0, sTV1;
	evalAccelInit(traj.var[pos1X], traj.var[vel1X], traj.var[pos2X], traj.var[vel2X], traj.var[duration1], a, dAdT, dAdV0, dAdV1, sTT, sTV0, sTV1);

	error = a - accelerationLimit;

	deriv[duration0] = 0;
	deriv[duration1] = dAdT;
	deriv[vel1X] = dAdV0;

	secondDeriv.setZero();
	secondDeriv(duration1, duration1) = sTT;
	secondDeriv(duration1, vel1X) = sTV0;
	secondDeriv(vel1X, duration1) = sTV0;
}

void evalConstraint6(const Trajectory & traj, double & error, Matrix<double, numVars, 1> & deriv, Matrix<double, numVars, numVars> & secondDeriv)
{
	double a, dAdT, dAdV0, dAdV1;
	double sTT, sTV0, sTV1;
	evalAccelFinal(traj.var[pos1X], traj.var[vel1X], traj.var[pos2X], traj.var[vel2X], traj.var[duration1], a, dAdT, dAdV0, dAdV1, sTT, sTV0, sTV1);

	error = -a - accelerationLimit;

	deriv[duration0] = 0;
	deriv[duration1] = -dAdT;
	deriv[vel1X] = -dAdV0;

	secondDeriv.setZero();
	secondDeriv(duration1, duration1) = -sTT;
	secondDeriv(duration1, vel1X) = -sTV0;
	secondDeriv(vel1X, duration1) = -sTV0;
}

void evalConstraint7(const Trajectory & traj, double & error, Matrix<double, numVars, 1> & deriv, Matrix<double, numVars, numVars> & secondDeriv)
{
	double a, dAdT, dAdV0, dAdV1;
	double sTT, sTV0, sTV1;
	evalAccelFinal(traj.var[pos1X], traj.var[vel1X], traj.var[pos2X], traj.var[vel2X], traj.var[duration1], a, dAdT, dAdV0, dAdV1, sTT, sTV0, sTV1);

	error = a - accelerationLimit;

	deriv[duration0] = 0;
	deriv[duration1] = dAdT;
	deriv[vel1X] = dAdV0;

	secondDeriv.setZero();
	secondDeriv(duration1, duration1) = sTT;
	secondDeriv(duration1, vel1X) = sTV0;
	secondDeriv(vel1X, duration1) = sTV0;
}

void evalConstraints(const Trajectory & traj, Matrix<double, numConstraints, 1> & error, Matrix<double, numConstraints, numVars> & deriv)
{
	for (size_t i = 0; i < numConstraints; ++i)
	{
		Matrix<double, numVars, 1> grad;
		Matrix<double, numVars, numVars> secondDeriv;
		(*constraints[i])(traj, error[i], grad, secondDeriv);
		deriv.row(i) = grad;
	}
}

void computeLagrangeMultipliers(Trajectory & traj)
{
	Matrix<double, numConstraints, 1> y;
	y.setZero();

	Matrix<double, numVars, 1> obj;
	obj << -0.707107, -0.707107, 0;

	// Get the constraint gradients at the current state position

	Matrix<double, numConstraints, 1> error;
	Matrix<double, numConstraints, numVars> deriv;
	evalConstraints(traj, error, deriv);

	// Iteratively come up with multipliers for the constraint gradients that get rid of
	//  objective motion in the direction of the constraint gradients, for constraints
	//  that have positive error.

	debug_printf("\n");

	Matrix<double, numVars, 1> sum;

	for (size_t iter = 0; iter < 8; ++iter)
	{
		for (size_t i = 0; i < numConstraints; ++i)
		{
			// Ignore inactive constraints

			if (error[i] < -1.0e-4)
				continue;

			// Compute the current constrained objective direction

			Matrix<double, numVars, 1> dir = obj - deriv.transpose() * y;

			// Compute the current error in the constrained objective direction with respect to constraint i

			double e = deriv.row(i).dot(dir);

			// Adjust constraint i's weight to cancel out the error (but don't allow a negative weight)

			double yNew = max(0.0, y[i] + e / deriv.row(i).squaredNorm());
			y[i] = yNew;

			Matrix<double, numVars, 1> dirAfter = obj - deriv.transpose() * y;

			debug_printf("%2u: %g (%g) --> %g (%g)\n", i, e, dir.norm(), deriv.row(i).dot(dirAfter), dirAfter.norm());
		}

		debug_printf("\n");
	}

	for (size_t i = 0; i < numConstraints; ++i)
	{
		traj.var[c0 + i] = y[i];
	}

	debug_printf("Final multipliers:");
	for (size_t i = 0; i < numConstraints; ++i)
		debug_printf(" %g", y[i]);
	debug_printf("\n");

	Matrix<double, numVars, 1> dir = obj - deriv.transpose() * y;
	dir *= std::min(32768.0, 1.0 / dir.norm());

	debug_printf("Constrained objective direction:");
	for (size_t i = 0; i < numVars; ++i)
		debug_printf(" %g", dir[i]);
	debug_printf("\n");

	for (size_t i = 0; i < numVars; ++i)
		traj.var[i] += dir[i];
}

void moveTowardFeasibility(Trajectory & traj)
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

	debug_printf("\nConstraints:\n");

	for (size_t i = 0; i < numConstraints; ++i)
	{
		debug_printf("%2u:", i);
		for (size_t j = 0; j < numVars; ++j)
		{
			debug_printf(" %g", constraintGradient(i, j));
		}
		debug_printf(" | %g x %g\n", constraintError[i], cm[i]);
	}

	debug_printf("   ");
	for (size_t i = 0; i < numVars; ++i)
		debug_printf(" %g", dX[i]);
	debug_printf("\n");

	for (size_t i = 0; i < numVars; ++i)
		traj.var[i] += dX[i];
}

void moveInConstrainedGradientDir(Trajectory & traj)
{
	// Unconstrained objective direction

	Matrix<double, numVars, 1> obj;
	obj << -0.707107, -0.707107, 0;

	// Collect active constraints

	Matrix<double, numConstraints, 1> constraintError;
	Matrix<double, numConstraints, numVars> constraintGradient;
	evalConstraints(traj, constraintError, constraintGradient);

	debug_printf("\n");

	size_t n = 0;
	size_t constraintIndex[numConstraints];
	for (size_t i = 0; i < numConstraints; ++i)
	{
		double d = 0;
		for (size_t j = 0; j < numVars; ++j)
			d += constraintGradient(i, j) * obj[j];

		debug_printf("Constraint %u: dot=%g, err=%g\n", i, d, constraintError[i]);

		if (constraintError[i] <= -1.0e-4)
			continue;

		constraintIndex[n] = i;
		++n;
	}

	// Constrain the objective direction to keep it from violating active constraints

	if (n > 0)
	{
		// Solve for Lagrange multipliers on the active constraints that will keep the objective direction from going in the constraint gradient direction

		MatrixXd g(n, numVars);
		for (size_t j = 0; j < n; ++j)
		{
			size_t i = constraintIndex[j];
			for (size_t k = 0; k < numVars; ++k)
			{
				g(j, k) = constraintGradient(i, k);
			}
		}

		debug_printf("Constraints:\n");
		for (size_t j = 0; j < n; ++j)
		{
			debug_printf("%2u:", constraintIndex[j]);
			for (size_t i = 0; i < numVars; ++i)
			{
				debug_printf(" %g", g(j, i));
			}
			debug_printf("\n");
		}

		MatrixXd m = g * g.transpose();
		VectorXd err = -(g * obj);
		VectorXd x = m.colPivHouseholderQr().solve(err);

		double lm[numConstraints];
		memset(lm, 0, sizeof(lm));
		for (size_t j = 0; j < n; ++j)
			lm[constraintIndex[j]] = x[j];

		obj += g.transpose() * x;

		double d = obj.norm();

		debug_printf("Constraint multipliers:");
		for (size_t i = 0; i < numConstraints; ++i)
			debug_printf(" %g", lm[i]);
		debug_printf("\n");
		debug_printf("Constraint scale: %g\n", d);

		obj *= std::min(32768.0, 4.0 / d);
	}

	// Take a step in the constrained objective direction

	debug_printf("Constrained objective dir:");
	for (size_t i = 0; i < numVars; ++i)
		debug_printf(" %g", obj[i]);
	debug_printf("\n");

	for (size_t i = 0; i < numVars; ++i)
		traj.var[i] += obj[i];
}

void trajectoryStep(const Trajectory & trajOriginal, const Matrix<double, numVars + numConstraints, 1> & dir, double scale, Trajectory & trajNew)
{
	const size_t c = numVars + numConstraints;

	for (size_t i = 0; i < c; ++i)
	{
		trajNew.var[i] = trajOriginal.var[i] + dir(i) * scale;
	}

	for (size_t i = c; i < M; ++i)
	{
		trajNew.var[i] = trajOriginal.var[i];
	}
}

bool constraintsSatisfied(const Trajectory & traj)
{
	for (size_t i = 0; i < numConstraints; ++i)
	{
		double error;
		Matrix<double, numVars, 1> grad;
		Matrix<double, numVars, numVars> secondDeriv;
		(*constraints[i])(traj, error, grad, secondDeriv);

		if (error > 0.0)
			return false;
	}

	return true;
}

void residual(const Trajectory & traj, double slack, Matrix<double, numVars + numConstraints, 1> & r)
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
		Matrix<double, numVars, numVars> secondDeriv;
		(*constraints[i])(traj, error, grad, secondDeriv);

		for (size_t j = 0; j < numVars; ++j)
		{
			r(j) += scale * grad(j);
		}

		r(numVars + i) = -scale * error - slack;
	}
}

double residualNorm(const Trajectory & traj, double slack)
{
	Matrix<double, numVars + numConstraints, 1> r;

	residual(traj, slack, r);

	return r.squaredNorm();
}

void moveInteriorPoint(Trajectory & traj, double slack)
{
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
		(*constraints[i])(traj, error, grad, secondDeriv);

		// Print the constraint info

		debug_printf("Constraint %u (scale %g):\n", i, scale);
		for (size_t row = 0; row < numVars; ++row)
		{
			for (size_t col = 0; col < numVars; ++col)
			{
				debug_printf("%10g ", secondDeriv(row, col));
			}

			debug_printf("| ");

			for (size_t col = 0; col < numVars; ++col)
			{
				debug_printf("%10g ", secondDeriv(row, col) * scale);
			}

			debug_printf("\n");
		}

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
			m(numVars + i, j) = -scale * grad(j);

			r(j) += scale * grad(j);
		}

		m(numVars + i, numVars + i) = -error;
		r(numVars + i) = -scale * error - slack;
	}

	// Print the final result

	debug_printf("Full matrix:\n");
	for (size_t row = 0; row < c; ++row)
	{
		for (size_t col = 0; col < c; ++col)
		{
			debug_printf("%9g ", m(row, col));
		}

		debug_printf("| ");

		debug_printf("%g\n", r(row));
	}

	// Solve the system for the step to try to eliminate the residual

	Matrix<double, c, 1> d;
	d = m.colPivHouseholderQr().solve(-r);

	debug_printf("Current state:\n");
	for (size_t i = 0; i < c; ++i)
	{
		debug_printf("%s%g", (i > 0) ? " " : "", traj.var[i]);
	}
	debug_printf("\nStep for slack %g:\n", slack);
	for (size_t i = 0; i < c; ++i)
	{
		debug_printf("%s%g", (i > 0) ? " " : "", d(i));
	}
	debug_printf("\n");

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
		Trajectory trajNew;
		trajectoryStep(traj, d, s, trajNew);

		if (constraintsSatisfied(trajNew))
			break;

		s *= 0.5;
	}

	// Backtrack while the magnitude of the residual is not decreasing

	double residualNormOriginal = residualNorm(traj, slack);

	for (size_t i = 0; i < 100; ++i)
	{
		Trajectory trajNew;
		trajectoryStep(traj, d, s, trajNew);

		double residualNormNew = residualNorm(trajNew, slack);

		if (residualNormNew <= residualNormOriginal * (1.0 - 0.01 * s))
			break;

		s *= 0.5;
	}

	// Take the step

	d *= s;

	for (size_t i = 0; i < c; ++i)
		traj.var[i] += d(i);
}

void fixupConstraint(Trajectory & traj, ConstraintFunc * constraint)
{
	double error;
	Matrix<double, numVars, 1> deriv;
	Matrix<double, numVars, numVars> secondDeriv;
	(*constraint)(traj, error, deriv, secondDeriv);

	if (error <= 0.0)
		return;

	double d = deriv.squaredNorm();

	double u = error / d;

	for (size_t i = 0; i < numVars; ++i)
		traj.var[i] -= deriv[i] * u;
}

void printConstraints(const Matrix<double, numConstraints, 1> & error, const Matrix<double, numConstraints, numVars> & deriv)
{
	Matrix<double, numVars, 1> obj;
	obj << -0.707107, -0.707107, 0;

	for (size_t i = 0; i < numConstraints; ++i)
	{
		double d = obj.dot(deriv.row(i));

		debug_printf("%c%u:", (error[i] > 0) ? '*' : ' ', i);
		for (size_t j = 0; j < numVars; ++j)
		{
			debug_printf(" %g", deriv(i, j));
		}
		debug_printf(" | %g (dot %g)\n", error[i], d);
	}
}

void printState(const Trajectory & traj)
{
	debug_printf("\nNode 0: pos=%g vel=%g\n", g_trajectory.var[pos0X], g_trajectory.var[vel0X]);
	debug_printf("Node 1: pos=%g vel=%g\n", g_trajectory.var[pos1X], g_trajectory.var[vel1X]);
	debug_printf("Node 2: pos=%g vel=%g\n", g_trajectory.var[pos2X], g_trajectory.var[vel2X]);
	debug_printf("Duration 0: %g\n", g_trajectory.var[duration0]);
	debug_printf("Duration 1: %g\n", g_trajectory.var[duration1]);

	Matrix<double, numConstraints, 1> constraintError;
	Matrix<double, numConstraints, numVars> constraintGradient;
	evalConstraints(g_trajectory, constraintError, constraintGradient);

	debug_printf("Constraints:\n");
	printConstraints(constraintError, constraintGradient);
}

void plotAcceleration(const Trajectory & traj)
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

void plotTrajectory(const Trajectory & traj)
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
