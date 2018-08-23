#include "onedpath.h"

#include "draw.h"
#include "vec.h"

#include <windows.h>
#include <gl/gl.h>
#include <gl/glu.h>

#undef min
#undef max

#include <Eigen/Dense>
#include <algorithm>
#include <vector>

using namespace Eigen;
using std::max;

enum V
{
	// variables

	duration0,
	duration1,
	vel1X,

	// constants

	pos0X,
	vel0X,
	pos1X,
	pos2X,
	vel2X,

	M
};

static const size_t numNodes = 3;
static const size_t numConstraints = 4;

struct Trajectory
{
	double var[M]; // variables and constants
};

static Trajectory g_trajectory;

const double accelerationLimit = 100.0;

static void getConstraintErrors(const Trajectory &, double error[numConstraints]);
static void moveTowardFeasibility(Trajectory &);
static void moveInConstrainedGradientDir(Trajectory &);
static void printState(const Trajectory &);

static void projectionMatrix(double m[16]);
static void modelViewMatrix(double m[16]);

static double posFromCubic(double x0, double v0, double x1, double v1, double h, double u);
static double posFromTime(const Trajectory &, double t);
static void plotTrajectory(const Trajectory &);
static void plotAcceleration(const Trajectory &);
static void descendObjectiveConstrained(Trajectory &);

static double constraintError0(const Trajectory &);
static double constraintError1(const Trajectory &);
static double constraintError2(const Trajectory &);
static double constraintError3(const Trajectory &);

static void constraintGradient0(const Trajectory &, double deriv[3]);
static void constraintGradient1(const Trajectory &, double deriv[3]);
static void constraintGradient2(const Trajectory &, double deriv[3]);
static void constraintGradient3(const Trajectory &, double deriv[3]);

static void fixupConstraint1(Trajectory &);
static void fixupConstraint2(Trajectory &);
static void fixupConstraint3(Trajectory &);
static void fixupConstraint4(Trajectory &);

inline double sqr(double x)
{
	return x * x;
}

inline double cube(double x)
{
	return x * x * x;
}

OneDPath::OneDPath()
{
}

OneDPath::~OneDPath()
{
}

void OneDPath::init()
{
	memset(&g_trajectory, 0, sizeof(g_trajectory));

	g_trajectory.var[pos0X] = 0;
	g_trajectory.var[vel0X] = 0;

	g_trajectory.var[pos1X] = 200;
	g_trajectory.var[vel1X] = 0;

	g_trajectory.var[pos2X] = 400;
	g_trajectory.var[vel2X] = 0;

	g_trajectory.var[duration0] = 3.4641;
	g_trajectory.var[duration1] = 3.4641;
}

void OneDPath::onKey(unsigned int key)
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

	case 'C':
		descendObjectiveConstrained(g_trajectory);
		repaint();
		break;

	case 'I':
		init();
		repaint();
		break;

	case 'P':
	{
		double error[numConstraints];
		getConstraintErrors(g_trajectory, error);
		double errorAccum = 0;
		for (double e : error)
		{
			if (e > 0)
			{
				errorAccum += sqr(e);
			}
		}
		debug_printf("Constraint errors: %g %g %g %g --> %g\n", error[0], error[1], error[2], error[3], errorAccum);
	}
	break;

	case 'S':
		printState(g_trajectory);
		break;

	case 'Z':
		moveInConstrainedGradientDir(g_trajectory);
		repaint();
		break;

	case '1':
		fixupConstraint1(g_trajectory);
		repaint();
		break;

	case '2':
		fixupConstraint2(g_trajectory);
		repaint();
		break;

	case '3':
		fixupConstraint3(g_trajectory);
		repaint();
		break;

	case '4':
		fixupConstraint4(g_trajectory);
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

void OneDPath::onDraw()
{
	const double margin = 20.0;
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

void OneDPath::onMouseMove(int x, int y)
{
}

void OneDPath::onMouseDown()
{
}

void OneDPath::onMouseUp()
{
}

double constraintError0(const Trajectory & traj)
{
	const double h = traj.var[duration0];
	const double p0 = traj.var[pos0X];
	const double v0 = traj.var[vel0X];
	const double p1 = traj.var[pos1X];
	const double v1 = traj.var[vel1X];
	const double dPos = p1 - p0;

	double a = (dPos * 6.0 / h + v0 * -4.0 + v1 * -2.0) / h;

	return sqr(a) - sqr(accelerationLimit);
}

void constraintGradient0(const Trajectory & traj, double deriv[3])
{
	const double h = traj.var[duration0];
	const double p0 = traj.var[pos0X];
	const double v0 = traj.var[vel0X];
	const double p1 = traj.var[pos1X];
	const double v1 = traj.var[vel1X];
	const double dPos = p1 - p0;

	// Take derivatives of dot(a, a) with respect to h, v1.x, and v1.y

	const double a = (dPos * 6.0 / h + v0 * -4.0 + v1 * -2.0) / h;
	const double dAdH = (dPos * -12.0 / h + v0 * 4.0 + v1 * 2.0) / sqr(h);

	deriv[duration0] = 2.0 * a * dAdH;
	deriv[duration1] = 0;
	deriv[vel1X] = a * -4.0 / h; // 2 * a[0] * d(a[0])/dV1X
}

double constraintError1(const Trajectory & traj)
{
	const double h = traj.var[duration0];
	const double p0 = traj.var[pos0X];
	const double v0 = traj.var[vel0X];
	const double p1 = traj.var[pos1X];
	const double v1 = traj.var[vel1X];
	const double dPos = p1 - p0;

	double a = (dPos * -6.0 / h + v0 * 2.0 + v1 * 4.0) / h;

	return sqr(a) - sqr(accelerationLimit);
}

void constraintGradient1(const Trajectory & traj, double deriv[3])
{
	const double h = traj.var[duration0];
	const double p0 = traj.var[pos0X];
	const double v0 = traj.var[vel0X];
	const double p1 = traj.var[pos1X];
	const double v1 = traj.var[vel1X];
	const double dPos = p1 - p0;

	// Take derivatives of dot(a, a) with respect to h, v1.x, and v1.y

	const double a = (dPos * -6.0 / h + v0 * 2.0 + v1 * 4.0) / h;
	const double dAdH = (dPos * 12.0 / h + v0 * -2.0 + v1 * -4.0) / sqr(h);

	deriv[duration0] = 2.0 * a * dAdH;
	deriv[duration1] = 0;
	deriv[vel1X] = a * 8.0 / h;
}

double constraintError2(const Trajectory & traj)
{
	const double h = traj.var[duration1];
	const double p0 = traj.var[pos1X];
	const double v0 = traj.var[vel1X];
	const double p1 = traj.var[pos2X];
	const double v1 = traj.var[vel2X];
	const double dPos = p1 - p0;

	double a = (dPos * 6.0 / h + v0 * -4.0 + v1 * -2.0) / h;

	return sqr(a) - sqr(accelerationLimit);
}

void constraintGradient2(const Trajectory & traj, double deriv[3])
{
	const double h = traj.var[duration1];
	const double p0 = traj.var[pos1X];
	const double v0 = traj.var[vel1X];
	const double p1 = traj.var[pos2X];
	const double v1 = traj.var[vel2X];
	const double dPos = p1 - p0;

	// Take derivatives of dot(a, a) with respect to h, v1.x, and v1.y

	const double a = (dPos * 6.0 / h + v0 * -4.0 + v1 * -2.0) / h;
	const double dAdH = (dPos * -12.0 / h + v0 * 4.0 + v1 * 2.0) / sqr(h);

	deriv[duration0] = 0;
	deriv[duration1] = 2.0 * a * dAdH;
	deriv[vel1X] = a * -8.0 / h;
}

double constraintError3(const Trajectory & traj)
{
	const double h = traj.var[duration1];
	const double p0 = traj.var[pos1X];
	const double v0 = traj.var[vel1X];
	const double p1 = traj.var[pos2X];
	const double v1 = traj.var[vel2X];
	const double dPos = p1 - p0;

	double a = (dPos * -6.0 / h + v0 * 2.0 + v1 * 4.0) / h;

	return sqr(a) - sqr(accelerationLimit);
}

void constraintGradient3(const Trajectory & traj, double deriv[3])
{
	const double h = traj.var[duration1];
	const double p0 = traj.var[pos1X];
	const double v0 = traj.var[vel1X];
	const double p1 = traj.var[pos2X];
	const double v1 = traj.var[vel2X];
	const double dPos = p1 - p0;

	// Take derivatives of dot(a, a) with respect to h, v1.x, and v1.y

	const double a = (dPos * -6.0 / h + v0 * 2.0 + v1 * 4.0) / h;
	const double dAdH = (dPos * 12.0 / h + v0 * -2.0 + v1 * -4.0) / sqr(h);

	deriv[duration0] = 0;
	deriv[duration1] = 2.0 * a * dAdH;
	deriv[vel1X] = a * 4.0 / h;
}

void getConstraintErrors(const Trajectory & traj, double error[numConstraints])
{
	error[0] = constraintError0(traj);
	error[1] = constraintError1(traj);
	error[2] = constraintError2(traj);
	error[3] = constraintError3(traj);
}

void moveTowardFeasibility(Trajectory & traj)
{
	// Collect violated constraints

	double constraintError[numConstraints];
	getConstraintErrors(traj, constraintError);

	debug_printf("\nConstraint errors: %g %g %g %g\n",
		max(0.0, constraintError[0]),
		max(0.0, constraintError[1]),
		max(0.0, constraintError[2]),
		max(0.0, constraintError[3]));

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

	if (n == 0)
	{
		debug_printf("No constraints violated\n");
		return;
	}

	// Get the errors and gradients of the violated constraints

	double constraintGradient[numConstraints][3];
	constraintGradient0(traj, constraintGradient[0]);
	constraintGradient1(traj, constraintGradient[1]);
	constraintGradient2(traj, constraintGradient[2]);
	constraintGradient3(traj, constraintGradient[3]);

	MatrixXd g(n, 3);
	VectorXd err(n);
	for (size_t j = 0; j < n; ++j)
	{
		size_t i = constraintIndex[j];
		err(j) = constraintError[i];
		for (size_t k = 0; k < 3; ++k)
		{
			g(j, k) = constraintGradient[i][k];
		}
	}

	debug_printf("Constraint gradients:\n");
	for (size_t j = 0; j < n; ++j)
	{
		for (size_t i = 0; i < 3; ++i)
		{
			debug_printf(" %g", g(j, i));
		}
		debug_printf("\n");
	}

	// Compute multipliers for the gradients of the violated constraints that will add up to remove the error

	MatrixXd a = g * g.transpose();

	VectorXd m = a.colPivHouseholderQr().solve(err);
	assert(m.size() == n);

	double cm[numConstraints];
	ZeroMemory(cm, sizeof(cm));
	for (size_t j = 0; j < n; ++j)
	{
		cm[constraintIndex[j]] = m[j];
	}

	debug_printf("Multipliers: %g %g %g %g\n", cm[0], cm[1], cm[2], cm[3]);

	Vector3d x = Vector3d::Zero();
	x -= g.transpose() * m;

	debug_printf("Move: %g %g %g\n", x[0], x[1], x[2]);

	traj.var[0] += x[0];
	traj.var[1] += x[1];
	traj.var[2] += x[2];

	traj.var[0] = max(1.0e-4, traj.var[0]);
	traj.var[1] = max(1.0e-4, traj.var[1]);
}

void moveInConstrainedGradientDir(Trajectory & traj)
{
	// Unconstrained objective direction

	Vector3d obj(-0.707107, -0.707107, 0);

	// Collect active constraints

	double constraintError[numConstraints];
	getConstraintErrors(traj, constraintError);

	double constraintGradient[numConstraints][3];
	constraintGradient0(traj, constraintGradient[0]);
	constraintGradient1(traj, constraintGradient[1]);
	constraintGradient2(traj, constraintGradient[2]);
	constraintGradient3(traj, constraintGradient[3]);

	debug_printf("\n");

	size_t n = 0;
	size_t constraintIndex[numConstraints];
	for (size_t i = 0; i < numConstraints; ++i)
	{
		double d = 0;
		for (size_t j = 0; j < 3; ++j)
			d += constraintGradient[i][j] * obj[j];

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

		MatrixXd g(n, 3);
		for (size_t j = 0; j < n; ++j)
		{
			size_t i = constraintIndex[j];
			for (size_t k = 0; k < 3; ++k)
			{
				g(j, k) = constraintGradient[i][k];
			}
		}

		debug_printf("Constraint gradients:\n");
		for (size_t j = 0; j < n; ++j)
		{
			debug_printf("%u:", constraintIndex[j]);
			for (size_t i = 0; i < 3; ++i)
			{
				debug_printf(" %g", g(j, i));
			}
			debug_printf("\n");
		}

		MatrixXd m = g * g.transpose();
		VectorXd err = -(g * obj);
		VectorXd x = m.colPivHouseholderQr().solve(err);

		Vector4d lm = Vector4d::Zero();
		for (size_t j = 0; j < n; ++j)
			lm[constraintIndex[j]] = x[j];

		obj += g.transpose() * x;

		double d = obj.norm();

		debug_printf("Constraint multipliers: %g %g %g %g\n", lm[0], lm[1], lm[2], lm[3]);
		debug_printf("Constraint scale: %g\n", d);

		obj /= max(1.0 / 1024.0, d);
	}

	// Take a step in the constrained objective direction

	debug_printf("Constrained objective dir: %g %g %g\n", obj[0], obj[1], obj[2]);

	traj.var[0] += obj[0];
	traj.var[1] += obj[1];
	traj.var[2] += obj[2];

	traj.var[0] = max(1.0e-4, traj.var[0]);
	traj.var[1] = max(1.0e-4, traj.var[1]);
}

void printState(const Trajectory & traj)
{
	debug_printf("\nNode 0: pos=%g vel=%g\n", g_trajectory.var[pos0X], g_trajectory.var[vel0X]);
	debug_printf("Node 1: pos=%g vel=%g\n", g_trajectory.var[pos1X], g_trajectory.var[vel1X]);
	debug_printf("Node 2: pos=%g vel=%g\n", g_trajectory.var[pos2X], g_trajectory.var[vel2X]);
	debug_printf("Duration 0: %g\n", g_trajectory.var[duration0]);
	debug_printf("Duration 1: %g\n", g_trajectory.var[duration1]);

	double err[numConstraints];
	getConstraintErrors(g_trajectory, err);

	double constraintGradient[numConstraints][3];
	constraintGradient0(traj, constraintGradient[0]);
	constraintGradient1(traj, constraintGradient[1]);
	constraintGradient2(traj, constraintGradient[2]);
	constraintGradient3(traj, constraintGradient[3]);

	for (size_t i = 0; i < numConstraints; ++i)
	{
		debug_printf("%u: %g %g %g = %g\n", i, constraintGradient[i][0], constraintGradient[i][1], constraintGradient[i][2], err[i]);
	}
}

void projectionMatrix(double m[16])
{
	double rx = windowSizeX();
	double ry = windowSizeY();
	double rz = 500;

	memset(m, 0, 16*sizeof(double));
	m[0] = 2.0 / rx;
	m[5] = 2.0 / ry;
	m[10] = -1.0 / rz;
	m[15] = 1;
}

void modelViewMatrix(double m[16])
{
	memset(m, 0, 16*sizeof(double));
	m[0] = 1;
	m[5] = 1;
	m[10] = 1;
	m[15] = 1;
	m[13] = -100;
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

	glColor3d(0.2, 0.2, 0.2);
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

	// Evaluate the segment position

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

double posFromCubic(double x0, double v0, double x1, double v1, double h, double u)
{
	double acc0 = (x1 - x0) * (6.0 / sqr(h)) - (v0 * 4.0 + v1 * 2.0) / h;
	double jrk0 = (v1 - v0) * (2.0 / sqr(h)) - acc0 * (2.0 / h);

	double pos = x0 + (v0 + (acc0 + jrk0 * (u / 3.0f)) * (u / 2.0f)) * u;

	return pos;
}

double posFromTime(const Trajectory & traj, double t)
{
	if (t < traj.var[duration0])
	{
		return posFromCubic(
			traj.var[pos0X],
			traj.var[vel0X],
			traj.var[pos1X],
			traj.var[vel1X],
			traj.var[duration0],
			t
		);
	}

	t -= traj.var[duration0];

	if (t < traj.var[duration1])
	{
		return posFromCubic(
			traj.var[pos1X],
			traj.var[vel1X],
			traj.var[pos2X],
			traj.var[vel2X],
			traj.var[duration1],
			t
		);
	}

	return traj.var[pos2X];
}

void plotTrajectory(const Trajectory & traj)
{
	glColor3d(0.2, 0.2, 0.2);
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

void descendObjectiveConstrained(Trajectory & traj)
{
	// Reduce h0 (traj.segmentDuration[0]) to minimum that satisfies both constraints 1 and 2
	// 1: aMax^2 - sqlen((x1 - x0) *  6/h0^2 + v0 * -4/h0 + v1 * -2/h0) >= 0
	// 2: aMax^2 - sqlen((x1 - x0) * -6/h0^2 + v0 *  2/h0 + v1 *  4/h0) >= 0

	// 1: aMax^2 - ((x1 - x0) *  6/h0^2 + vx0 * -4/h0 + vx1 * -2/h0)^2 - ((y1 - y0) *  6/h0^2 + vy0 * -4/h0 + vy1 * -2/h0)^2 >= 0

	const double aMax = accelerationLimit;
	const double h0 = traj.var[duration0];
	const double p0 = traj.var[pos0X];
	const double v0 = traj.var[vel0X];
	const double p1 = traj.var[pos1X];
	const double v1 = traj.var[vel1X];
	const double dPos = p1 - p0;

	double a0Vec = dPos * (6.0 / sqr(h0)) + v0 * -4.0 / h0 + v1 * -2.0 / h0;
	double a0 = fabs(a0Vec);

	double a0Excess = a0 - aMax;

	double a1Vec = dPos * (-6.0 / sqr(h0)) + v0 * 2.0 / h0 + v1 * 4.0 / h0;
	double a1 = fabs(a1Vec);

	double a1Excess = a1 - aMax;

	if (a0Excess <= 0)
		return;

	double x = dPos * (-12.0 / cube(h0)) + v0 * (4.0 / sqr(h0)) + v1 * (2.0 / sqr(h0));
	double dA_dH0 = x * a0Vec / a0;
	double dA_dVX = (-2.0 * a0Vec) / (a0 * h0);

	double u = a0Excess / (sqr(dA_dH0) + sqr(dA_dVX));

	traj.var[duration0] -= dA_dH0 * u;
	traj.var[vel1X] -= dA_dVX * u;
}

void fixupConstraint1(Trajectory & traj)
{
	// Constraint a1: aMax^2 - sqlen(x0 * -6/h0^2 + x1 * 6/h0^2 + v0 * -4/h0 + v1 * -2/h0) >= 0

	const double aMax = accelerationLimit;
	const double h0 = traj.var[duration0];
	const double p0 = traj.var[pos0X];
	const double v0 = traj.var[vel0X];
	const double p1 = traj.var[pos1X];
	const double v1 = traj.var[vel1X];
	const double dPos = p1 - p0;

	double aVec = dPos * (6.0 / sqr(h0)) + v0 * -4.0 / h0 + v1 * -2.0 / h0;
	double a = fabs(aVec);

	double aExcess = a - aMax;
	if (aExcess <= 0)
		return;

	double x = dPos * (-12.0 / cube(h0)) + v0 * (4.0 / sqr(h0)) + v1 * (2.0 / sqr(h0));
	double dA_dH0 = x * aVec / a;
	double dA_dVX = (-2.0 * aVec) / (a * h0);

	double u = aExcess / (sqr(dA_dH0) + sqr(dA_dVX));

	traj.var[duration0] -= dA_dH0 * u;
	traj.var[vel1X] -= dA_dVX * u;
}

void fixupConstraint2(Trajectory & traj)
{
	const double aMax = accelerationLimit;
	const double h0 = traj.var[duration0];
	const double p0 = traj.var[pos0X];
	const double v0 = traj.var[vel0X];
	const double p1 = traj.var[pos1X];
	const double v1 = traj.var[vel1X];
	const double dPos = p1 - p0;

	double aVec = dPos * (-6.0 / sqr(h0)) + v0 * 2.0 / h0 + v1 * 4.0 / h0;
	double a = fabs(aVec);

	double aExcess = a - aMax;
	if (aExcess <= 0)
		return;

	double x = dPos * (12.0 / cube(h0)) + v0 * (-2.0 / sqr(h0)) + v1 * (-4.0 / sqr(h0));
	double dA_dH0 = x * aVec / a;
	double dA_dVX = (4.0 * aVec) / (a * h0);

	double u = aExcess / (sqr(dA_dH0) + sqr(dA_dVX));

	traj.var[duration0] -= dA_dH0 * u;
	traj.var[vel1X] -= dA_dVX * u;
}

void fixupConstraint3(Trajectory & traj)
{
	const double aMax = accelerationLimit;
	const double h1 = traj.var[duration1];
	const double p1 = traj.var[pos1X];
	const double v1 = traj.var[vel1X];
	const double p2 = traj.var[pos2X];
	const double v2 = traj.var[vel2X];
	const double dPos = p2 - p1;

	double aVec = dPos * (6.0 / sqr(h1)) + v1 * -4.0 / h1 + v2 * -2.0 / h1;
	double a = fabs(aVec);

	double aExcess = a - aMax;
	if (aExcess <= 0)
		return;

	double x = dPos * (-12.0 / cube(h1)) + v1 * (4.0 / sqr(h1)) + v2 * (2.0 / sqr(h1));
	double dA_dH1 = x * aVec / a;
	double dA_dVX = (-4.0 * aVec) / (a * h1);

	double u = aExcess / (sqr(dA_dH1) + sqr(dA_dVX));

	traj.var[duration1] -= dA_dH1 * u;
	traj.var[vel1X] -= dA_dVX * u;
}

void fixupConstraint4(Trajectory & traj)
{
	const double aMax = accelerationLimit;
	const double h1 = traj.var[duration1];
	const double p1 = traj.var[pos1X];
	const double v1 = traj.var[vel1X];
	const double p2 = traj.var[pos2X];
	const double v2 = traj.var[vel2X];
	const double dPos = p2 - p1;

	double aVec = dPos * (-6.0 / sqr(h1)) + v1 * 2.0 / h1 + v2 * 4.0 / h1;
	double a = fabs(aVec);

	double aExcess = a - aMax;
	if (aExcess <= 0)
		return;

	double x = dPos * (12.0 / cube(h1)) + v1 * (-2.0 / sqr(h1)) + v2 * (-4.0 / sqr(h1));
	double dA_dH1 = x * aVec / a;
	double dA_dVX = (2.0 * aVec) / (a * h1);

	double u = aExcess / (sqr(dA_dH1) + sqr(dA_dVX));

	traj.var[duration1] -= dA_dH1 * u;
	traj.var[vel1X] -= dA_dVX * u;
}
