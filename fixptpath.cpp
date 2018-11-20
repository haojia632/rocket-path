#include "fixptpath.h"

#include "draw.h"

#include <GL/glut.h>

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
	vel1Y,

	// constants

	pos0X,
	pos0Y,
	vel0X,
	vel0Y,
	pos1X,
	pos1Y,
	pos2X,
	pos2Y,
	vel2X,
	vel2Y,

	M
};

static const size_t numNodes = 3;
static const size_t numConstraints = 4;

struct Trajectory
{
	double var[M]; // variables and constants
};

static int g_mouse_x = 0;
static int g_mouse_y = 0;
static size_t g_highlightedNode = 0;
static size_t g_selectedNode = 0;
static Vector2d g_posOffset(0, 0);
static Trajectory g_trajectory;

const double accelerationLimit = 100.0;

const double discRadius = 10.0;

static Vector2d & posNode(Trajectory &, size_t i);
static void getConstraintErrors(const Trajectory &, double error[numConstraints]);
static void moveTowardFeasibility(Trajectory &);
static void moveInConstrainedGradientDir(Trajectory &);
static void printState(const Trajectory &);

static void updateHighlight(int mouseX, int mouseY);
static Vector2d posMouseWorld(int mouseX, int mouseY);
static size_t closestNode(int mouseX, int mouseY);
static void projectionMatrix(double m[16]);
static void modelViewMatrix(double m[16]);

static Vector2d posFromCubic(const Vector2d & x0, const Vector2d & v0, const Vector2d & x1, const Vector2d & v1, double h, double u);
static Vector2d posFromTime(const Trajectory &, double t);
static void plotTrajectory(const Trajectory &);
static void plotAcceleration(const Trajectory &);
static void plotAccelerations(const Trajectory &);
static void plotSegmentAccelerationMagnitude(const Vector2d & x0, const Vector2d & v0, const Vector2d & x1, const Vector2d & v1, double aMax, double u0, double u1, double h, double r, double g, double b);
static void descendObjective(Trajectory &);
static void descendObjectiveConstrained(Trajectory &);
static void minimizeAcceleration1(Trajectory &);
static void minimizeAcceleration2(Trajectory &);

static double constraintError0(const Trajectory &);
static double constraintError1(const Trajectory &);
static double constraintError2(const Trajectory &);
static double constraintError3(const Trajectory &);

static void constraintGradient0(const Trajectory &, double deriv[4]);
static void constraintGradient1(const Trajectory &, double deriv[4]);
static void constraintGradient2(const Trajectory &, double deriv[4]);
static void constraintGradient3(const Trajectory &, double deriv[4]);

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

FixPointPath::FixPointPath()
{
}

FixPointPath::~FixPointPath()
{
}

void FixPointPath::init()
{
	memset(&g_trajectory, 0, sizeof(g_trajectory));

	g_trajectory.var[pos0X] = -200;
	g_trajectory.var[pos0Y] = 0;
	g_trajectory.var[vel0X] = 0;
	g_trajectory.var[vel0Y] = 0;

	g_trajectory.var[pos1X] = 0;
	g_trajectory.var[pos1Y] = 0;
	g_trajectory.var[vel1X] = 0;// 100;
	g_trajectory.var[vel1Y] = 0;

	g_trajectory.var[pos2X] = 200;
	g_trajectory.var[pos2Y] = 0;
	g_trajectory.var[vel2X] = 0;
	g_trajectory.var[vel2Y] = 0;

	g_trajectory.var[duration0] = 3.4641;
	g_trajectory.var[duration1] = 3.4641;

	// Nothing selected, initially

	g_highlightedNode = numNodes;
	g_selectedNode = numNodes;
}

void FixPointPath::onActivate()
{
	printf("2D path\n");
}

void FixPointPath::onKey(unsigned char key)
{
	switch (key)
	{
	case ' ':
		moveTowardFeasibility(g_trajectory);
		repaint();
		break;

	case 'c':
		descendObjectiveConstrained(g_trajectory);
		repaint();
		break;

	case 'd':
		descendObjective(g_trajectory);
		repaint();
		break;

	case 'i':
		init();
		updateHighlight(g_mouse_x, g_mouse_y);
		repaint();
		break;

	case 'p':
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
			printf("Constraint errors: %g %g %g %g --> %g\n", error[0], error[1], error[2], error[3], errorAccum);
		}
		break;

	case 's':
		printState(g_trajectory);
		break;

	case 'z':
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

	case '5':
		minimizeAcceleration1(g_trajectory);
		repaint();
		break;

	case '6':
		minimizeAcceleration2(g_trajectory);
		repaint();
		break;
	}
}

void FixPointPath::onSpecialKey(int key)
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
		g_trajectory.var[vel1Y] -= 1;
		repaint();
		break;

	case GLUT_KEY_DOWN:
		g_trajectory.var[vel1Y] += 1;
		repaint();
		break;
	}
}

void FixPointPath::onDraw()
{
	glClear(GL_COLOR_BUFFER_BIT);

	// Plot trajectory

	double m[16];

	glMatrixMode(GL_PROJECTION);
	projectionMatrix(m);
	glLoadMatrixd(m);

	glMatrixMode(GL_MODELVIEW);
	modelViewMatrix(m);
	glLoadMatrixd(m);

	plotTrajectory(g_trajectory);

	// Plot acceleration graph

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslated(-0.9, 0.4, 0);
	glScaled(1.8, 0.5, 1.0);

	plotAcceleration(g_trajectory);

	// Plot accelerations into limit circle

	glLoadIdentity();
	glTranslated(-0.65, 0.1, 0);
	glScaled(0.25, 0.25, 1.0);

	plotAccelerations(g_trajectory);
}

void FixPointPath::onMouseMove(int x, int y)
{
	g_mouse_x = x;
	g_mouse_y = y;

	if (g_selectedNode < numNodes)
	{
		posNode(g_trajectory, g_selectedNode) = g_posOffset + posMouseWorld(g_mouse_x, g_mouse_y);
	}
	else
	{
		updateHighlight(g_mouse_x, g_mouse_y);
	}

	repaint();
}

void FixPointPath::onMouseDown()
{
	g_selectedNode = g_highlightedNode;

	if (g_selectedNode < numNodes)
	{
		g_posOffset = posNode(g_trajectory, g_selectedNode) - posMouseWorld(g_mouse_x, g_mouse_y);
	}
}

void FixPointPath::onMouseUp()
{
	g_selectedNode = numNodes;

	updateHighlight(g_mouse_x, g_mouse_y);
}

Vector2d & posNode(Trajectory & traj, size_t i)
{
	switch (i)
	{
	case 0:
		return reinterpret_cast<Vector2d &>(traj.var[pos0X]);
	case 1:
		return reinterpret_cast<Vector2d &>(traj.var[pos1X]);
	case 2:
	default:
		return reinterpret_cast<Vector2d &>(traj.var[pos2X]);
	}
}

double constraintError0(const Trajectory & traj)
{
	const double h = traj.var[duration0];
	const Vector2d p0(traj.var[pos0X], traj.var[pos0Y]);
	const Vector2d v0(traj.var[vel0X], traj.var[vel0Y]);
	const Vector2d p1(traj.var[pos1X], traj.var[pos1Y]);
	const Vector2d v1(traj.var[vel1X], traj.var[vel1Y]);
	const Vector2d dPos = p1 - p0;

	Vector2d a = (dPos * 6.0 / h + v0 * -4.0 + v1 * -2.0) / h;

	return a.squaredNorm() - sqr(accelerationLimit);
}

void constraintGradient0(const Trajectory & traj, double deriv[4])
{
	const double h = traj.var[duration0];
	const Vector2d p0(traj.var[pos0X], traj.var[pos0Y]);
	const Vector2d v0(traj.var[vel0X], traj.var[vel0Y]);
	const Vector2d p1(traj.var[pos1X], traj.var[pos1Y]);
	const Vector2d v1(traj.var[vel1X], traj.var[vel1Y]);
	const Vector2d dPos = p1 - p0;

	// Take derivatives of dot(a, a) with respect to h, v1.x, and v1.y

	const Vector2d a = (dPos * 6.0 / h + v0 * -4.0 + v1 * -2.0) / h;
	const Vector2d dAdH = (dPos * -12.0 / h + v0 * 4.0 + v1 * 2.0) / sqr(h);

	deriv[duration0] = 2.0 * a.dot(dAdH);
	deriv[duration1] = 0;
	deriv[vel1X] = a[0] * -4.0 / h; // 2 * a[0] * d(a[0])/dV1X
	deriv[vel1Y] = a[1] * -4.0 / h;
}

double constraintError1(const Trajectory & traj)
{
	const double h = traj.var[duration0];
	const Vector2d p0(traj.var[pos0X], traj.var[pos0Y]);
	const Vector2d v0(traj.var[vel0X], traj.var[vel0Y]);
	const Vector2d p1(traj.var[pos1X], traj.var[pos1Y]);
	const Vector2d v1(traj.var[vel1X], traj.var[vel1Y]);
	const Vector2d dPos = p1 - p0;

	Vector2d a = (dPos * -6.0 / h + v0 * 2.0 + v1 * 4.0) / h;

	return a.squaredNorm() - sqr(accelerationLimit);
}

void constraintGradient1(const Trajectory & traj, double deriv[4])
{
	const double h = traj.var[duration0];
	const Vector2d p0(traj.var[pos0X], traj.var[pos0Y]);
	const Vector2d v0(traj.var[vel0X], traj.var[vel0Y]);
	const Vector2d p1(traj.var[pos1X], traj.var[pos1Y]);
	const Vector2d v1(traj.var[vel1X], traj.var[vel1Y]);
	const Vector2d dPos = p1 - p0;

	// Take derivatives of dot(a, a) with respect to h, v1.x, and v1.y

	const Vector2d a = (dPos * -6.0 / h + v0 * 2.0 + v1 * 4.0) / h;
	const Vector2d dAdH = (dPos * 12.0 / h + v0 * -2.0 + v1 * -4.0) / sqr(h);

	deriv[duration0] = 2.0 * a.dot(dAdH);
	deriv[duration1] = 0;
	deriv[vel1X] = a[0] * 8.0 / h;
	deriv[vel1Y] = a[1] * 8.0 / h;
}

double constraintError2(const Trajectory & traj)
{
	const double h = traj.var[duration1];
	const Vector2d p0(traj.var[pos1X], traj.var[pos1Y]);
	const Vector2d v0(traj.var[vel1X], traj.var[vel1Y]);
	const Vector2d p1(traj.var[pos2X], traj.var[pos2Y]);
	const Vector2d v1(traj.var[vel2X], traj.var[vel2Y]);
	const Vector2d dPos = p1 - p0;

	Vector2d a = (dPos * 6.0 / h + v0 * -4.0 + v1 * -2.0) / h;

	return a.squaredNorm() - sqr(accelerationLimit);
}

void constraintGradient2(const Trajectory & traj, double deriv[4])
{
	const double h = traj.var[duration1];
	const Vector2d p0(traj.var[pos1X], traj.var[pos1Y]);
	const Vector2d v0(traj.var[vel1X], traj.var[vel1Y]);
	const Vector2d p1(traj.var[pos2X], traj.var[pos2Y]);
	const Vector2d v1(traj.var[vel2X], traj.var[vel2Y]);
	const Vector2d dPos = p1 - p0;

	// Take derivatives of dot(a, a) with respect to h, v1.x, and v1.y

	const Vector2d a = (dPos * 6.0 / h + v0 * -4.0 + v1 * -2.0) / h;
	const Vector2d dAdH = (dPos * -12.0 / h + v0 * 4.0 + v1 * 2.0) / sqr(h);

	deriv[duration0] = 0;
	deriv[duration1] = 2.0 * a.dot(dAdH);
	deriv[vel1X] = a[0] * -8.0 / h;
	deriv[vel1Y] = a[1] * -8.0 / h;
}

double constraintError3(const Trajectory & traj)
{
	const double h = traj.var[duration1];
	const Vector2d p0(traj.var[pos1X], traj.var[pos1Y]);
	const Vector2d v0(traj.var[vel1X], traj.var[vel1Y]);
	const Vector2d p1(traj.var[pos2X], traj.var[pos2Y]);
	const Vector2d v1(traj.var[vel2X], traj.var[vel2Y]);
	const Vector2d dPos = p1 - p0;

	Vector2d a = (dPos * -6.0 / h + v0 * 2.0 + v1 * 4.0) / h;

	return a.squaredNorm() - sqr(accelerationLimit);
}

void constraintGradient3(const Trajectory & traj, double deriv[4])
{
	const double h = traj.var[duration1];
	const Vector2d p0(traj.var[pos1X], traj.var[pos1Y]);
	const Vector2d v0(traj.var[vel1X], traj.var[vel1Y]);
	const Vector2d p1(traj.var[pos2X], traj.var[pos2Y]);
	const Vector2d v1(traj.var[vel2X], traj.var[vel2Y]);
	const Vector2d dPos = p1 - p0;

	// Take derivatives of dot(a, a) with respect to h, v1.x, and v1.y

	const Vector2d a = (dPos * -6.0 / h + v0 * 2.0 + v1 * 4.0) / h;
	const Vector2d dAdH = (dPos * 12.0 / h + v0 * -2.0 + v1 * -4.0) / sqr(h);

	deriv[duration0] = 0;
	deriv[duration1] = 2.0 * a.dot(dAdH);
	deriv[vel1X] = a[0] * 4.0 / h;
	deriv[vel1Y] = a[1] * 4.0 / h;
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

	printf("\nConstraint errors: %g %g %g %g\n",
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
		printf("No constraints violated\n");
		return;
	}

	// Get the errors and gradients of the violated constraints

	double constraintGradient[numConstraints][4];
	constraintGradient0(traj, constraintGradient[0]);
	constraintGradient1(traj, constraintGradient[1]);
	constraintGradient2(traj, constraintGradient[2]);
	constraintGradient3(traj, constraintGradient[3]);

	MatrixXd g(n, 4);
	VectorXd err(n);
	for (size_t j = 0; j < n; ++j)
	{
		size_t i = constraintIndex[j];
		err(j) = constraintError[i];
		for (size_t k = 0; k < 4; ++k)
		{
			g(j, k) = constraintGradient[i][k];
		}
	}

	printf("Constraint gradients:\n");
	for (size_t j = 0; j < n; ++j)
	{
		for (size_t i = 0; i < 4; ++i)
		{
			printf(" %g", g(j, i));
		}
		printf("\n");
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

	printf("Multipliers: %g %g %g %g\n", cm[0], cm[1], cm[2], cm[3]);

	Vector4d x = Vector4d::Zero();
	x -= g.transpose() * m;

	printf("Move: %g %g %g %g\n", x[0], x[1], x[2], x[3]);

	traj.var[0] += x[0];
	traj.var[1] += x[1];
	traj.var[2] += x[2];
	traj.var[3] += x[3];

	traj.var[0] = max(1.0e-4, traj.var[0]);
	traj.var[1] = max(1.0e-4, traj.var[1]);
}

void moveInConstrainedGradientDir(Trajectory & traj)
{
	// Unconstrained objective direction

	Vector4d obj(-0.707107, -0.707107, 0, 0);

	// Collect active constraints

	double constraintError[numConstraints];
	getConstraintErrors(traj, constraintError);

	double constraintGradient[numConstraints][4];
	constraintGradient0(traj, constraintGradient[0]);
	constraintGradient1(traj, constraintGradient[1]);
	constraintGradient2(traj, constraintGradient[2]);
	constraintGradient3(traj, constraintGradient[3]);

	printf("\n");

	size_t n = 0;
	size_t constraintIndex[numConstraints];
	for (size_t i = 0; i < numConstraints; ++i)
	{
		double d = 0;
		for (size_t j = 0; j < 4; ++j)
			d += constraintGradient[i][j] * obj[j];

		printf("Constraint %u: dot=%g, err=%g\n", i, d, constraintError[i]);

		if (constraintError[i] <= -1.0e-4)
			continue;

		constraintIndex[n] = i;
		++n;
	}

	// Constrain the objective direction to keep it from violating active constraints

	if (n > 0)
	{
		// Solve for Lagrange multipliers on the active constraints that will keep the objective direction from going in the constraint gradient direction

		MatrixXd g(n, 4);
		for (size_t j = 0; j < n; ++j)
		{
			size_t i = constraintIndex[j];
			for (size_t k = 0; k < 4; ++k)
			{
				g(j, k) = constraintGradient[i][k];
			}
		}

		printf("Constraint gradients:\n");
		for (size_t j = 0; j < n; ++j)
		{
			printf("%u:", constraintIndex[j]);
			for (size_t i = 0; i < 4; ++i)
			{
				printf(" %g", g(j, i));
			}
			printf("\n");
		}

		MatrixXd m = g * g.transpose();
		VectorXd err = -(g * obj);
		VectorXd x = m.colPivHouseholderQr().solve(err);

		Vector4d lm = Vector4d::Zero();
		for (size_t j = 0; j < n; ++j)
			lm[constraintIndex[j]] = x[j];

		obj += g.transpose() * x;

		double d = obj.norm();

		printf("Constraint multipliers: %g %g %g %g\n", lm[0], lm[1], lm[2], lm[3]);
		printf("Constraint scale: %g\n", d);

		obj /= max(1.0 / 1024.0, d);
	}

	// Take a step in the constrained objective direction

	printf("Constrained objective dir: %g %g %g %g\n", obj[0], obj[1], obj[2], obj[3]);

	traj.var[0] += obj[0];
	traj.var[1] += obj[1];
	traj.var[2] += obj[2];
	traj.var[3] += obj[3];

	traj.var[0] = max(1.0e-4, traj.var[0]);
	traj.var[1] = max(1.0e-4, traj.var[1]);
}

void printState(const Trajectory & traj)
{
	printf("\nNode 0: pos=[%g %g] vel=[%g %g]\n", g_trajectory.var[pos0X], g_trajectory.var[pos0Y], g_trajectory.var[vel0X], g_trajectory.var[vel0Y]);
	printf("Node 1: pos=[%g %g] vel=[%g %g]\n", g_trajectory.var[pos1X], g_trajectory.var[pos1Y], g_trajectory.var[vel1X], g_trajectory.var[vel1Y]);
	printf("Node 2: pos=[%g %g] vel=[%g %g]\n", g_trajectory.var[pos2X], g_trajectory.var[pos2Y], g_trajectory.var[vel2X], g_trajectory.var[vel2Y]);
	printf("Duration 0: %g\n", g_trajectory.var[duration0]);
	printf("Duration 1: %g\n", g_trajectory.var[duration1]);

	double err[4];
	getConstraintErrors(g_trajectory, err);

	double constraintGradient[numConstraints][4];
	constraintGradient0(traj, constraintGradient[0]);
	constraintGradient1(traj, constraintGradient[1]);
	constraintGradient2(traj, constraintGradient[2]);
	constraintGradient3(traj, constraintGradient[3]);

	for (size_t i = 0; i < 4; ++i)
	{
		printf("%u: %g %g %g %g = %g\n", i, constraintGradient[i][0], constraintGradient[i][1], constraintGradient[i][2], constraintGradient[i][3], err[i]);
	}
}

void updateHighlight(int mouseX, int mouseY)
{
	size_t i = closestNode(mouseX, mouseY);
	if (i != g_highlightedNode)
	{
		g_highlightedNode = i;
		repaint();
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

Vector2d posMouseWorld(int mouseX, int mouseY)
{
	int viewport[4] = { 0, 0, windowSizeX(), windowSizeY() };
	double mvmatrix[16], projmatrix[16];
	projectionMatrix(projmatrix);
	modelViewMatrix(mvmatrix);

	mouseY = viewport[3] - mouseY - 1;

	Vector3d pos;
	gluUnProject(mouseX, mouseY, 0, mvmatrix, projmatrix, viewport, &pos[0], &pos[1], &pos[2]);

	return Vector2d(pos[0], pos[1]);
}

size_t closestNode(int mouse_x, int mouse_y)
{
	Vector2d posMouse = posMouseWorld(mouse_x, mouse_y);

	const double sqR = discRadius*discRadius;

	size_t iClosest = numNodes;
	double closestSqDist = std::numeric_limits<double>::infinity();
	for (size_t i = 0; i < numNodes; ++i)
	{
		double sqDist = (posNode(g_trajectory, i) - posMouse).squaredNorm();
		if (sqDist < sqR && (iClosest >= numNodes || sqDist < closestSqDist))
		{
			iClosest = i;
			closestSqDist = sqDist;
		}
	}

	return iClosest;
}

void plotSegmentAccelerationMagnitude(const Vector2d & x0, const Vector2d & v0, const Vector2d & x1, const Vector2d & v1, double aMax, double u0, double u1, double h, double r, double g, double b)
{
	glColor3d(r, g, b);

	Vector2d a0 = x0 * (-6.0 / sqr(h)) + x1 * (6.0 / sqr(h)) + v0 * (-4.0 / h) + v1 * (-2.0 / h);
	Vector2d a1 = x0 * (6.0 / sqr(h)) + x1 * (-6.0 / sqr(h)) + v0 * (2.0 / h) + v1 * (4.0 / h);

	glBegin(GL_LINE_STRIP);

	for (size_t j = 0; j < 48; ++j)
	{
		double t = double(j) / 48.0;

		Vector2d a = a0 + (a1 - a0) * t;
		double u = u0 + (u1 - u0) * t;

		glVertex2d(u, a.squaredNorm() / aMax);
	}

	glVertex2d(u1, a1.squaredNorm() / aMax);

	glEnd();
}

void plotAcceleration(const Trajectory & traj)
{
	double tTotal = traj.var[duration0] + traj.var[duration1];

	double aMax = 1;

	{
		Vector2d x0(traj.var[pos0X], traj.var[pos0Y]);
		Vector2d x1(traj.var[pos1X], traj.var[pos1Y]);
		Vector2d v0(traj.var[vel0X], traj.var[vel0Y]);
		Vector2d v1(traj.var[vel1X], traj.var[vel1Y]);
		double h = traj.var[duration0];

		Vector2d a0 = x0 * (-6.0 / sqr(h)) + x1 * (6.0 / sqr(h)) + v0 * (-4.0 / h) + v1 * (-2.0 / h);
		Vector2d a1 = x0 * (6.0 / sqr(h)) + x1 * (-6.0 / sqr(h)) + v0 * (2.0 / h) + v1 * (4.0 / h);

		aMax = std::max(aMax, a0.squaredNorm());
		aMax = std::max(aMax, a1.squaredNorm());
	}

	{
		Vector2d x0(traj.var[pos1X], traj.var[pos1Y]);
		Vector2d x1(traj.var[pos2X], traj.var[pos2Y]);
		Vector2d v0(traj.var[vel1X], traj.var[vel1Y]);
		Vector2d v1(traj.var[vel2X], traj.var[vel2Y]);
		double h = traj.var[duration1];

		Vector2d a0 = x0 * (-6.0 / sqr(h)) + x1 * (6.0 / sqr(h)) + v0 * (-4.0 / h) + v1 * (-2.0 / h);
		Vector2d a1 = x0 * (6.0 / sqr(h)) + x1 * (-6.0 / sqr(h)) + v0 * (2.0 / h) + v1 * (4.0 / h);

		aMax = std::max(aMax, a0.squaredNorm());
		aMax = std::max(aMax, a1.squaredNorm());
	}

	aMax = std::max(aMax, sqr(accelerationLimit * 2));

	glColor3d(0.2, 0.2, 0.2);
	glBegin(GL_LINE_LOOP);
	glVertex2d(0, 0);
	glVertex2d(1, 0);
	glVertex2d(1, 1);
	glVertex2d(0, 1);
	glEnd();

	glBegin(GL_LINES);

	{
		double u = traj.var[duration0] / tTotal;
		glVertex2d(u, 0);
		glVertex2d(u, 1);
	}

	{
		glColor3d(0.5, 0.1, 0.1);

		double y = sqr(accelerationLimit) / aMax;

		glVertex2d(0, y);
		glVertex2d(1, y);
	}

	glEnd();

	double u0 = 0;
	double u1 = traj.var[duration0] / tTotal;

	plotSegmentAccelerationMagnitude(
		Vector2d(traj.var[pos0X], traj.var[pos0Y]),
		Vector2d(traj.var[vel0X], traj.var[vel0Y]),
		Vector2d(traj.var[pos1X], traj.var[pos1Y]),
		Vector2d(traj.var[vel1X], traj.var[vel1Y]),
		aMax,
		0,
		traj.var[duration0] / tTotal,
		traj.var[duration0],
		1, 1, 0
	);

	plotSegmentAccelerationMagnitude(
		Vector2d(traj.var[pos1X], traj.var[pos1Y]),
		Vector2d(traj.var[vel1X], traj.var[vel1Y]),
		Vector2d(traj.var[pos2X], traj.var[pos2Y]),
		Vector2d(traj.var[vel2X], traj.var[vel2Y]),
		aMax,
		traj.var[duration0] / tTotal,
		1,
		traj.var[duration1],
		0, 1, 1
	);
}

void plotAccelerations(const Trajectory & traj)
{
	// Plot the constraint curves if you hold vy and h1 constant (vx versus h0)

	glColor3d(0.5, 0.1, 0.1);
	glBegin(GL_LINE_LOOP);
	for (int i = 0; i < 32; ++i)
	{
		double a = double(i) * pi / 16.0;
		glVertex2d(cos(a), sin(a));
	}
	glEnd();

	const double aMax = accelerationLimit;
	const double h0 = traj.var[duration0];
	const double h1 = traj.var[duration1];
	const Vector2d p0(traj.var[pos0X], traj.var[pos0Y]);
	const Vector2d v0(traj.var[vel0X], traj.var[vel0Y]);
	const Vector2d p1(traj.var[pos1X], traj.var[pos1Y]);
	const Vector2d v1(traj.var[vel1X], traj.var[vel1Y]);
	const Vector2d p2(traj.var[pos2X], traj.var[pos2Y]);
	const Vector2d v2(traj.var[vel2X], traj.var[vel2Y]);
	const Vector2d dPos0 = p1 - p0;
	const Vector2d dPos1 = p2 - p1;

	Vector2d a0 = (dPos0 * ( 6.0 / sqr(h0)) + v0 * -4.0 / h0 + v1 * -2.0 / h0) / aMax;
	Vector2d a1 = (dPos0 * (-6.0 / sqr(h0)) + v0 *  2.0 / h0 + v1 *  4.0 / h0) / aMax;
	Vector2d a2 = (dPos1 * ( 6.0 / sqr(h1)) + v1 * -4.0 / h1 + v2 * -2.0 / h1) / aMax;
	Vector2d a3 = (dPos1 * (-6.0 / sqr(h1)) + v1 *  2.0 / h1 + v2 *  4.0 / h1) / aMax;

	glColor3d(1, 1, 0);
	glPushMatrix();
	glTranslated(a0[0], a0[1], 0.0);
	glScaled(0.04, 0.04, 1.0);
	drawDisc();
	glPopMatrix();

	glColor3d(0, 1, 1);
	glPushMatrix();
	glTranslated(a2[0], a2[1], 0.0);
	glScaled(0.04, 0.04, 1.0);
	drawDisc();
	glPopMatrix();

	glBegin(GL_LINES);
	glColor3d(1, 1, 0);
	glVertex2dv(&a0[0]);
	glVertex2dv(&a1[0]);
	glColor3d(0, 1, 1);
	glVertex2dv(&a2[0]);
	glVertex2dv(&a3[0]);
	glEnd();
}

static void drawNode(double x, double y, bool highlighted)
{
	if (highlighted)
		glColor3d(1, 1, 1);
	else
		glColor3d(0.65, 0.65, 0.65);

	glPushMatrix();
	glTranslated(x, y, 0.0);
	glScaled(discRadius, discRadius, 1.0);
	drawDisc();
	glPopMatrix();
}

static void drawSegment(const Vector2d & x0, const Vector2d & v0, const Vector2d & x1, const Vector2d & v1, double h, double r, double g, double b)
{
	glColor3d(r, g, b);

	Vector2d acc0 = (x1 - x0) * (6.0 / sqr(h)) - (v0 * 4.0 + v1 * 2.0) / h;
	Vector2d jrk0 = (v1 - v0) * (2.0 / sqr(h)) - acc0 * (2.0 / h);

	// Evaluate the segment position

	glBegin(GL_LINE_STRIP);

	glVertex2dv(&x0[0]);

	for (size_t j = 1; j < 32; ++j)
	{
		double t = h * double(j) / 32.0;

		Vector2d pos = x0 + (v0 + (acc0 + jrk0 * (t / 3.0f)) * (t / 2.0f)) * t;

		glVertex2dv(&pos[0]);
	}

	glVertex2dv(&x1[0]);

	glEnd();
}

Vector2d posFromCubic(const Vector2d & x0, const Vector2d & v0, const Vector2d & x1, const Vector2d & v1, double h, double u)
{
	Vector2d acc0 = (x1 - x0) * (6.0 / sqr(h)) - (v0 * 4.0 + v1 * 2.0) / h;
	Vector2d jrk0 = (v1 - v0) * (2.0 / sqr(h)) - acc0 * (2.0 / h);

	Vector2d pos = x0 + (v0 + (acc0 + jrk0 * (u / 3.0f)) * (u / 2.0f)) * u;

	return pos;
}

Vector2d posFromTime(const Trajectory & traj, double t)
{
	if (t < traj.var[duration0])
	{
		return posFromCubic(
			Vector2d(traj.var[pos0X], traj.var[pos0Y]),
			Vector2d(traj.var[vel0X], traj.var[vel0Y]),
			Vector2d(traj.var[pos1X], traj.var[pos1Y]),
			Vector2d(traj.var[vel1X], traj.var[vel1Y]),
			traj.var[duration0],
			t
		);
	}

	t -= traj.var[duration0];

	if (t < traj.var[duration1])
	{
		return posFromCubic(
			Vector2d(traj.var[pos1X], traj.var[pos1Y]),
			Vector2d(traj.var[vel1X], traj.var[vel1Y]),
			Vector2d(traj.var[pos2X], traj.var[pos2Y]),
			Vector2d(traj.var[vel2X], traj.var[vel2Y]),
			traj.var[duration1],
			t
		);
	}

	return Vector2d(traj.var[pos2X], traj.var[pos2Y]);
}

void plotTrajectory(const Trajectory & traj)
{
	// Draw curve nodes

	drawNode(traj.var[pos0X], traj.var[pos0Y], g_highlightedNode == 0);
	drawNode(traj.var[pos1X], traj.var[pos1Y], g_highlightedNode == 1);
	drawNode(traj.var[pos2X], traj.var[pos2Y], g_highlightedNode == 2);

	// Draw the curve

	drawSegment(
		Vector2d(traj.var[pos0X], traj.var[pos0Y]),
		Vector2d(traj.var[vel0X], traj.var[vel0Y]),
		Vector2d(traj.var[pos1X], traj.var[pos1Y]),
		Vector2d(traj.var[vel1X], traj.var[vel1Y]),
		traj.var[duration0],
		1, 1, 0
	);

	drawSegment(
		Vector2d(traj.var[pos1X], traj.var[pos1Y]),
		Vector2d(traj.var[vel1X], traj.var[vel1Y]),
		Vector2d(traj.var[pos2X], traj.var[pos2Y]),
		Vector2d(traj.var[vel2X], traj.var[vel2Y]),
		traj.var[duration1],
		0, 1, 1
	);
}

void descendObjective(Trajectory & traj)
{
	traj.var[duration0] *= 0.75;
	traj.var[duration1] *= 0.75;
}

void descendObjectiveConstrained(Trajectory & traj)
{
	// Reduce h0 (traj.segmentDuration[0]) to minimum that satisfies both constraints 1 and 2
	// 1: aMax^2 - squaredNorm((x1 - x0) *  6/h0^2 + v0 * -4/h0 + v1 * -2/h0) >= 0
	// 2: aMax^2 - squaredNorm((x1 - x0) * -6/h0^2 + v0 *  2/h0 + v1 *  4/h0) >= 0

	// 1: aMax^2 - ((x1 - x0) *  6/h0^2 + vx0 * -4/h0 + vx1 * -2/h0)^2 - ((y1 - y0) *  6/h0^2 + vy0 * -4/h0 + vy1 * -2/h0)^2 >= 0

	const double aMax = accelerationLimit;
	const double h0 = traj.var[duration0];
	const Vector2d p0(traj.var[pos0X], traj.var[pos0Y]);
	const Vector2d v0(traj.var[vel0X], traj.var[vel0Y]);
	const Vector2d p1(traj.var[pos1X], traj.var[pos1Y]);
	const Vector2d v1(traj.var[vel1X], traj.var[vel1Y]);
	const Vector2d dPos = p1 - p0;

	Vector2d a0Vec = dPos * (6.0 / sqr(h0)) + v0 * -4.0 / h0 + v1 * -2.0 / h0;
	double a0 = a0Vec.norm();

	double a0Excess = a0 - aMax;

	Vector2d a1Vec = dPos * (-6.0 / sqr(h0)) + v0 * 2.0 / h0 + v1 * 4.0 / h0;
	double a1 = a1Vec.norm();

	double a1Excess = a1 - aMax;

	if (a0Excess <= 0)
		return;

	Vector2d x = dPos * (-12.0 / cube(h0)) + v0 * (4.0 / sqr(h0)) + v1 * (2.0 / sqr(h0));
	double dA_dH0 = x.dot(a0Vec) / a0;
	double dA_dVX = (-2.0 * a0Vec[0]) / (a0 * h0);
	double dA_dVY = (-2.0 * a0Vec[1]) / (a0 * h0);

	double u = a0Excess / (sqr(dA_dH0) + sqr(dA_dVX) + sqr(dA_dVY));

	traj.var[duration0] -= dA_dH0 * u;
}

void minimizeAcceleration1(Trajectory & traj)
{
	const double h0 = traj.var[duration0];
	const double h1 = traj.var[duration1];
	const double x0 = traj.var[pos0X];
	const double v0 = traj.var[vel0X];
	const double x1 = traj.var[pos1X];
	const double v1 = traj.var[vel1X];
	const double x2 = traj.var[pos2X];
	const double v2 = traj.var[vel2X];

	double v1n = -0.8 * (v0 * sqr(h1) + v1 * sqr(h0)) + 1.8 * ((x1 - x0) * sqr(h1) / h0 + (x2 - x1) * sqr(h0) / h1);
	v1n /= sqr(h0) + sqr(h1);

	traj.var[vel1X] = v1n;
}

void minimizeAcceleration2(Trajectory & traj)
{
	const double h0 = traj.var[duration0];
	const double h1 = traj.var[duration1];
	const double x0 = traj.var[pos0Y];
	const double v0 = traj.var[vel0Y];
	const double x1 = traj.var[pos1Y];
	const double v1 = traj.var[vel1Y];
	const double x2 = traj.var[pos2Y];
	const double v2 = traj.var[vel2Y];

	double v1n = -0.8 * (v0 * sqr(h1) + v1 * sqr(h0)) + 1.8 * ((x1 - x0) * sqr(h1) / h0 + (x2 - x1) * sqr(h0) / h1);
	v1n /= sqr(h0) + sqr(h1);

	traj.var[vel1Y] = v1n;
}

void fixupConstraint1(Trajectory & traj)
{
	// Constraint a1: aMax^2 - squaredNorm(x0 * -6/h0^2 + x1 * 6/h0^2 + v0 * -4/h0 + v1 * -2/h0) >= 0

	const double aMax = accelerationLimit;
	const double h0 = traj.var[duration0];
	const Vector2d p0(traj.var[pos0X], traj.var[pos0Y]);
	const Vector2d v0(traj.var[vel0X], traj.var[vel0Y]);
	const Vector2d p1(traj.var[pos1X], traj.var[pos1Y]);
	const Vector2d v1(traj.var[vel1X], traj.var[vel1Y]);
	const Vector2d dPos = p1 - p0;

	Vector2d aVec = dPos * (6.0 / sqr(h0)) + v0 * -4.0 / h0 + v1 * -2.0 / h0;
	double a = aVec.norm();

	double aExcess = a - aMax;
	if (aExcess <= 0)
		return;

	Vector2d x = dPos * (-12.0 / cube(h0)) + v0 * (4.0 / sqr(h0)) + v1 * (2.0 / sqr(h0));
	double dA_dH0 = x.dot(aVec) / a;
	double dA_dVX = (-2.0 * aVec[0]) / (a * h0);
	double dA_dVY = (-2.0 * aVec[1]) / (a * h0);

	double u = aExcess / (sqr(dA_dH0) + sqr(dA_dVX) + sqr(dA_dVY));

	traj.var[duration0] -= dA_dH0 * u;
	traj.var[vel1X] -= dA_dVX * u;
	traj.var[vel1Y] -= dA_dVY * u;
}

void fixupConstraint2(Trajectory & traj)
{
	const double aMax = accelerationLimit;
	const double h0 = traj.var[duration0];
	const Vector2d p0(traj.var[pos0X], traj.var[pos0Y]);
	const Vector2d v0(traj.var[vel0X], traj.var[vel0Y]);
	const Vector2d p1(traj.var[pos1X], traj.var[pos1Y]);
	const Vector2d v1(traj.var[vel1X], traj.var[vel1Y]);
	const Vector2d dPos = p1 - p0;

	Vector2d aVec = dPos * (-6.0 / sqr(h0)) + v0 * 2.0 / h0 + v1 * 4.0 / h0;
	double a = aVec.norm();

	double aExcess = a - aMax;
	if (aExcess <= 0)
		return;

	Vector2d x = dPos * (12.0 / cube(h0)) + v0 * (-2.0 / sqr(h0)) + v1 * (-4.0 / sqr(h0));
	double dA_dH0 = x.dot(aVec) / a;
	double dA_dVX = (4.0 * aVec[0]) / (a * h0);
	double dA_dVY = (4.0 * aVec[1]) / (a * h0);

	double u = aExcess / (sqr(dA_dH0) + sqr(dA_dVX) + sqr(dA_dVY));

	traj.var[duration0] -= dA_dH0 * u;
	traj.var[vel1X] -= dA_dVX * u;
	traj.var[vel1Y] -= dA_dVY * u;
}

void fixupConstraint3(Trajectory & traj)
{
	const double aMax = accelerationLimit;
	const double h1 = traj.var[duration1];
	const Vector2d p1(traj.var[pos1X], traj.var[pos1Y]);
	const Vector2d v1(traj.var[vel1X], traj.var[vel1Y]);
	const Vector2d p2(traj.var[pos2X], traj.var[pos2Y]);
	const Vector2d v2(traj.var[vel2X], traj.var[vel2Y]);
	const Vector2d dPos = p2 - p1;

	Vector2d aVec = dPos * (6.0 / sqr(h1)) + v1 * -4.0 / h1 + v2 * -2.0 / h1;
	double a = aVec.norm();

	double aExcess = a - aMax;
	if (aExcess <= 0)
		return;

	Vector2d x = dPos * (-12.0 / cube(h1)) + v1 * (4.0 / sqr(h1)) + v2 * (2.0 / sqr(h1));
	double dA_dH1 = x.dot(aVec) / a;
	double dA_dVX = (-4.0 * aVec[0]) / (a * h1);
	double dA_dVY = (-4.0 * aVec[1]) / (a * h1);

	double u = aExcess / (sqr(dA_dH1) + sqr(dA_dVX) + sqr(dA_dVY));

	traj.var[duration1] -= dA_dH1 * u;
	traj.var[vel1X] -= dA_dVX * u;
	traj.var[vel1Y] -= dA_dVY * u;
}

void fixupConstraint4(Trajectory & traj)
{
	const double aMax = accelerationLimit;
	const double h1 = traj.var[duration1];
	const Vector2d p1(traj.var[pos1X], traj.var[pos1Y]);
	const Vector2d v1(traj.var[vel1X], traj.var[vel1Y]);
	const Vector2d p2(traj.var[pos2X], traj.var[pos2Y]);
	const Vector2d v2(traj.var[vel2X], traj.var[vel2Y]);
	const Vector2d dPos = p2 - p1;

	Vector2d aVec = dPos * (-6.0 / sqr(h1)) + v1 * 2.0 / h1 + v2 * 4.0 / h1;
	double a = aVec.norm();

	double aExcess = a - aMax;
	if (aExcess <= 0)
		return;

	Vector2d x = dPos * (12.0 / cube(h1)) + v1 * (-2.0 / sqr(h1)) + v2 * (-4.0 / sqr(h1));
	double dA_dH1 = x.dot(aVec) / a;
	double dA_dVX = (2.0 * aVec[0]) / (a * h1);
	double dA_dVY = (2.0 * aVec[1]) / (a * h1);

	double u = aExcess / (sqr(dA_dH1) + sqr(dA_dVX) + sqr(dA_dVY));

	traj.var[duration1] -= dA_dH1 * u;
	traj.var[vel1X] -= dA_dVX * u;
	traj.var[vel1Y] -= dA_dVY * u;
}
