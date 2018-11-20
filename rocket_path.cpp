#include <GL/glut.h>

#include <cmath>
#include <cstdio>

#include "draw.h"
#include "fixptpath.h"
#include "onedpath.h"
#include "onedpath_ip.h"
#include "onedpath2_ip.h"

//----------------------------------------------------------------------------

const int kInitWindowSizeX = 800;
const int kInitWindowSizeY = 800;
const char * kWindowName = "Constrained Trajectory Optimization";

const double pi = 3.1415926535897932384626433832795;

//----------------------------------------------------------------------------

static unsigned makeDisc();
static void onDraw();
static void onMouseButton(int button, int state, int x, int y);
static void onMouseMove(int x, int y);
static void onKey(unsigned char key, int x, int y);
static void onSpecialKey(int key, int x, int y);

//----------------------------------------------------------------------------

static unsigned g_discList = 0;

static FixPointPath g_problem1;
static OneDPath g_problem2;
static OneDPathInteriorPoint g_problem3;
static OneDPath2InteriorPoint g_problem4;

static Problem * g_problems[] =
{
	&g_problem1,
	&g_problem2,
	&g_problem3,
	&g_problem4,
};

static Problem * g_problemCur = &g_problem3;

//----------------------------------------------------------------------------

int main(int argc, char * argv[])
{
	glutInit(&argc, argv);
	glutInitWindowPosition(16, 16);
	glutInitWindowSize(kInitWindowSizeX, kInitWindowSizeY);
	glutCreateWindow(kWindowName);
	glutDisplayFunc(onDraw);
	glutMouseFunc(onMouseButton);
	glutPassiveMotionFunc(onMouseMove);
	glutMotionFunc(onMouseMove);
	glutKeyboardFunc(onKey);
	glutSpecialFunc(onSpecialKey);

	g_discList = makeDisc();

	for (Problem * problem : g_problems)
	{
		problem->init();
	}

	g_problemCur->onActivate();

	glutMainLoop();

	return 0;
}

unsigned makeDisc()
{
	unsigned discList = glGenLists(1);
	glNewList(discList, GL_COMPILE);
	const int half_verts = 16;
	const double angle_inc = pi / half_verts;
	double angle = angle_inc / 2.0;
	glBegin(GL_TRIANGLE_STRIP);
	for (int i = 0; i < half_verts; ++i)
	{
		double x = cos(angle);
		double y = sin(angle);

		glVertex2d(x, -y);
		glVertex2d(x,  y);

		angle += angle_inc;
	}
	glEnd();
	glEndList();

	return discList;
}

void onDraw()
{
	g_problemCur->onDraw();

	glutSwapBuffers();
}

void onMouseButton(int button, int state, int x, int y)
{
	g_problemCur->onMouseMove(x, y);

	if (button == GLUT_LEFT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			g_problemCur->onMouseDown();
		}
		else if (state == GLUT_UP)
		{
			g_problemCur->onMouseUp();
		}
	}
}

void onMouseMove(int x, int y)
{
	g_problemCur->onMouseMove(x, y);
}

void onKey(unsigned char key, int x, int y)
{
	g_problemCur->onMouseMove(x, y);

	if (key == 27)
	{
		exit(0);
	}
	else
	{
		g_problemCur->onKey(key);
	}
}

void onSpecialKey(int key, int x, int y)
{
	g_problemCur->onMouseMove(x, y);

	if (key >= GLUT_KEY_F1 && key < GLUT_KEY_F1 + sizeof(g_problems) / sizeof(Problem *))
	{
		Problem * problem = g_problems[key - GLUT_KEY_F1];
		if (problem != g_problemCur)
		{
			g_problemCur = problem;
			g_problemCur->onActivate();
			repaint();
		}
	}
	else
	{
		g_problemCur->onSpecialKey(key);
	}
}

void drawDisc()
{
	glCallList(g_discList);
}

int windowSizeX()
{
	return glutGet(GLUT_WINDOW_WIDTH);
}

int windowSizeY()
{
	return glutGet(GLUT_WINDOW_HEIGHT);
}

void repaint()
{
	glutPostRedisplay();
}
