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
const char * kWindowName = "Rocket Path";

const double pi = 3.1415926535897932384626433832795;

//----------------------------------------------------------------------------

static void onDraw();
static void onMouseButton(int button, int state, int x, int y);
static void onMouseMove(int x, int y);
static void onKey(unsigned char key, int x, int y);
static void onSpecialKey(int key, int x, int y);

//----------------------------------------------------------------------------

static unsigned g_disc_list = 0;

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

static Problem * g_problemCur = &g_problem4;

//----------------------------------------------------------------------------

int main(int argc, char * argv[])
{
	for (Problem * problem : g_problems)
	{
		problem->init();
	}

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

	glutMainLoop();

	return 0;
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
	g_problemCur->onSpecialKey(key);
}

void drawDisc()
{
	glCallList(g_disc_list);
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
