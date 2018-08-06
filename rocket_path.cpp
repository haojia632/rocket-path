#include <windows.h>
#include <gl/gl.h>
#include <gl/glu.h>

#undef min
#undef max

#include <algorithm>
#include <limits>
#include <vector>

#include "vec.h"

struct NodeState
{
	dvec2 pos;
	dvec2 vel;
};

struct Trajectory
{
	std::vector<NodeState> node;
	std::vector<double> segmentDuration;
};

//----------------------------------------------------------------------------

const int kInitWindowSizeX = 800;
const int kInitWindowSizeY = 800;
const unsigned kWindowStyle = WS_OVERLAPPEDWINDOW | WS_VISIBLE;

const char * windowClassName = "Rocket Path";
const char * windowName = "Rocket Path";

const double pi = 3.1415926535897932384626433832795;

const double discRadius = 10.0;

const double accelerationLimit = 100.0;

//----------------------------------------------------------------------------

static LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
static void EnableOpenGL(HWND, HDC &, HGLRC &);
static void DisableOpenGL(HWND, HDC, HGLRC);
static void drawScene();
static void init();
static dvec2 posMouseWorld(int mouseX, int mouseY);
static void beginMouseDrag(HWND hWnd, int mouseX, int mouseY);
static void endMouseDrag();
static void updateMouseDrag(int mouseX, int mouseY);
static size_t closestNode(int mouseX, int mouseY);
static void updateHighlight(int mouseX, int mouseY);

static void repaint();

static void plotTrajectory(const Trajectory &);
static void plotAccelerations(const Trajectory &);
static void descendObjective(Trajectory &);
static void descendObjectiveConstrained(Trajectory &);
static void minimizeAcceleration1(Trajectory &);
static void minimizeAcceleration2(Trajectory &);

static void fixupConstraint1(Trajectory &);
static void fixupConstraint2(Trajectory &);
static void fixupConstraint3(Trajectory &);
static void fixupConstraint4(Trajectory &);

//----------------------------------------------------------------------------

static HWND g_hWnd = 0;
static HDC g_hDC = 0;
static bool g_active = false;
static int g_size_x = 1;
static int g_size_y = 1;
static int g_mouse_x = 0;
static int g_mouse_y = 0;
static size_t g_highlightedNode = 0;
static size_t g_selectedNode = 0;
static dvec2 g_posOffset(0, 0);
static unsigned g_disc_list = 0;

static Trajectory g_trajectory;

//----------------------------------------------------------------------------

int WINAPI WinMain
(
	HINSTANCE hInstance,
	HINSTANCE hPrevInstance, 
	LPSTR lpCmdLine,
	int iCmdShow
)
{
	// Register window class

	WNDCLASS wc;
	ZeroMemory(&wc, sizeof(wc));
	wc.style = CS_OWNDC;
	wc.lpfnWndProc = WndProc;
	wc.cbClsExtra = 0;
	wc.cbWndExtra = 0;
	wc.hInstance = hInstance;
	wc.hIcon = LoadIcon( NULL, IDI_APPLICATION );
	wc.hCursor = LoadCursor( NULL, IDC_ARROW );
	wc.hbrBackground = (HBRUSH)GetStockObject( BLACK_BRUSH );
	wc.lpszMenuName = NULL;
	wc.lpszClassName = windowClassName;
	RegisterClass(&wc);

	// Calculate the wize of the overall window for the given client area size.
	RECT window_rect = { 0, 0, kInitWindowSizeX, kInitWindowSizeY };
	AdjustWindowRect(&window_rect, kWindowStyle, FALSE);

	int window_size_x = window_rect.right - window_rect.left;
	int window_size_y = window_rect.bottom - window_rect.top;
	int window_pos_x  = (GetSystemMetrics(SM_CXSCREEN) - window_size_x) / 2;
	int window_pos_y  = (GetSystemMetrics(SM_CYSCREEN) - window_size_y) / 2;

	// create main window
	g_hWnd = CreateWindow
	(
		windowClassName, windowName, 
		kWindowStyle,
		window_pos_x, window_pos_y, window_size_x, window_size_y,
		NULL, NULL, hInstance, NULL
	);

	// enable OpenGL for the window
	HGLRC hRC;
	EnableOpenGL(g_hWnd, g_hDC, hRC);

	// Initialize simulation.
	init();

	// program main loop
	g_active = true;
    while (1)
    {
		bool quit = false;
	    MSG msg;
		while (!quit && PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			if (msg.message == WM_QUIT)
				quit = true;
			else
			{
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
		}

		if (quit)
			break;

		WaitMessage();
    }

	// shutdown OpenGL
	DisableOpenGL(g_hWnd, g_hDC, hRC);

	// destroy the window explicitly
//	DestroyWindow(g_hWnd);

	return 0;
}

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
	case WM_CREATE:
		return 0;

	case WM_CLOSE:
		PostQuitMessage(0);
		return 0;

	case WM_DESTROY:
		return 0;

    case WM_SIZE:
        // Check to see if we are losing our window...
        if (SIZE_MAXHIDE == wParam || SIZE_MINIMIZED == wParam)
		{
            g_active = false;
		}
        else
        {
            g_active = true;
			g_size_x = LOWORD(lParam);
			g_size_y = HIWORD(lParam);
			if (g_size_x > 0 && g_size_y > 0)
			{
				glViewport(0, 0, g_size_x, g_size_y);
				repaint();
			}
        }
		return 0;

	case WM_ERASEBKGND:
		return 1;

	case WM_MOUSEMOVE:
		g_mouse_x = int(short(LOWORD(lParam)));
		g_mouse_y = int(short(HIWORD(lParam)));
		if (g_selectedNode < g_trajectory.node.size())
			updateMouseDrag(g_mouse_x, g_mouse_y);
		else
			updateHighlight(g_mouse_x, g_mouse_y);
		repaint();
		return 0;

	case WM_PAINT:
		drawScene();
		ValidateRect(hWnd, NULL);
		return 0;

	case WM_LBUTTONDOWN:
		g_mouse_x = int(short(LOWORD(lParam)));
		g_mouse_y = int(short(HIWORD(lParam)));
		beginMouseDrag(hWnd, g_mouse_x, g_mouse_y);
		return 0;

	case WM_LBUTTONUP:
		endMouseDrag();
		return 0;

	case WM_KEYDOWN:
		switch (wParam)
		{
		case VK_ESCAPE:
			PostQuitMessage(0);
			return 0;

		case VK_SPACE:
			init();
			updateHighlight(g_mouse_x, g_mouse_y);
			repaint();
			return 0;

		case VK_END:
			g_trajectory.segmentDuration[0] -= 0.1;
			repaint();
			return 0;

		case VK_HOME:
			g_trajectory.segmentDuration[0] += 0.1;
			repaint();
			return 0;

		case VK_NEXT:
			g_trajectory.segmentDuration[1] += 0.1;
			repaint();
			return 0;

		case VK_PRIOR:
			g_trajectory.segmentDuration[1] -= 0.1;
			repaint();
			return 0;

		case VK_LEFT:
			g_trajectory.node[1].vel[0] -= 1;
			repaint();
			return 0;

		case VK_RIGHT:
			g_trajectory.node[1].vel[0] += 1;
			repaint();
			return 0;

		case VK_UP:
			g_trajectory.node[1].vel[1] -= 1;
			repaint();
			return 0;

		case VK_DOWN:
			g_trajectory.node[1].vel[1] += 1;
			repaint();
			return 0;

		case 'D':
			descendObjective(g_trajectory);
			repaint();
			return 0;

		case 'C':
			descendObjectiveConstrained(g_trajectory);
			repaint();
			return 0;

		case '1':
			fixupConstraint1(g_trajectory);
			repaint();
			return 0;

		case '2':
			fixupConstraint2(g_trajectory);
			repaint();
			return 0;

		case '3':
			fixupConstraint3(g_trajectory);
			repaint();
			return 0;

		case '4':
			fixupConstraint4(g_trajectory);
			repaint();
			return 0;

		case '5':
			minimizeAcceleration1(g_trajectory);
			repaint();
			return 0;

		case '6':
			minimizeAcceleration2(g_trajectory);
			repaint();
			return 0;
		}
		return 0;
	
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
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

void EnableOpenGL(HWND hWnd, HDC &hDC, HGLRC &hRC)
{
	// get the device context (DC)
	hDC = GetDC(hWnd);
	
	// set the pixel format for the DC
	PIXELFORMATDESCRIPTOR pfd;
	ZeroMemory(&pfd, sizeof(pfd));
	pfd.nSize = sizeof(pfd);
	pfd.nVersion = 1;
	pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
	pfd.iPixelType = PFD_TYPE_RGBA;
	pfd.cColorBits = 24;
	pfd.cDepthBits = 32;
	pfd.iLayerType = PFD_MAIN_PLANE;
	int format = ChoosePixelFormat(hDC, &pfd);
	SetPixelFormat(hDC, format, &pfd);
	
	// create and enable the render context (RC)
	hRC = wglCreateContext(hDC);
	wglMakeCurrent(hDC, hRC);

	// Initialize OpenGL.
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
//	glEnable(GL_POLYGON_SMOOTH);
//	glHint(GL_POLYGON_SMOOTH_HINT, GL_DONT_CARE);

	// Generate a disc display list.
	g_disc_list = glGenLists(1);
	glNewList(g_disc_list, GL_COMPILE);
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
}

void DisableOpenGL(HWND hWnd, HDC hDC, HGLRC hRC)
{
	wglMakeCurrent(NULL, NULL);
	wglDeleteContext(hRC);
	ReleaseDC(hWnd, hDC);
}

void init()
{
	g_trajectory.node.clear();

	g_trajectory.node.reserve(10);
//	g_trajectory.node.push_back({ dvec2(0, -200), dvec2(0, 0) });
//	g_trajectory.node.push_back({ dvec2(0, 0), dvec2(85, 85) });
//	g_trajectory.node.push_back({ dvec2(100, 0), dvec2(0, 0) });
	g_trajectory.node.push_back({ dvec2(-200, -150), dvec2(0, 0) });
	g_trajectory.node.push_back({ dvec2(0, -150), dvec2(100, 0) });
	g_trajectory.node.push_back({ dvec2(200, -150), dvec2(0, 0) });

	g_trajectory.segmentDuration.clear();
	g_trajectory.segmentDuration.reserve(9);
	g_trajectory.segmentDuration.push_back(3);
	g_trajectory.segmentDuration.push_back(3);

	// Nothing selected, initially

	g_highlightedNode = g_trajectory.node.size();
	g_selectedNode = g_trajectory.node.size();
}

static void drawDisc()
{
	glCallList(g_disc_list);
}

static void projectionMatrix(double m[16])
{
	double rx = g_size_x;
	double ry = g_size_y;
	double rz = 500;

	memset(m, 0, 16*sizeof(double));
	m[0] = 2.0 / rx;
	m[5] = 2.0 / ry;
	m[10] = -1.0 / rz;
	m[15] = 1;
}

static void modelviewMatrix(double m[16])
{
	memset(m, 0, 16*sizeof(double));
	m[0] = 1;
	m[5] = 1;
	m[10] = 1;
	m[15] = 1;
}

inline double sqr(double x)
{
	return x * x;
}

inline double cube(double x)
{
	return x * x * x;
}

static void plotAcceleration(const Trajectory & traj)
{
	double tTotal = 0;
	for (double t : traj.segmentDuration)
		tTotal += t;

	double aMax = 1;

	for (size_t i = 0; i < traj.segmentDuration.size(); ++i)
	{
		dvec2 x0 = traj.node[i].pos;
		dvec2 x1 = traj.node[i + 1].pos;
		dvec2 v0 = traj.node[i].vel;
		dvec2 v1 = traj.node[i + 1].vel;
		double h = traj.segmentDuration[i];

		dvec2 a0 = x0 * (-6.0 / sqr(h)) + x1 * (6.0 / sqr(h)) + v0 * (-4.0 / h) + v1 * (-2.0 / h);
		dvec2 a1 = x0 * (6.0 / sqr(h)) + x1 * (-6.0 / sqr(h)) + v0 * (2.0 / h) + v1 * (4.0 / h);

		aMax = std::max(aMax, a0.sqlen());
		aMax = std::max(aMax, a1.sqlen());
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

	double u0 = 0;

	for (size_t i = 0; i < traj.segmentDuration.size() - 1; ++i)
	{
		double h = traj.segmentDuration[i];

		u0 += h / tTotal;

		glVertex2d(u0, 0);
		glVertex2d(u0, 1);
	}

	{
		glColor3d(0.5, 0.1, 0.1);

		double y = sqr(accelerationLimit) / aMax;

		glVertex2d(0, y);
		glVertex2d(1, y);
	}

	glEnd();

	u0 = 0;
	for (size_t i = 0; i < traj.segmentDuration.size(); ++i)
	{
		if (i == 0)
			glColor3d(1, 1, 0);
		else
			glColor3d(0, 1, 1);

		dvec2 x0 = traj.node[i].pos;
		dvec2 x1 = traj.node[i + 1].pos;
		dvec2 v0 = traj.node[i].vel;
		dvec2 v1 = traj.node[i + 1].vel;
		double h = traj.segmentDuration[i];

		dvec2 a0 = x0 * (-6.0 / sqr(h)) + x1 * (6.0 / sqr(h)) + v0 * (-4.0 / h) + v1 * (-2.0 / h);
		dvec2 a1 = x0 * (6.0 / sqr(h)) + x1 * (-6.0 / sqr(h)) + v0 * (2.0 / h) + v1 * (4.0 / h);

		double u1 = u0 + h / tTotal;

		glBegin(GL_LINE_STRIP);

		for (size_t j = 0; j < 48; ++j)
		{
			double t = double(j) / 48.0;

			dvec2 a = a0 + (a1 - a0) * t;
			double u = u0 + (u1 - u0) * t;

			glVertex2d(u, a.sqlen() / aMax);
		}

		glVertex2d(u1, a1.sqlen() / aMax);

		glEnd();

		u0 = u1;
	}
}

void drawScene()
{
	glClear(GL_COLOR_BUFFER_BIT);

	// Plot trajectory

	double m[16];

	glMatrixMode(GL_PROJECTION);
	projectionMatrix(m);
	glLoadMatrixd(m);

	glMatrixMode(GL_MODELVIEW);
	modelviewMatrix(m);
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

	SwapBuffers(g_hDC);
}

void beginMouseDrag(HWND hWnd, int mouseX, int mouseY)
{
	g_selectedNode = g_highlightedNode;

	if (g_selectedNode < g_trajectory.node.size())
	{
		g_posOffset = g_trajectory.node[g_selectedNode].pos - posMouseWorld(mouseX, mouseY);
	}

	SetCapture(hWnd);
}

void endMouseDrag()
{
	g_selectedNode = g_trajectory.node.size();

	ReleaseCapture();
	updateHighlight(g_mouse_x, g_mouse_y);
}

dvec2 posMouseWorld(int mouseX, int mouseY)
{
	int viewport[4] = { 0, 0, g_size_x, g_size_y };
	double mvmatrix[16], projmatrix[16];
	projectionMatrix(projmatrix);
	modelviewMatrix(mvmatrix);

	mouseY = viewport[3] - mouseY - 1;

	dvec3 pos;
	gluUnProject(mouseX, mouseY, 0, mvmatrix, projmatrix, viewport, &pos[0], &pos[1], &pos[2]);

	return dvec2(pos[0], pos[1]);
}

void updateMouseDrag(int mouseX, int mouseY)
{
	g_trajectory.node[g_selectedNode].pos = g_posOffset + posMouseWorld(mouseX, mouseY);
}

size_t closestNode(int mouse_x, int mouse_y)
{
	dvec2 posMouse = posMouseWorld(mouse_x, mouse_y);

	const double sqR = discRadius*discRadius;

	size_t iClosest = g_trajectory.node.size();
	double closestSqDist = std::numeric_limits<double>::infinity();
	for (size_t i = 0; i < g_trajectory.node.size(); ++i)
	{
		double sqDist = (g_trajectory.node[i].pos - posMouse).sqlen();
		if (sqDist < sqR && (iClosest >= g_trajectory.node.size() || sqDist < closestSqDist))
		{
			iClosest = i;
			closestSqDist = sqDist;
		}
	}

	return iClosest;
}

void repaint()
{
	InvalidateRect(g_hWnd, NULL, FALSE);
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
	const double h0 = traj.segmentDuration[0];
	const double h1 = traj.segmentDuration[1];
	const dvec2 p0 = traj.node[0].pos;
	const dvec2 v0 = traj.node[0].vel;
	const dvec2 p1 = traj.node[1].pos;
	const dvec2 v1 = traj.node[1].vel;
	const dvec2 p2 = traj.node[2].pos;
	const dvec2 v2 = traj.node[2].vel;
	const dvec2 dPos0 = p1 - p0;
	const dvec2 dPos1 = p2 - p1;

	dvec2 a0 = (dPos0 * ( 6.0 / sqr(h0)) + v0 * -4.0 / h0 + v1 * -2.0 / h0) / aMax;
	dvec2 a1 = (dPos0 * (-6.0 / sqr(h0)) + v0 *  2.0 / h0 + v1 *  4.0 / h0) / aMax;
	dvec2 a2 = (dPos1 * ( 6.0 / sqr(h1)) + v1 * -4.0 / h1 + v2 * -2.0 / h1) / aMax;
	dvec2 a3 = (dPos1 * (-6.0 / sqr(h1)) + v1 *  2.0 / h1 + v2 *  4.0 / h1) / aMax;

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

void plotTrajectory(const Trajectory & traj)
{
	// Draw curve nodes

	for (size_t i = 0; i < traj.node.size(); ++i)
	{
		const NodeState & s = traj.node[i];

		if (i == g_highlightedNode)
			glColor3d(1, 1, 1);
		else
			glColor3d(0.65, 0.65, 0.65);

		glPushMatrix();
		glTranslated(s.pos[0], s.pos[1], 0.0);
		glScaled(discRadius, discRadius, 1.0);
		drawDisc();
		glPopMatrix();
	}

	// Draw the curve

	for (size_t i = 0; i < traj.segmentDuration.size(); ++i)
	{
		if (i == 0)
			glColor3d(1, 1, 0);
		else
			glColor3d(0, 1, 1);

		double h = traj.segmentDuration[i];
		dvec2 x0 = traj.node[i].pos;
		dvec2 x1 = traj.node[i + 1].pos;
		dvec2 v0 = traj.node[i].vel;
		dvec2 v1 = traj.node[i + 1].vel;

		dvec2 acc0 = (x1 - x0) * (6.0 / sqr(h)) - (v0 * 4.0 + v1 * 2.0) / h;
		dvec2 jrk0 = (v1 - v0) * (2.0 / sqr(h)) - acc0 * (2.0 / h);

		// Evaluate the segment position

		glBegin(GL_LINE_STRIP);

		glVertex2dv(&x0[0]);

		for (size_t j = 1; j < 16; ++j)
		{
			double t = h * double(j) / 16.0;

			dvec2 pos = x0 + (v0 + (acc0 + jrk0 * (t / 3.0f)) * (t / 2.0f)) * t;

			glVertex2dv(&pos[0]);
		}

		glVertex2dv(&x1[0]);

		glEnd();
	}
}

void descendObjective(Trajectory & traj)
{
	traj.segmentDuration[0] *= 0.75;
	traj.segmentDuration[1] *= 0.75;
}

void descendObjectiveConstrained(Trajectory & traj)
{
	// Reduce h0 (traj.segmentDuration[0]) to minimum that satisfies both constraints 1 and 2
	// 1: aMax^2 - sqlen((x1 - x0) *  6/h0^2 + v0 * -4/h0 + v1 * -2/h0) >= 0
	// 2: aMax^2 - sqlen((x1 - x0) * -6/h0^2 + v0 *  2/h0 + v1 *  4/h0) >= 0

	// 1: aMax^2 - ((x1 - x0) *  6/h0^2 + vx0 * -4/h0 + vx1 * -2/h0)^2 - ((y1 - y0) *  6/h0^2 + vy0 * -4/h0 + vy1 * -2/h0)^2 >= 0

	const double aMax = accelerationLimit;
	const double h0 = traj.segmentDuration[0];
	const dvec2 p0 = traj.node[0].pos;
	const dvec2 v0 = traj.node[0].vel;
	const dvec2 p1 = traj.node[1].pos;
	const dvec2 v1 = traj.node[1].vel;
	const dvec2 dPos = p1 - p0;

	dvec2 a0Vec = dPos * (6.0 / sqr(h0)) + v0 * -4.0 / h0 + v1 * -2.0 / h0;
	double a0 = a0Vec.len();

	double a0Excess = a0 - aMax;

	dvec2 a1Vec = dPos * (-6.0 / sqr(h0)) + v0 * 2.0 / h0 + v1 * 4.0 / h0;
	double a1 = a1Vec.len();

	double a1Excess = a1 - aMax;

	if (a0Excess <= 0)
		return;

	dvec2 x = dPos * (-12.0 / cube(h0)) + v0 * (4.0 / sqr(h0)) + v1 * (2.0 / sqr(h0));
	double dA_dH0 = x.dot(a0Vec) / a0;
	double dA_dVX = (-2.0 * a0Vec[0]) / (a0 * h0);
	double dA_dVY = (-2.0 * a0Vec[1]) / (a0 * h0);

	double u = a0Excess / (sqr(dA_dH0) + sqr(dA_dVX) + sqr(dA_dVY));

	traj.segmentDuration[0] -= dA_dH0 * u;
}

void minimizeAcceleration1(Trajectory & traj)
{
	const double h0 = traj.segmentDuration[0];
	const double h1 = traj.segmentDuration[1];
	const double x0 = traj.node[0].pos[0];
	const double v0 = traj.node[0].vel[0];
	const double x1 = traj.node[1].pos[0];
	const double v1 = traj.node[1].vel[0];
	const double x2 = traj.node[2].pos[0];
	const double v2 = traj.node[2].vel[0];

	double v1n = -0.8 * (v0 * sqr(h1) + v1 * sqr(h0)) + 1.8 * ((x1 - x0) * sqr(h1) / h0 + (x2 - x1) * sqr(h0) / h1);
	v1n /= sqr(h0) + sqr(h1);

	traj.node[1].vel[0] = v1n;
}

void minimizeAcceleration2(Trajectory & traj)
{
	const double h0 = traj.segmentDuration[0];
	const double h1 = traj.segmentDuration[1];
	const double x0 = traj.node[0].pos[1];
	const double v0 = traj.node[0].vel[1];
	const double x1 = traj.node[1].pos[1];
	const double v1 = traj.node[1].vel[1];
	const double x2 = traj.node[2].pos[1];
	const double v2 = traj.node[2].vel[1];

	double v1n = -0.8 * (v0 * sqr(h1) + v1 * sqr(h0)) + 1.8 * ((x1 - x0) * sqr(h1) / h0 + (x2 - x1) * sqr(h0) / h1);
	v1n /= sqr(h0) + sqr(h1);

	traj.node[1].vel[1] = v1n;
}

void fixupConstraint1(Trajectory & traj)
{
	// Constraint a1: aMax^2 - sqlen(x0 * -6/h0^2 + x1 * 6/h0^2 + v0 * -4/h0 + v1 * -2/h0) >= 0

	const double aMax = accelerationLimit;
	const double h0 = traj.segmentDuration[0];
	const dvec2 p0 = traj.node[0].pos;
	const dvec2 v0 = traj.node[0].vel;
	const dvec2 p1 = traj.node[1].pos;
	const dvec2 v1 = traj.node[1].vel;
	const dvec2 dPos = p1 - p0;

	dvec2 aVec = dPos * (6.0 / sqr(h0)) + v0 * -4.0 / h0 + v1 * -2.0 / h0;
	double a = aVec.len();

	double aExcess = a - aMax;
	if (aExcess <= 0)
		return;

	dvec2 x = dPos * (-12.0 / cube(h0)) + v0 * (4.0 / sqr(h0)) + v1 * (2.0 / sqr(h0));
	double dA_dH0 = x.dot(aVec) / a;
	double dA_dVX = (-2.0 * aVec[0]) / (a * h0);
	double dA_dVY = (-2.0 * aVec[1]) / (a * h0);

	double u = aExcess / (sqr(dA_dH0) + sqr(dA_dVX) + sqr(dA_dVY));

	traj.segmentDuration[0] -= dA_dH0 * u;
	traj.node[1].vel[0] -= dA_dVX * u;
	traj.node[1].vel[1] -= dA_dVY * u;
}

void fixupConstraint2(Trajectory & traj)
{
	const double aMax = accelerationLimit;
	const double h0 = traj.segmentDuration[0];
	const dvec2 p0 = traj.node[0].pos;
	const dvec2 v0 = traj.node[0].vel;
	const dvec2 p1 = traj.node[1].pos;
	const dvec2 v1 = traj.node[1].vel;
	const dvec2 dPos = p1 - p0;

	dvec2 aVec = dPos * (-6.0 / sqr(h0)) + v0 * 2.0 / h0 + v1 * 4.0 / h0;
	double a = aVec.len();

	double aExcess = a - aMax;
	if (aExcess <= 0)
		return;

	dvec2 x = dPos * (12.0 / cube(h0)) + v0 * (-2.0 / sqr(h0)) + v1 * (-4.0 / sqr(h0));
	double dA_dH0 = x.dot(aVec) / a;
	double dA_dVX = (4.0 * aVec[0]) / (a * h0);
	double dA_dVY = (4.0 * aVec[1]) / (a * h0);

	double u = aExcess / (sqr(dA_dH0) + sqr(dA_dVX) + sqr(dA_dVY));

	traj.segmentDuration[0] -= dA_dH0 * u;
	traj.node[1].vel[0] -= dA_dVX * u;
	traj.node[1].vel[1] -= dA_dVY * u;
}

void fixupConstraint3(Trajectory & traj)
{
	const double aMax = accelerationLimit;
	const double h1 = traj.segmentDuration[1];
	const dvec2 p1 = traj.node[1].pos;
	const dvec2 v1 = traj.node[1].vel;
	const dvec2 p2 = traj.node[2].pos;
	const dvec2 v2 = traj.node[2].vel;
	const dvec2 dPos = p2 - p1;

	dvec2 aVec = dPos * (6.0 / sqr(h1)) + v1 * -4.0 / h1 + v2 * -2.0 / h1;
	double a = aVec.len();

	double aExcess = a - aMax;
	if (aExcess <= 0)
		return;

	dvec2 x = dPos * (-12.0 / cube(h1)) + v1 * (4.0 / sqr(h1)) + v2 * (2.0 / sqr(h1));
	double dA_dH1 = x.dot(aVec) / a;
	double dA_dVX = (-4.0 * aVec[0]) / (a * h1);
	double dA_dVY = (-4.0 * aVec[1]) / (a * h1);

	double u = aExcess / (sqr(dA_dH1) + sqr(dA_dVX) + sqr(dA_dVY));

	traj.segmentDuration[1] -= dA_dH1 * u;
	traj.node[1].vel[0] -= dA_dVX * u;
	traj.node[1].vel[1] -= dA_dVY * u;
}

void fixupConstraint4(Trajectory & traj)
{
	const double aMax = accelerationLimit;
	const double h1 = traj.segmentDuration[1];
	const dvec2 p1 = traj.node[1].pos;
	const dvec2 v1 = traj.node[1].vel;
	const dvec2 p2 = traj.node[2].pos;
	const dvec2 v2 = traj.node[2].vel;
	const dvec2 dPos = p2 - p1;

	dvec2 aVec = dPos * (-6.0 / sqr(h1)) + v1 * 2.0 / h1 + v2 * 4.0 / h1;
	double a = aVec.len();

	double aExcess = a - aMax;
	if (aExcess <= 0)
		return;

	dvec2 x = dPos * (12.0 / cube(h1)) + v1 * (-2.0 / sqr(h1)) + v2 * (-4.0 / sqr(h1));
	double dA_dH1 = x.dot(aVec) / a;
	double dA_dVX = (2.0 * aVec[0]) / (a * h1);
	double dA_dVY = (2.0 * aVec[1]) / (a * h1);

	double u = aExcess / (sqr(dA_dH1) + sqr(dA_dVX) + sqr(dA_dVY));

	traj.segmentDuration[1] -= dA_dH1 * u;
	traj.node[1].vel[0] -= dA_dVX * u;
	traj.node[1].vel[1] -= dA_dVY * u;
}
