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

//----------------------------------------------------------------------------

const int kInitWindowSizeX = 800;
const int kInitWindowSizeY = 800;
const unsigned kWindowStyle = WS_OVERLAPPEDWINDOW | WS_VISIBLE;

const char * windowClassName = "Rocket Path";
const char * windowName = "Rocket Path";

const double pi = 3.1415926535897932384626433832795;

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

//----------------------------------------------------------------------------

static HWND g_hWnd = 0;
static HDC g_hDC = 0;
static bool g_active = false;
static int g_size_x = 1;
static int g_size_y = 1;
static int g_mouse_x = 0;
static int g_mouse_y = 0;
static double g_au_per_pixel = 0.1;
static size_t g_highlightedNode = 0;
static size_t g_selectedNode = 0;
static dvec2 g_posOffset(0, 0);
static unsigned g_disc_list = 0;

static std::vector<NodeState> g_nodes;

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
		if (g_selectedNode < g_nodes.size())
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
//	glEnable(GL_LINE_SMOOTH);
//	glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
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

	// Generate font display lists.

	// make the system font the device context's selected font 
	SelectObject (g_hDC, GetStockObject (SYSTEM_FONT)); 
	 
	// create the bitmap display lists 
	// we're making images of glyphs 0 thru 255 
	// the display list numbering starts at 1000, an arbitrary choice 
	wglUseFontBitmaps (g_hDC, 0, 255, 1000); 
	glListBase(1000); 
}

void DisableOpenGL(HWND hWnd, HDC hDC, HGLRC hRC)
{
	wglMakeCurrent(NULL, NULL);
	wglDeleteContext(hRC);
	ReleaseDC(hWnd, hDC);
}

void init()
{
	g_nodes.clear();
	g_nodes.reserve(10);
	g_nodes.push_back({ dvec2(-10, 0), dvec2(0, 0) });
	g_nodes.push_back({ dvec2(0, 0), dvec2(1, 0) });
	g_nodes.push_back({ dvec2(10, 0), dvec2(0, 0) });

	// Nothing selected, initially

	g_highlightedNode = g_nodes.size();
	g_selectedNode = g_nodes.size();
}

static void drawDisc()
{
	glCallList(g_disc_list);
}

static void projectionMatrix(double m[16])
{
	double rx = g_size_x * g_au_per_pixel;
	double ry = g_size_y * g_au_per_pixel;
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

void drawScene()
{
	glClear(GL_COLOR_BUFFER_BIT);

	double m[16];

	glMatrixMode(GL_PROJECTION);
	projectionMatrix(m);
	glLoadMatrixd(m);

	glMatrixMode(GL_MODELVIEW);
	modelviewMatrix(m);
	glLoadMatrixd(m);

	// Draw curve nodes

	for (size_t i = 0; i < g_nodes.size(); ++i)
	{
		const NodeState & s = g_nodes[i];

		if (i == g_highlightedNode)
			glColor3d(1, 1, 1);
		else
			glColor3d(0.65, 0.65, 0.65);

		const double r = 1.0;
		glPushMatrix();
		glTranslated(s.pos[0], s.pos[1], 0.0);
		glScaled(r, r, 1.0);
		drawDisc();
		glPopMatrix();
	}

	// Draw the curve

	glColor3d(1, 1, 1);
	glBegin(GL_LINES);
	for (size_t i = 1; i < g_nodes.size(); ++i)
	{
		glVertex2dv(&g_nodes[i - 1].pos[0]);
		glVertex2dv(&g_nodes[i].pos[0]);
	}
	glEnd();

	SwapBuffers(g_hDC);
}

static void beginMouseDrag(HWND hWnd, int mouseX, int mouseY)
{
	g_selectedNode = g_highlightedNode;

	if (g_selectedNode < g_nodes.size())
	{
		g_posOffset = g_nodes[g_selectedNode].pos - posMouseWorld(mouseX, mouseY);
	}

	SetCapture(hWnd);
}

static void endMouseDrag()
{
	g_selectedNode = g_nodes.size();

	ReleaseCapture();
	updateHighlight(g_mouse_x, g_mouse_y);
}

static dvec2 posMouseWorld(int mouseX, int mouseY)
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

static void updateMouseDrag(int mouseX, int mouseY)
{
	g_nodes[g_selectedNode].pos = g_posOffset + posMouseWorld(mouseX, mouseY);
}

static size_t closestNode(int mouse_x, int mouse_y)
{
	dvec2 posMouse = posMouseWorld(mouse_x, mouse_y);

	const double r = 1.0;
	const double sqR = r*r;

	size_t iClosest = g_nodes.size();
	double closestSqDist = std::numeric_limits<double>::infinity();
	for (size_t i = 0; i < g_nodes.size(); ++i)
	{
		double sqDist = (g_nodes[i].pos - posMouse).sqlen();
		if (sqDist < sqR && (iClosest >= g_nodes.size() || sqDist < closestSqDist))
		{
			iClosest = i;
			closestSqDist = sqDist;
		}
	}

	return iClosest;
}

static void repaint()
{
	InvalidateRect(g_hWnd, NULL, FALSE);
}
