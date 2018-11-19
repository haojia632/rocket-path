#include <windows.h>
#include <gl/gl.h>

#include <cmath>
#include <stdio.h>

#include "draw.h"
#include "fixptpath.h"
#include "onedpath.h"
#include "onedpath_ip.h"
#include "onedpath2_ip.h"

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
static void dumpImage();
static void dumpClient(HWND hWnd, const char * filename);

//----------------------------------------------------------------------------

static HWND g_hWnd = 0;
static HDC g_hDC = 0;
static bool g_active = false;
static int g_size_x = 1;
static int g_size_y = 1;
static unsigned g_disc_list = 0;
static int g_imageIndex = 0;

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

	// Calculate the size of the overall window for the given client area size.
	RECT window_rect = { 0, 0, kInitWindowSizeX, kInitWindowSizeY };
	AdjustWindowRect(&window_rect, kWindowStyle, FALSE);

	int window_size_x = window_rect.right - window_rect.left;
	int window_size_y = window_rect.bottom - window_rect.top;
	int window_pos_x = 16; // (GetSystemMetrics(SM_CXSCREEN) - window_size_x) / 2;
	int window_pos_y = (GetSystemMetrics(SM_CYSCREEN) - window_size_y) / 2;

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
	for (Problem * problem : g_problems)
	{
		problem->init();
	}

	{
		POINT posMouse = { 0, 0 };
		GetCursorPos(&posMouse);
		g_problemCur->onMouseMove(posMouse.x, posMouse.y);
	}

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
		{
			int x = int(short(LOWORD(lParam)));
			int y = int(short(HIWORD(lParam)));
			g_problemCur->onMouseMove(x, y);
			repaint();
		}
		return 0;

	case WM_PAINT:
		g_problemCur->onDraw();
		SwapBuffers(g_hDC);
		ValidateRect(hWnd, NULL);
		return 0;

	case WM_LBUTTONDOWN:
		{
			int x = int(short(LOWORD(lParam)));
			int y = int(short(HIWORD(lParam)));
			g_problemCur->onMouseMove(x, y);
			g_problemCur->onMouseDown();
			SetCapture(hWnd);
		}
		return 0;

	case WM_LBUTTONUP:
		{
			g_problemCur->onMouseUp();
			ReleaseCapture();
		}
		return 0;

	case WM_KEYDOWN:
		if (wParam == VK_ESCAPE)
		{
			PostQuitMessage(0);
		}
		else if (wParam == VK_F2)
		{
			dumpImage();
		}
		else
		{
			g_problemCur->onKey(wParam);
		}
		return 0;
	
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
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

void drawDisc()
{
	glCallList(g_disc_list);
}

int windowSizeX()
{
	return g_size_x;
}

int windowSizeY()
{
	return g_size_y;
}

void repaint()
{
	InvalidateRect(g_hWnd, NULL, FALSE);
}

void debug_printf(const char * fmt, ...)
{
	va_list args;
	va_start(args, fmt);

	char buffer[512];

	vsprintf_s(buffer, sizeof(buffer), fmt, args);

	va_end(args);

	OutputDebugString(buffer);
}

void dumpImage()
{
	char filename[32];
	sprintf_s(filename, sizeof(filename), "image%04d.bmp", g_imageIndex);
	++g_imageIndex;

	dumpClient(g_hWnd, filename);

	debug_printf("Wrote %s\n", filename);
}

void dumpClient(HWND hWnd, const char * filename)
{
	// Get the window client rectangle.
	RECT rc;
	GetClientRect(hWnd, &rc);
	ClientToScreen(hWnd, (POINT*)&rc.left);
	ClientToScreen(hWnd, (POINT*)&rc.right);

	int width = rc.right - rc.left;
	int height = rc.bottom - rc.top;

	// Get the screen DC
	HDC hDCScreen = GetDC(NULL);

	// Create a memory HDC and HBITMAP for the window
	HDC hDCMem = CreateCompatibleDC(hDCScreen);
	HBITMAP hBmMem = CreateCompatibleBitmap(hDCScreen, width, height);
	HBITMAP hBmMemOld = (HBITMAP)SelectObject(hDCMem, hBmMem);

	// Blt the window to the memory DC
	BitBlt(hDCMem, 0, 0, width, height, hDCScreen, rc.left, rc.top, SRCCOPY);

	// Convert the HBITMAP into a DIB
	BITMAPINFO bi;
	bi.bmiHeader.biSize = sizeof(BITMAPINFO);
	bi.bmiHeader.biWidth = width;
	bi.bmiHeader.biHeight = height;
	bi.bmiHeader.biPlanes = 1;
	bi.bmiHeader.biBitCount = 24;
	bi.bmiHeader.biCompression = BI_RGB;
	bi.bmiHeader.biSizeImage = 0;
	bi.bmiHeader.biXPelsPerMeter = 0;
	bi.bmiHeader.biYPelsPerMeter = 0;
	bi.bmiHeader.biClrUsed = 0;
	bi.bmiHeader.biClrImportant = 0;
	GetDIBits(hDCMem, hBmMem, 0, height, NULL, &bi, DIB_RGB_COLORS);

	// GetDIBits filled in bi.bmiHeader.biSizeImage, allocate memory and now get the DIB bits
	void * data = malloc(bi.bmiHeader.biSizeImage);
	GetDIBits(hDCMem, hBmMem, 0, height, data, &bi, DIB_RGB_COLORS);

	// Now save into disk file.
	FILE * fp;
	if (0 != fopen_s(&fp, filename, "wb"))
	{
		free(data);
		return;
	}

	// First, the bitmap file header
	BITMAPFILEHEADER bmfh;
	bmfh.bfType = *(WORD*)"BM";
	bmfh.bfSize = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER) + bi.bmiHeader.biSizeImage;
	bmfh.bfReserved1 = 0;
	bmfh.bfReserved2 = 0;
	bmfh.bfOffBits = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);

	fwrite(&bmfh, sizeof(BITMAPFILEHEADER), 1, fp);

	// Next, the bitmap info header
	fwrite(&bi.bmiHeader, sizeof(BITMAPINFOHEADER), 1, fp);

	// Finally, the data itself
	fwrite(data, sizeof(BYTE), bi.bmiHeader.biSizeImage, fp);

	fclose(fp);

	// Cleanup
	free(data);
	SelectObject(hDCMem, hBmMemOld);
	DeleteDC(hDCMem);
	DeleteObject(hBmMem);
	ReleaseDC(NULL, hDCScreen);
}
