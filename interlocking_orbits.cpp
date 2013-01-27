#include <windows.h>
#include <math.h>

#include <string>
#include <fstream>
#include <list>
#include <limits>
#include <vector>

#include <gl/gl.h>
#include <gl/glu.h>

#include "vec.h"

#undef min
#undef max

struct tOrbitElements
{
	// Shape and size:
	double a; // half of the ellipse in the long direction (astronomical units)
	double b; // half of the ellipse in the short direction (astronomical units)
	double e; // eccentricity (0 <= e < 1): 0 = circle, 0-1 = ellipse, 1 = parabola

	// Phase:
	int period;
	double t0; // fraction of orbit completed at time=0 (0 <= t0 < 1)

	// Orientation
	double w; // Rotate about Z axis by w, the argument of periapsis (radians)
	double cos_w;
	double sin_w;
};

struct tPlanet
{
	std::string name;
	dvec3 color;
	double radius;
	tOrbitElements orbit;
	bool visited;

	dvec2 start_position;
	dvec2 position;
};

//----------------------------------------------------------------------------

const double pi = 3.1415926535897932384626433832795;
const double two_pi = 6.283185307179586476925286766559;
const double kEpsilon = 1.0e-8;

const double days_per_time_unit = 10.0;

const double kSunRadius = 2.0;//0.2; // AU
const double kZoomFactor = 1.12;
const double kWheelZoomFactor = 1.25;
const double kPeriodScale = two_pi * 8.0;

const int kInitWindowSizeX = 800;
const int kInitWindowSizeY = 800;
const unsigned kWindowStyle = WS_OVERLAPPEDWINDOW | WS_VISIBLE;

//----------------------------------------------------------------------------

static LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
static void EnableOpenGL(HWND, HDC &, HGLRC &);
static void DisableOpenGL(HWND, HDC, HGLRC);
static void draw_scene();
static void create_planets();
static void init_simulation();
static void begin_mouse_drag(HWND hWnd);
static void end_mouse_drag();
static void update_mouse_drag();
static void travel();
static void draw_orbit(const tPlanet &, double color_scale);
static void draw_planet(const tPlanet &, dvec2 pos, double color_scale, bool selected);
static void draw_trajectory(dvec2 pos0, dvec2 vel0, double dt);
static tPlanet *closest_planet(int mouse_x, int mouse_y);
static void update_highlight();
static void update_planet_pos(const tOrbitElements & orbit, double t, dvec2 &pos);
static void dump_planets(const char * file);

static double eccentric_anomaly(double M, double e);
static double eccentric_anomaly(int mouse_x, int mouse_y, const tOrbitElements & orbit);

static void set_time(double t);
static void advance_time(double dt);
static void repaint();

static void lambert
(
	const dvec2 &start_pos,
	const dvec2 &end_pos,
	double time_of_flight,
	bool long_way,

	dvec2 &start_vel,
	dvec2 &end_vel
);

static void elements_to_state
(
	// In:
	const tOrbitElements &orbit,
	double t, // current time, in time units

	// Out:
	dvec2 &position,
	dvec2 &velocity
);

static void state_to_elements
(
	// In:
	const dvec2 &position, // astronomical units
	const dvec2 &velocity, // astronomical units per time unit

	// Out:
	tOrbitElements &orbit
);

//----------------------------------------------------------------------------

inline double deg2rad(double deg)
{
	return deg * pi / 180;
}

inline double rad2deg(double rad)
{
	return rad * 180 / pi;
}

//----------------------------------------------------------------------------

static HWND g_hWnd = 0;
static HDC g_hDC = 0;
static bool g_active = false;
static int g_size_x = 1;
static int g_size_y = 1;
static int g_mouse_x = 0;
static int g_mouse_y = 0;
static double g_au_per_pixel = 0.1;
static double g_start_time = 0;
static double g_time = 0;
static std::list<tPlanet> g_planets;
static tPlanet *g_location_planet = NULL;
static tPlanet *g_highlighted_planet = NULL;
static tPlanet *g_selected_planet = NULL;
static double g_mean_anomaly_offset = 0.0;
static double g_dv_cur = 0.0;
static double g_dv_max = 0.3;
static unsigned g_disc_list = 0;

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
	wc.lpszClassName = "Interlocking Orbits";
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
		"Interlocking Orbits", "Interlocking Orbits", 
		kWindowStyle,
		window_pos_x, window_pos_y, window_size_x, window_size_y,
		NULL, NULL, hInstance, NULL
	);

	// enable OpenGL for the window
	HGLRC hRC;
	EnableOpenGL(g_hWnd, g_hDC, hRC);

	// Initialize simulation.
	init_simulation();

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

//----------------------------------------------------------------------------

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
		if (g_selected_planet)
			update_mouse_drag();
		else
			update_highlight();
		repaint();
		return 0;

	case WM_PAINT:
		draw_scene();
		ValidateRect(hWnd, NULL);
		return 0;

	case WM_MOUSEWHEEL:
		g_au_per_pixel /= pow(kWheelZoomFactor, double(GET_WHEEL_DELTA_WPARAM(wParam)) / double(WHEEL_DELTA));
		update_highlight();
		repaint();
		return 0;

	case WM_LBUTTONDOWN:
		begin_mouse_drag(hWnd);
		return 0;

	case WM_LBUTTONUP:
		end_mouse_drag();
		return 0;

	case WM_RBUTTONDOWN:
		travel();
		return 0;

	case WM_KEYDOWN:
		switch (wParam)
		{
		case VK_ESCAPE:
			PostQuitMessage(0);
			return 0;
		case 'X':
			g_au_per_pixel /= kZoomFactor;
			update_highlight();
			repaint();
			return 0;
		case 'Z':
			g_au_per_pixel *= kZoomFactor;
			update_highlight();
			repaint();
			return 0;
		case 'R':
			set_time(g_start_time);
			update_highlight();
			repaint();
			return 0;
		case 'D':
			dump_planets("planets.txt");
			return 0;
		case VK_SPACE:
			init_simulation();
			update_highlight();
			repaint();
			return 0;
		}
		return 0;
	
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
}

//----------------------------------------------------------------------------

void update_highlight()
{
	tPlanet * highlighted_planet = closest_planet(g_mouse_x, g_mouse_y);
	if (highlighted_planet != g_highlighted_planet)
	{
		g_highlighted_planet = highlighted_planet;
		repaint();
	}
}

//----------------------------------------------------------------------------

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

	// Generate a planet display list.
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

//----------------------------------------------------------------------------

void DisableOpenGL(HWND hWnd, HDC hDC, HGLRC hRC)
{
	wglMakeCurrent(NULL, NULL);
	wglDeleteContext(hRC);
	ReleaseDC(hWnd, hDC);
}

//----------------------------------------------------------------------------

void init_simulation()
{
	g_highlighted_planet = NULL;
	g_selected_planet = NULL;

	srand(timeGetTime());

	// Reset time
	g_start_time = 0.0;
	g_time = 0.0;

	// Generate planets
	create_planets();

	// Compute planet positions.
	for (std::list<tPlanet>::iterator p = g_planets.begin(); p != g_planets.end(); ++p)
	{
		update_planet_pos(p->orbit, g_time, p->position);
		p->start_position = p->position;
	}

	// Put the rocket at one of the planets.
	std::list<tPlanet>::iterator p = g_planets.begin();
	std::advance(p, g_planets.size() / 2);
	p->visited = true;
	g_location_planet = &(*p);
}

//----------------------------------------------------------------------------

double frand()
{
	return double(rand()) / double(RAND_MAX);
}

//----------------------------------------------------------------------------

static const char * initial_consonant[] =
{
	"B",
	"Br",
	"C",
	"Ch",
	"Cr",
	"D",
	"Dr",
	"F",
	"Fr",
	"G",
	"Gr",
	"H",
	"J",
	"K",
	"L",
	"M",
	"N",
	"P",
	"Pr",
	"R",
	"S",
	"Sh",
	"St",
	"T",
	"Th",
	"Tr",
	"V",
	"W",
	"Wr",
	"Z",
};
static const int num_initial_consonants = sizeof(initial_consonant) / sizeof(initial_consonant[0]);

static const char * initial_vowel[] =
{
	"A",
	"E",
	"I",
	"O",
	"U",
};
static const int num_initial_vowels = sizeof(initial_vowel) / sizeof(initial_vowel[0]);

static const char * interior_consonant[] =
{
	"b",
	"bb",
	"br",
	"bs",
	"bt",
	"c",
	"ch",
	"ck",
	"cr",
	"cs",
	"ct",
	"d",
	"dd",
	"dr",
	"ds",
	"f",
	"ff",
	"fr",
	"fs",
	"ft",
	"g",
	"gh",
	"h",
	"l",
	"ll",
	"m",
	"mm",
	"mn",
	"mp",
	"n",
	"ng",
	"nn",
	"nt",
	"nth",
	"nthr",
	"p",
	"pp",
	"s",
	"sh",
	"sp",
	"ss",
	"st",
	"str",
	"t",
	"th",
	"tr",
	"ts",
	"tt",
	"tch",
	"v",
	"w",
	"x",
	"z",
};
static const int num_interior_consonants = sizeof(interior_consonant) / sizeof(interior_consonant[0]);

static const char * interior_vowel[] =
{
	"a",
	"ae",
	"ai",
	"ao",
	"au",
	"e",
	"ee",
	"eo",
	"i",
	"ia",
	"ie",
	"io",
	"iu",
	"o",
	"oo",
	"oa",
	"ou",
	"u",
	"ua",
	"ui",
};
static const int num_interior_vowels = sizeof(interior_vowel) / sizeof(interior_vowel[0]);

static const char * final_consonant[] =
{
	"b",
	"ch",
	"ck",
	"d",
	"f",
	"g",
	"ll",
	"m",
	"n",
	"ng",
	"p",
	"r",
	"s",
	"ss",
	"t",
	"th",
	"tch",
	"w",
	"x",
	"z",
};
static const int num_final_consonants = sizeof(final_consonant) / sizeof(final_consonant[0]);

static std::string random_name()
{
	std::string name;

	int n = rand() % (num_initial_consonants + num_initial_vowels);

	if (n < num_initial_consonants)
	{
		name += initial_consonant[n];

		n = rand() % num_interior_vowels;
		name += interior_vowel[n];
	}
	else
	{
		name += initial_vowel[n - num_initial_consonants];
	}

	for (int num_syllables = 1; ; ++num_syllables)
	{
		if (num_syllables > 1 && (rand() & 7) < 7)
		{
			n = rand() % num_final_consonants;
			name += final_consonant[n];

			break;
		}

		n = rand() % num_interior_consonants;
		name += interior_consonant[n];

		if ((rand() & 7) < 7)
		{
			n = rand() % num_interior_vowels;
			name += interior_vowel[n];

			break;
		}

		n = rand() % num_interior_vowels;
		name += interior_vowel[n];
	}

	return name;
}

//----------------------------------------------------------------------------

static void create_planet(const tOrbitElements & orbit, double r)
{
	g_planets.push_back(tPlanet());
	tPlanet & planet = g_planets.back();

	planet.orbit = orbit;
	planet.radius = r;
	planet.name = random_name();
	planet.visited = false;

	double ea = eccentric_anomaly(pi / 2.0, orbit.e);

	// Position of planet in its orbit, in perifocal coordinates:
	double x = orbit.a * (cos(ea) - orbit.e);
    double y = orbit.b * sin(ea);

	double middle_r = sqrt(x*x + y*y);

	double u = std::min(1.0, middle_r / 30.0);

	planet.color[0] = 0.87 - 0.87 * u;
	planet.color[1] = 0.87 - 0.8 * u;
	planet.color[2] = u;
}

//----------------------------------------------------------------------------

int gcd(int a, int b)
{
	while (b != 0)
	{
		int a_new = b;
		b = a % b;
		a = a_new;
	}

	return a;
}

//----------------------------------------------------------------------------

static bool planet_collides(const tOrbitElements & orbit, double r)
{
	const double kMinDistance = 1.0;

	double perihelion = orbit.a * (1.0 - orbit.e);
	if (perihelion < r + kSunRadius + kMinDistance)
		return true;

	double aphelion = orbit.a * (1.0 + orbit.e);

	for (std::list<tPlanet>::const_iterator p = g_planets.begin(); p != g_planets.end(); ++p)
	{
		double dist_min = kMinDistance + r + p->radius;

		double perihelion2 = p->orbit.a * (1.0 - p->orbit.e);
		double aphelion2 = p->orbit.a * (1.0 + p->orbit.e);

		if (perihelion - aphelion2 >= dist_min || perihelion2 - aphelion >= dist_min)
			continue;

		int common_period = (orbit.period * p->orbit.period) / gcd(orbit.period, p->orbit.period);

		int steps = common_period * 48;

		double t = 0.0;
		double t_inc = kPeriodScale / 48.0;
		double sqdist_min = dist_min * dist_min;

		for (int i = 0; i < steps; ++i, t += t_inc)
		{
			dvec2 p0;
			dvec2 p1;
			update_planet_pos(orbit, t, p0);
			update_planet_pos(p->orbit, t, p1);

			if ((p0 - p1).sqlen() < sqdist_min)
				return true;
		}
	}

	return false;
}

//----------------------------------------------------------------------------

static bool operator < (const tPlanet & p0, const tPlanet & p1)
{
	if (p0.orbit.period < p1.orbit.period)
		return true;
	if (p1.orbit.period < p0.orbit.period)
		return false;

	if (p0.orbit.e < p1.orbit.e)
		return true;
	if (p1.orbit.e < p0.orbit.e)
		return false;

	return p0.radius < p1.radius;
}

//----------------------------------------------------------------------------

static const int period[64] =
{
	1, 1,
	2, 2, 2,
	3, 3, 3, 3,
	4, 4, 4, 4, 4,
	5, 5, 5, 5, 5, 5,
	6, 6, 6, 6, 6, 6, 6,
	7, 7, 7, 7, 7, 7, 7, 7,
	8, 8, 8, 8, 8, 8, 8, 8, 8,
	9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
	10,10,10,10,10,10,10,10,10,10
};

static void create_planets()
{
	g_planets.clear();

	for (int i = 0; i < 512; ++i)
	{
//		double r = 0.08 - 0.03125 * log(frand());
		double r = 0.72 - 0.28 * log(frand());

		tOrbitElements orbit;
//		orbit.period = 1 + (rand() % 10);
		orbit.period = period[rand() & 63];
		orbit.w = two_pi * frand();
		orbit.t0 = frand();
		orbit.e = std::min(0.98, -0.25 * log(frand()));

		orbit.a = pow(orbit.period * kPeriodScale / two_pi, 0.666666666666666666667);
		orbit.b = orbit.a * sqrt(1.0 - orbit.e * orbit.e);
		orbit.cos_w = cos(orbit.w);
		orbit.sin_w = sin(orbit.w);

		if (!planet_collides(orbit, r))
		{
			create_planet(orbit, r);
		}
	}

	g_planets.sort();
}

//----------------------------------------------------------------------------

static void draw_disc()
{
	glCallList(g_disc_list);
}

//----------------------------------------------------------------------------

static void glPrint(const char *msg)
{
	glCallLists(int(strlen(msg)), GL_UNSIGNED_BYTE, msg); 
}

//----------------------------------------------------------------------------

static void projection_matrix(double m[16])
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

//----------------------------------------------------------------------------

static void modelview_matrix(double m[16])
{
	memset(m, 0, 16*sizeof(double));
	m[0] = 1;
	m[5] = 1;
	m[10] = 1;
	m[15] = 1;
}

//----------------------------------------------------------------------------

void draw_scene()
{
	glClear(GL_COLOR_BUFFER_BIT);

	double m[16];

	glMatrixMode(GL_PROJECTION);
	projection_matrix(m);
	glLoadMatrixd(m);

	glMatrixMode(GL_MODELVIEW);
	modelview_matrix(m);
	glLoadMatrixd(m);

	// Draw the sun.
	glColor3d(1, 1, 0.75);
	glPushMatrix();
	glScaled(kSunRadius, kSunRadius, 1.0);
	draw_disc();
	glPopMatrix();

	for (std::list<tPlanet>::const_iterator p = g_planets.begin(); p != g_planets.end(); ++p)
	{
		if (g_highlighted_planet != &*p)
			draw_orbit(*p, 0.125);
	}

	if (g_time > g_start_time)
	{
		for (std::list<tPlanet>::const_iterator p = g_planets.begin(); p != g_planets.end(); ++p)
			draw_planet(*p, p->start_position, 0.125, &(*p) == g_location_planet);
	}

	if (g_highlighted_planet)
		draw_orbit(*g_highlighted_planet, 0.95);

	for (std::list<tPlanet>::const_iterator p = g_planets.begin(); p != g_planets.end(); ++p)
		draw_planet(*p, p->position, 1.0, &(*p) == g_location_planet);

	// Draw an integrated trajectory of the highlighted planet, to double-check the math.

	if (g_time > g_start_time)
	{
		dvec2 pos1;
		dvec2 vel1;

		if (g_highlighted_planet)// && g_highlighted_planet != g_location_planet)
		{
			elements_to_state(g_highlighted_planet->orbit, g_time, pos1, vel1);
		}
		else
		{
			pos1[0] = (double(g_mouse_x) - double(g_size_x) / 2.0) * g_au_per_pixel;
			pos1[1] = (double(g_size_y) / 2.0 - double(g_mouse_y)) * g_au_per_pixel;

			vel1[0] = 0.0;
			vel1[1] = 0.0;
		}

		double dt = g_time - g_start_time;

		dvec2 pos0;
		dvec2 vel0;
		elements_to_state(g_location_planet->orbit, g_start_time, pos0, vel0);

		dvec2 v0_short;
		dvec2 v1_short;
		lambert(pos0, pos1, dt, false, v0_short, v1_short);
		double dv_short = (vel0 - v0_short).len() + (vel1 - v1_short).len();

		dvec2 v0_long;
		dvec2 v1_long;
		lambert(pos0, pos1, dt, true, v0_long, v1_long);
		double dv_long = (vel0 - v0_long).len() + (vel1 - v1_long).len();

		g_dv_cur = (dv_short < dv_long) ? dv_short : dv_long;
		dvec2 vel = (dv_short < dv_long) ? v0_short : v0_long;

		if (!g_highlighted_planet)
			glColor3d(1.0, 1.0, 1.0);
		else if (g_dv_cur <= g_dv_max)
			glColor3d(0.0, 1.0, 0.0);
		else
			glColor3d(1.0, 0.0, 0.0);
		draw_trajectory(pos0, vel, dt);
	}

	// Display the highlighted planet's name.

	if (g_highlighted_planet)
	{
		const tPlanet &p = *g_highlighted_planet;

		glColor3d(1, 1, 1);
		double offset = p.radius * 0.707107 + 3 * g_au_per_pixel;
		glRasterPos3d(p.position[0] + offset, p.position[1] + offset, 0.0);

		if (g_time > g_start_time && g_highlighted_planet != g_location_planet)
		{
			char msg[128];
			sprintf_s(msg, sizeof(msg), "%s (%.0f)",
				p.name.c_str(),
				floor(g_dv_cur * 100.0 + 0.5));
			glPrint(msg);
		}
		else
		{
			glPrint(p.name.c_str());
		}
	}

	// display a string:
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, g_size_x, 0, g_size_y, -1, 1);

	glColor3d(1, 1, 1);
	glRasterPos2d(10, 10);

	if (g_time > g_start_time)
	{
		double dt = (g_time - g_start_time) * days_per_time_unit;

		int days = int(floor(dt + 0.5));

		char buffer[128];
		sprintf_s(buffer, sizeof(buffer), "+%d days", days);
		glPrint(buffer);
	}

	SwapBuffers(g_hDC);
}

//----------------------------------------------------------------------------

// Compute the eccentric anomaly from mean anomaly using iteration.
// M - mean anomaly in radians
// e - orbit eccentricity
static double eccentric_anomaly(double M, double e)
{
    // initial approximation of eccentric anomaly
	double E = M + e*sin(M) * (1.0 + e*cos(M));

    // iterate to improve accuracy
	double E1;
	int n = 0;
	do
	{
		E1 = E;
		E -= (E - e*sin(E) - M) / (1.0 - e*cos(E));
		++n;
	}
	while (n < 20 && fabs(E - E1) >= 1.0e-7);

	return E;
}

static void draw_orbit(const tPlanet &planet, double color_scale)
{
	glColor3d(planet.color[0] * color_scale, planet.color[1] * color_scale, planet.color[2] * color_scale);

	double a = planet.orbit.a;
	double b = planet.orbit.b;
	double e = planet.orbit.e;
	double cos_w = planet.orbit.cos_w;
	double sin_w = planet.orbit.sin_w;

	double angle_inc = two_pi / 64;

	glBegin(GL_LINE_LOOP);

	for (double angle = 0; angle < two_pi; angle += angle_inc)
	{
		double x = a * (cos(angle) - e);
		double y = b * sin(angle);

		dvec2 p;
		p[0] = cos_w * x - sin_w * y;
		p[1] = sin_w * x + cos_w * y;

		glVertex2dv(&p[0]);
	}

	glEnd();
}

static void update_planet_pos(const tOrbitElements & orbit, double t, dvec2 &pos)
{
	// ma = mean anomaly (the phase angle of the planet in its orbit)
	double ma = orbit.t0 + t / (kPeriodScale * orbit.period);
	ma -= floor(ma);
	ma *= two_pi;

	// ea = eccentric anomaly (angle on unit circle that corresponds to planet position)
	double ea = eccentric_anomaly(ma, orbit.e);

	// Position of planet in its orbit, in perifocal coordinates:
	double x = orbit.a * (cos(ea) - orbit.e);
    double y = orbit.b * sin(ea);

	// Rotate the coordinates from perifocal space to world space:
	double cos_w = orbit.cos_w;
	double sin_w = orbit.sin_w;

	pos[0] = cos_w * x - sin_w * y;
	pos[1] = sin_w * x + cos_w * y;
}

static void elements_to_state
(
	// In:
	const tOrbitElements &orbit,
	double t,

	// Out:
	dvec2 & position,
	dvec2 & velocity
)
{
	// ma = mean anomaly (the phase angle of the planet in its orbit)
	double ma = orbit.t0 + t / (kPeriodScale * orbit.period);
	ma -= floor(ma);
	ma *= two_pi;

	// ea = eccentric anomaly (angle on unit circle that corresponds to planet position)
	double ea = eccentric_anomaly(ma, orbit.e);

	// Position of planet in its orbit, in perifocal coordinates:
	double x = orbit.a * (cos(ea) - orbit.e);
    double y = orbit.b * sin(ea);

	double r = sqrt(x*x + y*y);
	double vx = -y / r;
	double vy = orbit.e + x / r;

	double scale = sqrt(orbit.a) / orbit.b;
	vx *= scale;
	vy *= scale;

	// Rotate the coordinates from perifocal space to world space:
	double cos_w = orbit.cos_w;
	double sin_w = orbit.sin_w;

	position[0] = cos_w * x - sin_w * y;
	position[1] = sin_w * x + cos_w * y;

	velocity[0] = cos_w * vx - sin_w * vy;
	velocity[1] = sin_w * vx + cos_w * vy;
}

void dump_planets(const char * file)
{
	FILE * f = fopen(file, "w");
	if (!f)
		return;

	std::list<tPlanet>::const_iterator p_end = g_planets.end();
	int i = 0;
	for (std::list<tPlanet>::const_iterator p = g_planets.begin(); p != p_end; ++p, ++i)
	{
		dvec2 pos, vel;
		elements_to_state(p->orbit, g_time, pos, vel);

		fprintf(f, "{ %g, %g, %g, %g, %g }, // %d\n",
			pos[0],
			pos[1],
			vel[0],
			vel[1],
			p->radius,
			i);
	}

	fclose(f);
}

static void draw_planet(const tPlanet &planet, dvec2 pos, double color_scale, bool selected)
{
	glPushMatrix();
	glTranslated(pos[0], pos[1], 0.0);
	double scale = planet.radius + 2.0 * g_au_per_pixel;
	glScaled(scale, scale, 1.0);
	if (selected)
		glColor3d(color_scale, color_scale, color_scale);
	else if (planet.visited)
		glColor3d(0, color_scale, 0);
	else
		glColor3d(0, 0, 0);
	draw_disc();
	glPopMatrix();

	glPushMatrix();
	glTranslated(pos[0], pos[1], 0.0);
	glScaled(planet.radius, planet.radius, 1.0);
	glColor3d(planet.color[0] * color_scale, planet.color[1] * color_scale, planet.color[2] * color_scale);
	draw_disc();
	glPopMatrix();
}

static void draw_trajectory(dvec2 pos0, dvec2 vel0, double dt)
{
	int num_steps = int(ceil(dt * 32.0));

	double step = dt / double(num_steps);
	double half_step = step / 2.0;

	glBegin(GL_LINE_STRIP);

	dvec2 pos = pos0;
	dvec2 vel = vel0;
	double r2 = pos.sqlen();
	dvec2 acc = pos / -(r2 * sqrt(r2));

	glVertex2dv(&pos[0]);

	for (int i = 0; i < num_steps; ++i)
	{
		dvec2 pos1 = pos + vel * step;
		dvec2 vel1 = vel + acc * step;

		r2 = pos1.sqlen();
		if (r2 < kSunRadius * kSunRadius)
			break;

		dvec2 acc1 = pos1 / -(r2 * sqrt(r2));

		pos += (vel + vel1) * half_step;
		vel += (acc + acc1) * half_step;
		acc = acc1;

		glVertex2dv(&pos[0]);
	}

	glEnd();
}

static void begin_mouse_drag(HWND hWnd)
{
	g_selected_planet = g_highlighted_planet;

	if (g_selected_planet)
	{
		const tOrbitElements & orbit = g_selected_planet->orbit;
		double period = kPeriodScale * orbit.period;

		double current_mean_anomaly = orbit.t0 + g_time / period;
		current_mean_anomaly -= floor(current_mean_anomaly);

		double ea = eccentric_anomaly(g_mouse_x, g_mouse_y, orbit);
		double mean_anomaly = (ea - orbit.e * sin(ea)) / two_pi;

		g_mean_anomaly_offset = mean_anomaly - current_mean_anomaly;
	}

	SetCapture(hWnd);
}

static void end_mouse_drag()
{
	g_selected_planet = NULL;

	ReleaseCapture();
	update_highlight();
}

static void update_mouse_drag()
{
	const tOrbitElements & orbit = g_selected_planet->orbit;
	double period = kPeriodScale * orbit.period;

	double current_mean_anomaly = g_mean_anomaly_offset + orbit.t0 + g_time / period;
	current_mean_anomaly -= floor(current_mean_anomaly);

	double ea = eccentric_anomaly(g_mouse_x, g_mouse_y, orbit);
	double mean_anomaly = (ea - orbit.e * sin(ea)) / two_pi;

	double mean_anomaly_change = mean_anomaly - current_mean_anomaly;
	if (mean_anomaly_change < -0.5)
		mean_anomaly_change += 1.0;
	else if (mean_anomaly_change > 0.5)
		mean_anomaly_change -= 1.0;

	double dt = mean_anomaly_change * period;

	advance_time(dt);
}

static void travel()
{
	if (!g_highlighted_planet)
		return;

	if (g_dv_cur > g_dv_max)
		return;

	for (std::list<tPlanet>::iterator p = g_planets.begin(); p != g_planets.end(); ++p)
		p->start_position = p->position;

	g_start_time = g_time;

	g_location_planet = g_highlighted_planet;
	g_location_planet->visited = true;

	repaint();
}

//----------------------------------------------------------------------------

inline double sq_dist_point_line
(
	const dvec3 & point,
	const dvec3 & line_origin,
	const dvec3 & line_delta // assumed to be unit length
)
{
	dvec3 diff = point;
	diff -= line_origin;
	diff -= line_delta*line_delta.dot(diff);
	return diff.sqlen();
}

//----------------------------------------------------------------------------

static tPlanet *closest_planet(int mouse_x, int mouse_y)
{
	int viewport[4] = { 0, 0, g_size_x, g_size_y };
	double mvmatrix[16], projmatrix[16];
	projection_matrix(projmatrix);
	modelview_matrix(mvmatrix);

	mouse_y = viewport[3] - mouse_y - 1;

	dvec3 line_origin;
	dvec3 line_delta;
	gluUnProject(mouse_x, mouse_y, 0, mvmatrix, projmatrix, viewport, &line_origin[0], &line_origin[1], &line_origin[2]);
	gluUnProject(mouse_x, mouse_y, 1, mvmatrix, projmatrix, viewport, &line_delta[0], &line_delta[1], &line_delta[2]);
	line_delta -= line_origin;
	line_delta /= line_delta.len();

	tPlanet *closestPlanet = NULL;
	double closestDist = std::numeric_limits<double>::max();
	std::list<tPlanet>::iterator p;
	for (p = g_planets.begin(); p != g_planets.end(); ++p)
	{
		dvec3 planet_pos(p->position[0], p->position[1], 0.0);
		double dist = sq_dist_point_line(planet_pos, line_origin, line_delta);
		double r2 = p->radius * p->radius;
		if (dist < r2 && (!closestPlanet || dist < closestDist))
		{
			closestPlanet = & *p;
			closestDist = dist;
		}
	}

	return closestPlanet;
}

//----------------------------------------------------------------------------

static double eccentric_anomaly(int mouse_x, int mouse_y, const tOrbitElements & orbit)
{
	if (orbit.e >= 1)
		return 0;

	// Outline:
	// Circle space is the space where the source unit circle is drawn.
	// World space is where the circle is transformed into an ellipse in the world
	// Line space is where the ellipse is rotated and translated so that the
	//   Z axis represents the query line.
	// circle_to_line = circle_to_world * world_to_line

	int viewport[4];
	double mvmatrix[16], projmatrix[16];
	glGetIntegerv(GL_VIEWPORT, viewport);
	modelview_matrix(mvmatrix);
	projection_matrix(projmatrix);

	mouse_y = viewport[3] - mouse_y - 1;

	dvec3 line_origin;
	dvec3 line_delta;
	gluUnProject(mouse_x, mouse_y, 0, mvmatrix, projmatrix, viewport, &line_origin[0], &line_origin[1], &line_origin[2]);
	gluUnProject(mouse_x, mouse_y, 1, mvmatrix, projmatrix, viewport, &line_delta[0], &line_delta[1], &line_delta[2]);
	line_delta -= line_origin;

	// world_point = world_from_line[0] * line_point[0] +
	//               world_from_line[1] * line_point[1] +
	//               world_from_line[2] * line_point[2] +
	//               world_from_line[3];
	dvec3 world_from_line[4];
	world_from_line[2] = line_delta;
	world_from_line[2] /= world_from_line[2].len();
	int max_axis = 0;
	if (fabs(world_from_line[2][1]) > fabs(world_from_line[2][max_axis]))
		max_axis = 1;
	if (fabs(world_from_line[2][2]) > fabs(world_from_line[2][max_axis]))
		max_axis = 2;
	world_from_line[1] = dvec3(0, 0, 0);
	world_from_line[1][(max_axis+1)%3] = 1;
	world_from_line[1] = world_from_line[2].cross(world_from_line[1]);
	world_from_line[1] /= world_from_line[1].len();
	world_from_line[0] = world_from_line[1].cross(world_from_line[2]);
	world_from_line[3] = line_origin;

	// line_point = line_from_world[0] * world_point[0] +
	//              line_from_world[1] * world_point[1] +
	//              line_from_world[2] * world_point[2] +
	//              line_from_world[3];
	dvec3 line_from_world[4];
	line_from_world[0][0] = world_from_line[0][0];
	line_from_world[0][1] = world_from_line[1][0];
	line_from_world[0][2] = world_from_line[2][0];
	line_from_world[1][0] = world_from_line[0][1];
	line_from_world[1][1] = world_from_line[1][1];
	line_from_world[1][2] = world_from_line[2][1];
	line_from_world[2][0] = world_from_line[0][2];
	line_from_world[2][1] = world_from_line[1][2];
	line_from_world[2][2] = world_from_line[2][2];
	line_from_world[3][0] = -line_origin.dot(world_from_line[0]);
	line_from_world[3][1] = -line_origin.dot(world_from_line[1]);
	line_from_world[3][2] = -line_origin.dot(world_from_line[2]);

	// world_point = world_from_circle[0] * circle_point[0] +
	//               world_from_circle[1] * circle_point[1] +
	//               world_from_circle[2] * circle_point[2] + // circle_point[2] = 0
	//               world_from_circle[3];
	dvec3 world_from_circle[4];

	world_from_circle[0][0] = orbit.a * orbit.cos_w;
	world_from_circle[0][1] = orbit.a * orbit.sin_w;
	world_from_circle[0][2] = 0.0;

	world_from_circle[1][0] = orbit.b * -orbit.sin_w;
	world_from_circle[1][1] = orbit.b * orbit.cos_w;
	world_from_circle[1][2] = 0.0;

	world_from_circle[2] = dvec3(0, 0, 1);
	world_from_circle[3] = world_from_circle[0] * -orbit.e;

	// line_point = line_from_circle[0] * circle_point[0] +
	//              line_from_circle[1] * circle_point[1] +
	//              line_from_circle[2] * circle_point[2] + // circle_point[2] = 0
	//              line_from_circle[3];
	//
	// line_from_circle = line_from_world * world_from_circle;
	dvec3 line_from_circle[4];
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 3; ++j)
				line_from_circle[i][j] = world_from_line[j].dot(world_from_circle[i]);
	line_from_circle[3] += line_from_world[3];

	const int num_search_intervals = 16;
	const double angle_inc = two_pi / num_search_intervals;//64;

	double CC = line_from_circle[0][0]*line_from_circle[0][0] + line_from_circle[0][1]*line_from_circle[0][1];
	double SS = line_from_circle[1][0]*line_from_circle[1][0] + line_from_circle[1][1]*line_from_circle[1][1];
	double CS = 2.0 * (line_from_circle[0][0]*line_from_circle[1][0] + line_from_circle[0][1]*line_from_circle[1][1]);
	double C = 2.0 * (line_from_circle[0][0]*line_from_circle[3][0] + line_from_circle[0][1]*line_from_circle[3][1]);
	double S = 2.0 * (line_from_circle[1][0]*line_from_circle[3][0] + line_from_circle[1][1]*line_from_circle[3][1]);
	double offset = line_from_circle[3][0]*line_from_circle[3][0] + line_from_circle[3][1]*line_from_circle[3][1];

	double dCCSS = CS;
	double dCS = 2.0 * (SS - CC);
	double dC = S;
	double dS = -C;

	double closest_angle = 0;
	double closest_d2 = std::numeric_limits<double>::max();

	for (int i = 0; i < num_search_intervals; ++i)
	{
		double a_min = angle_inc * i;
		double c = cos(a_min);
		double s = sin(a_min);
		double d0 = c*(c*dCCSS + s*dCS + dC) + s*(s*-dCCSS + dS);
		double a_max = angle_inc * (i + 1);
		c = cos(a_max);
		s = sin(a_max);
		double d1 = c*(c*dCCSS + s*dCS + dC) + s*(s*-dCCSS + dS);
		if (d0 < 0 && 0 <= d1)
		{
			// Hunt for a zero crossing within this interval.
			for (int i = 0; i < 64; ++i)
			{
				double a = (a_min + a_max) / 2;
				c = cos(a);
				s = sin(a);
				double d = c*(c*dCCSS + s*dCS + dC) + s*(s*-dCCSS + dS);
				if (d < 0)
					a_min = a;
				else
					a_max = a;
			}
			// Evaluate the function at the zero crossing and make sure it's less than the minimum found so far.
			c = cos(a_min);
			s = sin(a_min);
			double d2 = c*(c*CC + C) + s*(s*SS + c*CS + S) + offset;
			if (d2 < closest_d2)
			{
				closest_d2 = d2;
				closest_angle = a_min;
			}
		}
	}

	return closest_angle;
}

//----------------------------------------------------------------------------

static void set_time(double t)
{
	if (t == g_time)
		return;

	g_time = t;

	// Compute planet positions.
	for (std::list<tPlanet>::iterator p = g_planets.begin(); p != g_planets.end(); ++p)
		update_planet_pos(p->orbit, g_time, p->position);
}

//----------------------------------------------------------------------------

static void advance_time(double dt)
{
	double new_time = std::max(g_start_time, g_time + dt);

	if (new_time != g_time)
	{
		set_time(new_time);
		repaint();
	}
}

//----------------------------------------------------------------------------

static void repaint()
{
	InvalidateRect(g_hWnd, NULL, FALSE);
}

//----------------------------------------------------------------------------

#if 0
static void state_to_elements
(
	// In:
	const dvec3 &position, // astronomical units
	const dvec3 &velocity, // astronomical units per year

	// Out:
	tOrbitElements &orbit
)
{
	dvec3 h = position.cross(velocity); // angular momentum vector
	dvec3 n(-h[1], h[0], 0); // node vector
	double r = position.len();
	dvec3 e_vec = (position*(velocity.sqlen() - 1.0/r) - velocity*position.dot(velocity));

	orbit.e = e_vec.len();
	orbit.p = h.sqlen();
	orbit.i = acos(h[2] / h.len());
	orbit.O = atan2(n[1], n[0]);
	if (orbit.O < 0)
		orbit.O += two_pi;
	orbit.w = atan2(n.cross(e_vec).len(), n.dot(e_vec));
	if (e_vec[2] < 0)
		orbit.w = two_pi - orbit.w;

	double cos_true_anomaly = e_vec.dot(position) / (orbit.e*r);
	double eccentric_anomaly = acos((orbit.e + cos_true_anomaly) / (1 + orbit.e * cos_true_anomaly));
	if (position.dot(velocity) < 0)
		eccentric_anomaly = two_pi - eccentric_anomaly;
	double mean_anomaly = eccentric_anomaly - orbit.e * sin(eccentric_anomaly);

	orbit.t0 = mean_anomaly / two_pi;
}
#endif

//----------------------------------------------------------------------------

static void find_c2_c3(double z, double &c2, double &c3)
{
	if (z > kEpsilon)
	{
		c2 = (1 - cos(sqrt(z))) / z;
		c3 = (sqrt(z) - sin(sqrt(z))) / pow(z, 1.5);
	}
	else if (z < -kEpsilon)
	{
		c2 = (1.0 - cosh(sqrt(-z))) / z;
		c3 = (sinh(sqrt(-z)) - sqrt(-z)) / pow(-z, 1.5);
	}
	else
	{
		c2 = 0.5;
		c3 = 0.16666666666666666666666666666667; // 1/6
	}
}

//----------------------------------------------------------------------------

static void lambert
(
	const dvec2 &start_pos,
	const dvec2 &end_pos,
	double time_of_flight,
	bool long_way,

	dvec2 &start_vel,
	dvec2 &end_vel
)
{
	double r0 = start_pos.len();
	double r1 = end_pos.len();

	double angle = atan2(start_pos.cross(end_pos), start_pos.dot(end_pos));

	double A = sqrt(r0*r1*(1.0 + cos(angle)));
	if (A < kEpsilon)
	{
		start_vel = dvec2(0, 0);
		end_vel = start_vel;
		return;
	}

	if (long_way)
		A = -A;

	double psi_new = 0;
	double c2_new, c3_new;
	find_c2_c3(psi_new, c2_new, c3_new);
	double z_upper = 4.0 * pi * pi;
	double z_lower = -4.0 * pi;
	double yn;

	for (int iteration = 1000; iteration > 0; --iteration)
	{
		if (fabs(c2_new) > kEpsilon)
			yn = r0 + r1 + (A * (psi_new*c3_new - 1) / sqrt(c2_new));
		else
			yn = r0 + r1;

		if (A > 0.0 && yn < 0.0) // readjust z_lower until y > 0
		{
			int YNegKtr = 1;
			while(yn < 0)
			{
				psi_new = 0.8 * (1.0 / c3_new) * (1.0 - (r0 + r1) * sqrt(c2_new) / A);
				find_c2_c3(psi_new, c2_new, c3_new);
				z_lower = psi_new;
				if (fabs(c2_new) > kEpsilon)
					yn = r0 + r1 + (A * (psi_new*c3_new - 1) / sqrt(c2_new));
				else
					yn = r0 + r1;
				++YNegKtr;
				if (YNegKtr >= 10)
					return;
			}
		}
		double x0 = sqrt(yn / c2_new);
		double t = (x0*x0*x0*c3_new + A*(sqrt(yn)));
		if (t <= time_of_flight)
			z_lower = psi_new;
		else
			z_upper = psi_new;
		double z_new = (z_upper + z_lower) / 2.0;
		find_c2_c3(z_new, c2_new, c3_new);
		psi_new = z_new;
		if (fabs(t - time_of_flight) < kEpsilon)
			break;
	}

	// Use F and G series to find velocity vectors.
	double F    = 1.0 - yn / r0;
	double GDot = 1.0 - yn / r1;
	double G    = A * sqrt(yn);

	start_vel = (end_pos - start_pos*F) / G;
	end_vel   = (end_pos*GDot - start_pos) / G;
}
