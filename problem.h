#pragma once

struct Problem
{
	virtual ~Problem() {}
	virtual void init() = 0;
	virtual void onKey(unsigned int) = 0;
	virtual void onDraw() = 0;
	virtual void onMouseMove(int x, int y) = 0;
	virtual void onMouseDown() = 0;
	virtual void onMouseUp() = 0;
};
