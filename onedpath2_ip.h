#pragma once

#include "problem.h"

struct OneDPath2InteriorPoint : public Problem
{
	OneDPath2InteriorPoint();
	~OneDPath2InteriorPoint() override;
	void init() override;
	void onActivate() override;
	void onKey(unsigned char) override;
	void onSpecialKey(int) override;
	void onDraw() override;
	void onMouseMove(int x, int y) override;
	void onMouseDown() override;
	void onMouseUp() override;
};
