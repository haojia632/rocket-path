#pragma once

#include "problem.h"

struct OneDPathInteriorPoint : public Problem
{
	OneDPathInteriorPoint();
	~OneDPathInteriorPoint() override;
	void init() override;
	void onActivate() override;
	void onKey(unsigned char) override;
	void onSpecialKey(int) override;
	void onDraw() override;
	void onMouseMove(int x, int y) override;
	void onMouseDown() override;
	void onMouseUp() override;
};
