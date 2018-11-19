#pragma once

#include "problem.h"

struct OneDPath : public Problem
{
	OneDPath();
	~OneDPath() override;
	void init() override;
	void onKey(unsigned char) override;
	void onSpecialKey(int) override;
	void onDraw() override;
	void onMouseMove(int x, int y) override;
	void onMouseDown() override;
	void onMouseUp() override;
};
