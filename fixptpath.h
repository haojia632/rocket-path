#pragma once

#include "problem.h"

struct FixPointPath : public Problem
{
	FixPointPath();
	~FixPointPath() override;
	void init() override;
	void onKey(unsigned int) override;
	void onDraw() override;
	void onMouseMove(int x, int y) override;
	void onMouseDown() override;
	void onMouseUp() override;
};
