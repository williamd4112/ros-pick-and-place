#include "action.hpp"

#include <iostream>
#include <cstdio>
#include <cstdlib>

#include <cmath>

#include <unistd.h>

using namespace std;

extern Arm * g_arm;

struct dir_t
{
	float x, y;
};

void action_retry(int times, float step_angular_interval, float step_linear_interval, float time_interval)
{
	for (int i = 0; i < times; i++) {
		for (float ang = 0; ang <= 360.0f; ang += step_angular_interval) {
			dir_t dir = dir_t { cos(ang), sin(ang) };
			g_arm->move_relative(dir.x * step_linear_interval, dir.y * step_linear_interval, 0);
			usleep(time_interval);
			g_arm->move_relative(-dir.x * step_linear_interval, -dir.y * step_linear_interval, 0);
		}
	}
}
