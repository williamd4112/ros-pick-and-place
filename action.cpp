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
	static float up = 3.5;
	static float down = -2.5 * up;

	for (int i = 0; i < times; i++) {
		for (float ang = 0; ang <= 360.0f; ang += step_angular_interval) {
			printf("Try %f\n", ang);
			dir_t dir = dir_t { cos(ang), sin(ang) };
			// Up
			g_arm->move_relative(0, 0, up);
			usleep(time_interval);
			// Offset +
			g_arm->move_relative(dir.x * step_linear_interval, dir.y * step_linear_interval, 0);
			usleep(time_interval);
			// Down
			g_arm->move_relative(0, 0, down);
			usleep(time_interval);
			// Up
			g_arm->move_relative(0, 0, up);
			usleep(time_interval);
			// Offset -
			g_arm->move_relative(-dir.x * step_linear_interval, -dir.y * step_linear_interval, 0);
			usleep(time_interval);
			// Down
			g_arm->move_relative(0, 0, down);
			usleep(time_interval);
		}
	}
}
