#include <math.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

double vector_to_angle(double x, double y) {
	double real_x = x;
	double real_y = -y;

	double radians = atan2(real_y, real_x);
	double degrees = (180.0 * radians) / 3.141592654;
	double normalize = fmod(360.0 + degrees, 360.0);

	return normalize;
}

double min_rotation(double start_angle, double end_angle) {
	double delta = end_angle - start_angle;

	while (delta < 0) {
		delta += 360.0;
	}

	while (delta >= 360) {
		delta -= 360.0;
	}

	if (delta > 180.0) {
		return -(360.0 - delta);
	} else {
		return delta;
	}
}

void msleep(int time) {
	usleep(time * 1000);
}

long sleep_time = 0;

long current_time_millis() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	long start_time = (long) (tv.tv_sec) * 1000 + (long) (tv.tv_usec) / 1000;
	return start_time;
}

void psuedo_sleep(long time) {
	sleep_time = current_time_millis() + time;
}

int is_psuedo_sleeping() {
	if (current_time_millis() < sleep_time) {
//		fprintf(stderr, "%d -> %d (%d)\n", current_time_millis(), sleep_time, 1);
		return 1;
	} else {
//		fprintf(stderr, "%d -> %d (%d)\n", current_time_millis(), sleep_time, 0);
		return 0;
	}
}

