#include <math.h>

#define DPF 17.1919863
#define FPS 5.66

// Degrees per second
//#define DPS 76.6333715
#define DPS 66.6333715

#define MAX_DPS 40.0
#define CAMERA_DELAY 2000

// Arena Dimensions (Webcam)
#define ARENA_WIDTH 1024.0
#define ARENA_HEIGHT 768.0

// Robot Measurements (Real Life)
#define ROBO_REAL_WIDTH 15.7
#define ROBO_REAL_HEIGHT 5.6

// Robot Measurements (Webcam)
#define ROBO_ARENA_WIDTH 145.0
#define ROBO_ARENA_HEIGHT 48.0

// Travel Distance (cm) per Second @ Speed 20
#define CPS_SLOW 6.4

// Travel Distance (cm) per Second @ Speed 100
// #define CPS_FAST 19.16
#define CPS_FAST 38.32

// This is the angle that we predict we're facing.
double angle_prediction = -1.0;

int estimate_angle_delay(double angle) {
	double abs = fabs(angle);

//	double seconds = abs / (DPF * FPS);
	double seconds = abs / DPS;

	int millis = ceil(seconds * 1000);
	return millis;
}

double inverse_angle(double angle) {
	double inverse = angle + 180.0;
	return fmod(inverse, 360.0);
}

void adjust_angle_by_direction(double, double, double);

double translate_distance(double pixels) {
	double width_ratio = ROBO_ARENA_WIDTH / ROBO_REAL_WIDTH;
	double height_ratio = ROBO_ARENA_HEIGHT / ROBO_REAL_HEIGHT;
	double ratio_average = (width_ratio + height_ratio) / 2.0;
	double as_centimeters = pixels / ratio_average;
	return as_centimeters;
}

int estimate_travel_delay(double pixels, double speed, double barrier) {
	double centimeters = translate_distance(pixels) - barrier;
	double seconds = centimeters / speed;

	int millis = ceil(seconds * 1000);
	return millis;
}

double magnitude(double x, double y) {
	return sqrt(pow(x, 2) + pow(y, 2));
}
