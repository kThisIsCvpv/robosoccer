#define MODEL_SIZE 5

double vector_angle(double x, double y) {
	if(x == 0) {
		if(y < 0) {
			return 90.0;
		} else {
			return 270.0;
		}
	} else {
		double rad = atan(y / x);
		double deg = (rad / 3.1415926) * 180.0;
		double abs = fabs(deg);

		if(x > 0) {
			if(y > 0) {
				return 360.0 - abs;
			} else {
				return abs;
			}
		} else {
			if(y > 0) {
				return 180 + abs;
			} else {
				return 180 - abs;
			}
		}
	}
}

double known_angle = -1;
double calibrate_angle = 0;
int count_calibrate = 0;

double find_rotation_angle(double angle_start, double angle_finish) {
	double angle_change = angle_finish - angle_start;

	while(angle_change < 0) {
		angle_change += 360.0;
	}

	while(angle_change >= 360) {
		angle_change -= 360.0;
	}

	if(angle_change >= 180.0) {
		return -(360.0 - angle_change);
	} else {
		return angle_change;
	}
}

int model_count = 0;
double velocity_mag[MODEL_SIZE] = {};
double velocity_ang[MODEL_SIZE] = {};