#include "roboAI.h"

#include <math.h>

#include "model.c"
#include "util.c"
#include "music.c"

struct displayList *addPoint(struct displayList *head, int x, int y, double R, double G, double B) {
	struct displayList *newNode;
	newNode = (struct displayList *) calloc(1, sizeof(struct displayList));
	if (newNode == NULL) {
		fprintf(stderr, "addPoint(): Out of memory!\n");
		return head;
	}
	newNode->type = 0;
	newNode->x1 = x;
	newNode->y1 = y;
	newNode->x2 = -1;
	newNode->y2 = -1;
	newNode->R = R;
	newNode->G = G;
	newNode->B = B;

	newNode->next = head;
	return (newNode);
}

struct displayList *addLine(struct displayList *head, int x1, int y1, int x2, int y2, double R, double G, double B) {
	struct displayList *newNode;
	newNode = (struct displayList *) calloc(1, sizeof(struct displayList));
	if (newNode == NULL) {
		fprintf(stderr, "addLine(): Out of memory!\n");
		return head;
	}
	newNode->type = 1;
	newNode->x1 = x1;
	newNode->y1 = y1;
	newNode->x2 = x2;
	newNode->y2 = y2;
	newNode->R = R;
	newNode->G = G;
	newNode->B = B;
	newNode->next = head;
	return (newNode);
}

struct displayList *addVector(struct displayList *head, int x1, int y1, double dx, double dy, int length, double R, double G, double B) {
	struct displayList *newNode;
	double l;

	l = sqrt((dx * dx) + (dy * dy));
	dx = dx / l;
	dy = dy / l;

	newNode = (struct displayList *) calloc(1, sizeof(struct displayList));
	if (newNode == NULL) {
		fprintf(stderr, "addVector(): Out of memory!\n");
		return head;
	}
	newNode->type = 1;
	newNode->x1 = x1;
	newNode->y1 = y1;
	newNode->x2 = x1 + (length * dx);
	newNode->y2 = y1 + (length * dy);
	newNode->R = R;
	newNode->G = G;
	newNode->B = B;
	newNode->next = head;
	return (newNode);
}

struct displayList *addCross(struct displayList *head, int x, int y, int length, double R, double G, double B) {
	struct displayList *newNode;
	newNode = (struct displayList *) calloc(1, sizeof(struct displayList));
	if (newNode == NULL) {
		fprintf(stderr, "addLine(): Out of memory!\n");
		return head;
	}
	newNode->type = 1;
	newNode->x1 = x - length;
	newNode->y1 = y;
	newNode->x2 = x + length;
	newNode->y2 = y;
	newNode->R = R;
	newNode->G = G;
	newNode->B = B;
	newNode->next = head;
	head = newNode;

	newNode = (struct displayList *) calloc(1, sizeof(struct displayList));
	if (newNode == NULL) {
		fprintf(stderr, "addLine(): Out of memory!\n");
		return head;
	}
	newNode->type = 1;
	newNode->x1 = x;
	newNode->y1 = y - length;
	newNode->x2 = x;
	newNode->y2 = y + length;
	newNode->R = R;
	newNode->G = G;
	newNode->B = B;
	newNode->next = head;
	return (newNode);
}

struct displayList *clearDP(struct displayList *head) {
	struct displayList *q;
	while (head) {
		q = head->next;
		free(head);
		head = q;
	}
	return (NULL);
}

struct blob *id_coloured_blob2(struct RoboAI *ai, struct blob *blobs, int col) {

	struct blob *p, *fnd;
	double vr_x, vr_y, maxfit, mincos, dp;
	double vb_x, vb_y, fit;
	double maxsize = 0;
	double maxgray;
	int grayness;
	int i;

	maxfit = .025;
	mincos = .65;
	maxgray = .25;

	if (col == 0) {
		vr_x = cos(0);
		vr_y = sin(0);
	} else if (col == 2) {
		vr_x = cos(2.0 * PI * (60.0 / 360.0));
		vr_y = sin(2.0 * PI * (60.0 / 360.0));
	} else if (col == 1) {
		vr_x = cos(2.0 * PI * (240.0 / 360.0));
		vr_y = sin(2.0 * PI * (240.0 / 360.0));
	}

	p = blobs;
	while (p != NULL) {
		if (p->size > maxsize)
			maxsize = p->size;
		p = p->next;
	}

	p = blobs;
	fnd = NULL;
	while (p != NULL) {

		vb_x = cos(p->H);
		vb_y = sin(p->H);

		dp = (vb_x * vr_x) + (vb_y * vr_y);

		fit = dp * p->S * p->S * (p->size / maxsize);

		grayness = 0;
		if (fabs(p->R - p->G) / p->R < maxgray && fabs(p->R - p->G) / p->G < maxgray && fabs(p->R - p->B) / p->R < maxgray && fabs(p->R - p->B) / p->B < maxgray && fabs(p->G - p->B) / p->G < maxgray && fabs(p->G - p->B) / p->B < maxgray)
			grayness = 1;

		if (fit > maxfit && dp > mincos && grayness == 0) {
			fnd = p;
			maxfit = fit;
		}

		p = p->next;
	}

	return (fnd);
}

void track_agents(struct RoboAI *ai, struct blob *blobs) {

	struct blob *p;
	double mg, vx, vy, pink, doff, dmin, dmax, adj;

	ai->st.ballID = 0;
	ai->st.selfID = 0;
	ai->st.oppID = 0;
	ai->st.ball = NULL;
	ai->st.self = NULL;
	ai->st.opp = NULL;

	p = id_coloured_blob2(ai, blobs, 2);
	if (p) {
		ai->st.ball = p;
		ai->st.ballID = 1;
		ai->st.bvx = p->cx - ai->st.old_bcx;
		ai->st.bvy = p->cy - ai->st.old_bcy;
		ai->st.ball->vx = ai->st.bvx;
		ai->st.ball->vy = ai->st.bvy;
		ai->st.bdx = p->dx;
		ai->st.bdy = p->dy;

		ai->st.old_bcx = p->cx;
		ai->st.old_bcy = p->cy;
		ai->st.ball->idtype = 3;

		vx = ai->st.bvx;
		vy = ai->st.bvy;
		mg = sqrt((vx * vx) + (vy * vy));
		if (mg > NOISE_VAR) {
			vx /= mg;
			vy /= mg;
			ai->st.bmx = vx;
			ai->st.bmy = vy;
		} else {
			ai->st.bmx = 0;
			ai->st.bmy = 0;
		}
		ai->st.ball->mx = ai->st.bmx;
		ai->st.ball->my = ai->st.bmy;
	} else {
		ai->st.ball = NULL;
	}

	if (ai->st.botCol == 0)
		p = id_coloured_blob2(ai, blobs, 1);
	else
		p = id_coloured_blob2(ai, blobs, 0);
	if (p != NULL && p != ai->st.ball) {
		ai->st.self = p;

		if (fabs(p->adj_Y[0][0]) > .1) {
			dmax = 384.0 - p->adj_Y[0][0];
			dmin = 767.0 - p->adj_Y[1][0];
			pink = (dmax - dmin) / (768.0 - 384.0);
			adj = dmin + ((p->adj_Y[1][0] - p->cy) * pink);
			p->cy = p->cy + adj;
			if (p->cy > 767)
				p->cy = 767;
			if (p->cy < 1)
				p->cy = 1;
		}

		ai->st.selfID = 1;
		ai->st.svx = p->cx - ai->st.old_scx;
		ai->st.svy = p->cy - ai->st.old_scy;
		ai->st.self->vx = ai->st.svx;
		ai->st.self->vy = ai->st.svy;
		ai->st.sdx = p->dx;
		ai->st.sdy = p->dy;

		vx = ai->st.svx;
		vy = ai->st.svy;
		mg = sqrt((vx * vx) + (vy * vy));

		if (mg > NOISE_VAR) {
			vx /= mg;
			vy /= mg;
			ai->st.smx = vx;
			ai->st.smy = vy;
		} else {
			ai->st.smx = 0;
			ai->st.smy = 0;
		}
		ai->st.self->mx = ai->st.smx;
		ai->st.self->my = ai->st.smy;
		ai->st.old_scx = p->cx;
		ai->st.old_scy = p->cy;
		ai->st.self->idtype = 1;
	} else
		ai->st.self = NULL;

	if (ai->st.botCol == 0)
		p = id_coloured_blob2(ai, blobs, 0);
	else
		p = id_coloured_blob2(ai, blobs, 1);
	if (p != NULL && p != ai->st.ball && p != ai->st.self) {
		ai->st.opp = p;

		if (fabs(p->adj_Y[0][1]) > .1) {
			dmax = 384.0 - p->adj_Y[0][1];
			dmin = 767.0 - p->adj_Y[1][1];
			pink = (dmax - dmin) / (768.0 - 384.0);
			adj = dmin + ((p->adj_Y[1][1] - p->cy) * pink);
			p->cy = p->cy + adj;
			if (p->cy > 767)
				p->cy = 767;
			if (p->cy < 1)
				p->cy = 1;
		}

		ai->st.oppID = 1;
		ai->st.ovx = p->cx - ai->st.old_ocx;
		ai->st.ovy = p->cy - ai->st.old_ocy;
		ai->st.opp->vx = ai->st.ovx;
		ai->st.opp->vy = ai->st.ovy;
		ai->st.odx = p->dx;
		ai->st.ody = p->dy;

		ai->st.old_ocx = p->cx;
		ai->st.old_ocy = p->cy;
		ai->st.opp->idtype = 2;

		vx = ai->st.ovx;
		vy = ai->st.ovy;
		mg = sqrt((vx * vx) + (vy * vy));
		if (mg > NOISE_VAR) {
			vx /= mg;
			vy /= mg;
			ai->st.omx = vx;
			ai->st.omy = vy;
		} else {
			ai->st.omx = 0;
			ai->st.omy = 0;
		}
		ai->st.opp->mx = ai->st.omx;
		ai->st.opp->my = ai->st.omy;
	} else
		ai->st.opp = NULL;

}

void id_bot(struct RoboAI *ai, struct blob *blobs) {

	struct blob *p;
	static double stepID = 0;
	double frame_inc = 1.0 / 5.0;

	BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 30);

	track_agents(ai, blobs);

	if (ai->st.selfID == 1 && ai->st.self != NULL)
		fprintf(stderr, "Successfully identified self blob at (%f,%f)\n", ai->st.self->cx, ai->st.self->cy);
	if (ai->st.oppID == 1 && ai->st.opp != NULL)
		fprintf(stderr, "Successfully identified opponent blob at (%f,%f)\n", ai->st.opp->cx, ai->st.opp->cy);
	if (ai->st.ballID == 1 && ai->st.ball != NULL)
		fprintf(stderr, "Successfully identified ball blob at (%f,%f)\n", ai->st.ball->cx, ai->st.ball->cy);

	stepID += frame_inc;
	if (stepID >= 1 && ai->st.selfID == 1) {
		ai->st.state += 1;
		stepID = 0;
		BT_all_stop(0);
	} else if (stepID >= 1)
		stepID = 0;

	return;
}

int setupAI(int mode, int own_col, struct RoboAI *ai) {
	switch (mode) {
	case AI_SOCCER:
		fprintf(stderr, "Standard Robo-Soccer mode requested\n");
		ai->st.state = 0;
		break;
	case AI_PENALTY:
		fprintf(stderr, "Penalty mode! let's kick it!\n");
		ai->st.state = 100;
		break;
	case AI_CHASE:
		fprintf(stderr, "Chasing the ball...\n");
		ai->st.state = 200;
		break;
	default:
		fprintf(stderr, "AI mode %d is not implemented, setting mode to SOCCER\n", mode);
		ai->st.state = 0;
	}

	BT_all_stop(0);
	ai->runAI = AI_main;
	ai->calibrate = AI_calibrate;
	ai->st.ball = NULL;
	ai->st.self = NULL;
	ai->st.opp = NULL;
	ai->st.side = 0;
	ai->st.botCol = own_col;
	ai->st.old_bcx = 0;
	ai->st.old_bcy = 0;
	ai->st.old_scx = 0;
	ai->st.old_scy = 0;
	ai->st.old_ocx = 0;
	ai->st.old_ocy = 0;
	ai->st.bvx = 0;
	ai->st.bvy = 0;
	ai->st.svx = 0;
	ai->st.svy = 0;
	ai->st.ovx = 0;
	ai->st.ovy = 0;
	ai->st.sdx = 0;
	ai->st.sdy = 0;
	ai->st.odx = 0;
	ai->st.ody = 0;
	ai->st.bdx = 0;
	ai->st.bdy = 0;
	ai->st.selfID = 0;
	ai->st.oppID = 0;
	ai->st.ballID = 0;
	ai->DPhead = NULL;
	fprintf(stderr, "Initialized!\n");

	return (1);
}

void AI_calibrate(struct RoboAI *ai, struct blob *blobs) {
	track_agents(ai, blobs);
}

void AI_main(struct RoboAI *ai, struct blob *blobs, void *state) {

	static double ux, uy, len, mmx, mmy, px, py, lx, ly, mi;
	double angDif, lPow, rPow;
	char line[1024];
	static int count = 0;
	static double old_dx = 0, old_dy = 0;
	struct displayList *q;

	static double old_ball_px, old_ball_py, old_self_px, old_self_py;

	char lport = MOTOR_A;
	char rport = MOTOR_B;

	// Variables for Defense

	static double sg_px = 0;
	static double sg_py = 0;
	static double og_px = 0;
	static double og_py = 0;

	if (ai->st.state == 0 || ai->st.state == 100 || ai->st.state == 200) { // Initial set up - find own, ball, and opponent blobs
	// Carry out self id process.
		fprintf(stderr, "Initial state, self-id in progress...\n");

		id_bot(ai, blobs);
		if ((ai->st.state % 100) != 0) { // The id_bot() routine will change the AI state to initial state + 1
			// if robot identification is successful.
			if (ai->st.self->cx >= 512)
				ai->st.side = 1;
			else
				ai->st.side = 0;
			BT_all_stop(0);

			ai->st.side = 0;

			fprintf(stderr, "Self-ID complete. Current position: (%f,%f), current heading: [%f, %f], blob direction=[%f, %f], AI state=%d\n", ai->st.self->cx, ai->st.self->cy, ai->st.smx, ai->st.smy, ai->st.sdx, ai->st.sdy, ai->st.state);

			if (ai->st.self != NULL) {
				// This checks that the motion vector and the blob direction vector
				// are pointing in the same direction. If they are not (the dot product
				// is less than 0) it inverts the blob direction vector so it points
				// in the same direction as the motion vector.
				if (((ai->st.smx * ai->st.sdx) + (ai->st.smy * ai->st.sdy)) < 0) {
					ai->st.self->dx *= -1.0;
					ai->st.self->dy *= -1.0;
					ai->st.sdx *= -1;
					ai->st.sdy *= -1;
				}
				old_dx = ai->st.sdx;
				old_dy = ai->st.sdy;

				angle_prediction = vector_to_angle(ai->st.sdx, ai->st.sdy);
				fprintf(stderr, "Start Angle Prediction: %.2f\n", angle_prediction);
//				exit(1);
			}

			if (ai->st.opp != NULL) {
				// Checks motion vector and blob direction for opponent. See above.
				if (((ai->st.omx * ai->st.odx) + (ai->st.omy * ai->st.ody)) < 0) {
					ai->st.opp->dx *= -1;
					ai->st.opp->dy *= -1;
					ai->st.odx *= -1;
					ai->st.ody *= -1;
				}
			}

			// update self goal position and opponent's goal position
			const double GOAL_OFFSET = 15.0;

			if (ai->st.side == 0) { // self goal on left side
				sg_px = 0.0 + GOAL_OFFSET;
				sg_py = 384.0;
				og_px = 1024.0;
				og_py = 384.0;
			} else { // self goal on right side
				sg_px = 1024.0 - GOAL_OFFSET;
				sg_py = 384.0;
				og_px = 0.0;
				og_py = 384.0;
			}

			// Notice that the ball's blob direction is not useful! only its
			// position and motion matter.
		}

	} else {
		/****************************************************************************
		 TO DO:
		 You will need to replace this 'catch-all' code with actual program logic to
		 implement your bot's state-based AI.

		 After id_bot() has successfully completed its work, the state should be
		 1 - if the bot is in SOCCER mode
		 101 - if the bot is in PENALTY mode
		 201 - if the bot is in CHASE mode

		 Your AI code needs to handle these states and their associated state
		 transitions which will determine the robot's behaviour for each mode.

		 Please note that in this function you should add appropriate functions below
		 to handle each state's processing, and the code here should mostly deal with
		 state transitions and with calling the appropriate function based on what
		 the bot is supposed to be doing.
		 *****************************************************************************/
		// keep tracking
		fprintf(stderr, "Tracking Agents...\n");  // bot, opponent, and ball.
		track_agents(ai, blobs);  // Currently, does nothing but endlessly track

		// Calibration
		if (1 == 0) {
//			BT_turn(MOTOR_A, -20, MOTOR_B, 20);

			if (ai->st.self != NULL) {
				struct blob *self = ai->st.self;
				int width = abs(self->x2 - self->x1);
				int height = abs(self->y2 - self->y1);

				double real_x = translate_distance(width);
				double real_y = translate_distance(height);

				fprintf(stderr, "Width: %d pixels (%d, %d) -> %.2f | Height: %d pixels (%d, %d) -> %.2f \n", width, self->x1, self->x2, real_x, height, self->y1, self->y2, real_y);
			} else {
				fprintf(stderr, "Unable to calibrate image. No robot found.\n");
			}

			return;
		}

		// play soccer mode
		if (ai->st.state < 100) {
			if (ai->st.state == 1) { // Initial State: Determine Whether to Attack or Defend

				double init_state = drand48();

				if (init_state <= 0.3) { // Defend (Init State -> 51)
					fprintf(stderr, "[State 1] Initial Mode: Defense [State 51]\n");
					sound_defend();
					ai->st.state = 2;
				} else { // Attack (Init State -> 2)
					fprintf(stderr, "[State 1] Initial Mode: Attack [State 2]\n");
					sound_attack();
					ai->st.state = 2;
//					ai->st.state = 2;
				}

			} else if (ai->st.state == 2) { // State 2: Fast Travel

				if (ai->st.self == NULL) { // If we can't see ourself, just wait.
					fprintf(stderr, "[State 2] Cannot see self. Waiting.\n");
					return;
				}

				if (ai->st.ball == NULL) { // If we can't see the ball, go defend instead.
					fprintf(stderr, "[State 2] Cannot see ball. Switching to defense [State 51].\n");
					sound_defend();
					ai->st.state = 51;
					return;
				}

				// Adjust the angle prediction based on the facing direction.
				adjust_angle_by_direction(ai->st.sdx, ai->st.sdy, MAX_DPS);

				// Calculate the vector from self to ball.
				double vector_x = ai->st.ball->cx - ai->st.self->cx;
				double vector_y = ai->st.ball->cy - ai->st.self->cy;

				// Calculate the angle that faces the ball based on the vector.
				double vector_angle = vector_to_angle(vector_x, vector_y);

				// Calculate the rotation required to face the ball from the current angle.
				double delta_angle = min_rotation(angle_prediction, vector_angle);
				double abs_delta_angle = fabs(delta_angle);

				// Calculate the pixel distance from self to the ball.
				double distance_pixel = magnitude(vector_x, vector_y);

				// Calculate the travel time from self to the ball (using fast travel).
				const double BALL_BARRIER = 5; // We want to appear close to the ball, not on top of it.
				int est_travel = estimate_travel_delay(distance_pixel, CPS_FAST, BALL_BARRIER);

				const double ERROR_THRESH = 20.0; // If it's within |T|, we don't care.
				const int TURN_SPEED = 20; // This is the speed of which we will turn at.

				if (abs_delta_angle >= ERROR_THRESH) { // We need to turn.
					int est_rotate = estimate_angle_delay(abs_delta_angle); // We estimate the amount of time that it takes to turn.

					if (delta_angle < 0) {
						fprintf(stderr, "[State 2] Turning to the right. Sleeping for %d ms.\n", est_rotate);
						BT_turn(LEFT_MOTOR, TURN_SPEED, RIGHT_MOTOR, -TURN_SPEED);
					} else {
						fprintf(stderr, "[State 2] Turning to the left. Sleeping for %d ms.\n", est_rotate);
						BT_turn(LEFT_MOTOR, -TURN_SPEED, RIGHT_MOTOR, TURN_SPEED);
					}

					msleep(est_rotate); // Sleep for the estimated time based on model.
					angle_prediction = vector_angle; // We change our current predicted angle to the one we've rotated to.
				} else { // No need to turn.
					fprintf(stderr, "[State 2] No need to turn. (%.2f) < (%.2f)\n", abs_delta_angle, ERROR_THRESH);
				}

				// Drive straight towards the ball.
				fprintf(stderr, "[State 2] Fast travel to the ball. Sleeping for %d ms.\n", est_travel);
				BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 100);

				// Sleep based on the estimated time model.
				msleep(est_travel);
				BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 0);

				// Wait for the camera to catch up. Switch states.
				fprintf(stderr, "[State 2] Wait for camera delay. Switch to [State 50]. Sleeping for %d ms.\n", CAMERA_DELAY);
				psuedo_sleep(CAMERA_DELAY);
				ai->st.state = 50;

			} else if (ai->st.state == 3) { // State 3: Position Ball and Kick

				if (ai->st.self == NULL) { // If we can't see ourself, just wait.
					fprintf(stderr, "[State 3] Cannot see self. Waiting.\n");
					return;
				}

				if (ai->st.ball == NULL) { // If we can't see the ball, go defend instead.
					fprintf(stderr, "[State 3] Cannot see ball. Switching to defense [State 51].\n");
					sound_defend();
					ai->st.state = 51;
					return;
				}

				// Adjust the angle prediction based on the facing direction.
				adjust_angle_by_direction(ai->st.sdx, ai->st.sdy, MAX_DPS);

				// Calculate the vector from self to ball.
				double vector_x = ai->st.ball->cx - ai->st.self->cx;
				double vector_y = ai->st.ball->cy - ai->st.self->cy;

				// Calculate the angle that faces the ball based on the vector.
				double vector_angle = vector_to_angle(vector_x, vector_y);

				// Calculate the rotation required to face the ball from the current angle.
				double delta_angle = min_rotation(angle_prediction, vector_angle);
				double abs_delta_angle = fabs(delta_angle);

				// Calculate the pixel distance from self to the ball.
				double distance_pixel = magnitude(vector_x, vector_y);

				// Calculate the travel time from self to the ball (using slow travel).
				const double BALL_BARRIER = 0; // This is precision. We want to appear on top of the ball.
				int est_travel = estimate_travel_delay(distance_pixel, CPS_SLOW, BALL_BARRIER);

				const double ERROR_THRESH = 10.0; // If it's within |T|, we don't care.
				const int TURN_SPEED = 20; // This is the speed of which we will turn at.

				if (abs_delta_angle >= ERROR_THRESH) { // We need to turn.
					int est_rotate = estimate_angle_delay(abs_delta_angle); // We estimate the amount of time that it takes to turn.

					if (delta_angle < 0) {
						fprintf(stderr, "[State 3] Turning to the right. Sleeping for %d ms.\n", est_rotate);
						BT_turn(LEFT_MOTOR, TURN_SPEED, RIGHT_MOTOR, -TURN_SPEED);
					} else {
						fprintf(stderr, "[State 3] Turning to the left. Sleeping for %d ms.\n", est_rotate);
						BT_turn(LEFT_MOTOR, -TURN_SPEED, RIGHT_MOTOR, TURN_SPEED);
					}

					msleep(est_rotate); // Sleep for the estimated time based on model.
					angle_prediction = vector_angle; // We change our current predicted angle to the one we've rotated to.
				} else { // No need to turn.
					fprintf(stderr, "[State 3] No need to turn. (%.2f) < (%.2f)\n", abs_delta_angle, ERROR_THRESH);
				}

				// Drive straight towards the ball.
				fprintf(stderr, "[State 3] Precise travel to the ball. Sleeping for %d ms.\n", est_travel);
				BT_drive(LEFT_MOTOR, RIGHT_MOTOR, TURN_SPEED);

				// Sleep based on the estimated time model.
				msleep(est_travel);
				BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 0);

				// Wait for the camera to catch up. Switch states.
				fprintf(stderr, "[State 3] Wait for camera delay. Switch to [State 5]. Sleeping for %d ms.\n", CAMERA_DELAY);
				psuedo_sleep(CAMERA_DELAY);
				ai->st.state = 5; // Camera delay, but ball could be invisible (due to camera).

			} else if (ai->st.state == 50) { // State 4: Wait for Camera to Catch Up

				long time_remaining = sleep_time - current_time_millis();

				if (time_remaining > 0) {
					fprintf(stderr, "[State 50] Waiting for camera delay. Time remaining: %d ms\n", time_remaining);
					return;
				}

				fprintf(stderr, "[State 50] Attempting to find next state.\n");

				if (ai->st.self == NULL) { // If we can't see ourself, just wait.
					fprintf(stderr, "[State 50] Cannot see self. Waiting.\n");
					return;
				}

				if (ai->st.ball == NULL) { // If we can't see the ball, go defend instead.
					fprintf(stderr, "[State 50] Cannot see ball. Switching to defense [State 51].\n");
					sound_defend();
					ai->st.state = 51;
					return;
				}

				// Adjust the angle prediction based on the facing direction.
				adjust_angle_by_direction(ai->st.sdx, ai->st.sdy, MAX_DPS);

				// Calculate the vector from self to ball.
				double vector_x = ai->st.ball->cx - ai->st.self->cx;
				double vector_y = ai->st.ball->cy - ai->st.self->cy;

				// Calculate the pixel distance from self to the ball.
				double distance_pixel = magnitude(vector_x, vector_y);
				double distance_cm = translate_distance(distance_pixel);

				// Distance threshold.
				const double DIST_THRESH = 25.0;

				if (distance_cm <= DIST_THRESH) { // We're close enough to the ball. Start fine tuning.
					fprintf(stderr, "[State 50] Distance is close to the ball (%.2f cm). Switch to [State 3].\n", distance_cm);
					ai->st.state = 3;
				} else { // We're still pretty far away from the ball.
					double ball_x = ai->st.ball->cx;
					double self_x = ai->st.self->cx;
					double abs_x = abs(ball_x - self_x);

					if (ai->st.side == 0) { // Our goal is on the left side.
						if (abs_x >= DIST_THRESH && self_x > ball_x) { // We are on the opposite side of the ball.
							fprintf(stderr, "[State 50] On opposite side of the ball (S: %.2f, B: %.2f). Switch to [State 51].\n", self_x, ball_x);
							sound_defend();
							ai->st.state = 51;
						} else { // We're still within range. Go back to fast kick.
							fprintf(stderr, "[State 50] Distance is close to the ball (%.2f cm). Switch to [State 2].\n", distance_cm);
							ai->st.state = 2;
						}
					} else { // Our goal is on the right side.
						if (abs_x >= DIST_THRESH && self_x < ball_x) { // We are on the opposite side of the ball.
							fprintf(stderr, "[State 50] On opposite side of the ball (S: %.2f, B: %.2f). Switch to [State 51].\n", self_x, ball_x);
							sound_defend();
							ai->st.state = 51;
						} else { // We're still within range. Go back to fast kick.
							fprintf(stderr, "[State 50] Distance is close to the ball (%.2f cm). Switch to [State 2].\n", distance_cm);
							ai->st.state = 2;
						}
					}
				}

			} else if (ai->st.state == 5) { // Wait for the camera, but ball could be invisible.

				long time_remaining = sleep_time - current_time_millis();

				if (time_remaining > 0) {
					fprintf(stderr, "[State 5] Waiting for camera delay. Time remaining: %d ms\n", time_remaining);
					return;
				}

				fprintf(stderr, "[State 5] Attempting to find next state.\n");

				if (ai->st.self == NULL) { // If we can't see ourself, just wait.
					fprintf(stderr, "[State 5] Cannot see self. Waiting.\n");
					return;
				}

				if (ai->st.ball == NULL) { // We are probably on top of the ball. This is good enough.
					fprintf(stderr, "[State 5] Cannot see ball. We are on top of it? Switch to [State 6].\n");
					ai->st.state = 6;
					return;
				}

				// Calculate the vector from self to ball.
				double vector_x = ai->st.ball->cx - ai->st.self->cx;
				double vector_y = ai->st.ball->cy - ai->st.self->cy;

				// Calculate the pixel distance from self to the ball.
				double distance_pixel = magnitude(vector_x, vector_y);
				double distance_cm = translate_distance(distance_pixel);

				// Distance threshold. We want to be on top.
				const double DIST_THRESH = 12.0;

				if (distance_cm <= DIST_THRESH) { // We're on top of the ball. Good enough.
					fprintf(stderr, "[State 5] We are on top of the ball. Switch to [State 6].\n");
					ai->st.state = 6;
					return;
				} else {
					fprintf(stderr, "[State 5] We are not close enough to the ball. Switch to [State 50].\n");
					ai->st.state = 50;
					return;
				}

			} else if (ai->st.state == 6) { // We are on top of the ball. We can penalty kick it now.

				fprintf(stderr, "[State 6] Performing Penalty Kick.\n");

				if (ai->st.self == NULL) { // If we can't see ourself, just wait.
					fprintf(stderr, "[State 6] Cannot see self. Waiting.\n");
					return;
				}

				sound_kick();

				// Adjust the angle prediction based on the facing direction.
				adjust_angle_by_direction(ai->st.sdx, ai->st.sdy, MAX_DPS);

				// Find the goal post.
				double goal_x, goal_y;
				goal_y = ARENA_HEIGHT / 2; // Center of the arena on the vertical.

				if (ai->st.side == 0) { // Our goal is on the left side.
					// We want to kick to the right side.
					goal_x = ARENA_WIDTH;
				} else { // Our goal is on the right side.
					// We want to kick to the left side.
					goal_x = 0;
				}

				// Calculate the vector from self to the goal.
				double vector_x = goal_x - ai->st.self->cx;
				double vector_y = goal_y - ai->st.self->cy;

				// Calculate the angle that faces the ball based on the vector.
				double vector_angle = vector_to_angle(vector_x, vector_y);

				// Calculate the rotation required to face the ball from the current angle.
				double delta_angle = min_rotation(angle_prediction, vector_angle);
				double abs_delta_angle = fabs(delta_angle);

				const double ERROR_THRESH = 20.0; // If it's within |T|, we don't care.
				const int TURN_SPEED = 20; // This is the speed of which we will turn at.

				if (abs_delta_angle >= ERROR_THRESH) { // We need to turn.
					int est_rotate = estimate_angle_delay(abs_delta_angle); // We estimate the amount of time that it takes to turn.

					if (delta_angle < 0) {
						fprintf(stderr, "[State 6] Turning to the right. Sleeping for %d ms.\n", est_rotate);
						BT_turn(LEFT_MOTOR, TURN_SPEED, RIGHT_MOTOR, -TURN_SPEED);
					} else {
						fprintf(stderr, "[State 6] Turning to the left. Sleeping for %d ms.\n", est_rotate);
						BT_turn(LEFT_MOTOR, -TURN_SPEED, RIGHT_MOTOR, TURN_SPEED);
					}

					msleep(est_rotate); // Sleep for the estimated time based on model.
					angle_prediction = vector_angle; // We change our current predicted angle to the one we've rotated to.
				} else { // No need to turn.
					fprintf(stderr, "[State 6] No need to turn. (%.2f) < (%.2f)\n", abs_delta_angle, ERROR_THRESH);
				}

				// Drive straight towards the ball.
				const int KICK_TIME = 1000; // This is the speed of which we will turn at.
				fprintf(stderr, "[State 6] Kicking the ball ball. Sleeping for %d ms.\n", KICK_TIME);
				BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 100);

				// Sleep based on the estimated time model.
				msleep(KICK_TIME);
				BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 0);

				// Wait for the camera to catch up. Switch states.
				fprintf(stderr, "[State 6] Wait for camera delay. Switch to [State 50]. Sleeping for %d ms.\n", CAMERA_DELAY);
				psuedo_sleep(CAMERA_DELAY);
				ai->st.state = 50;

			}
      // defending 
      else if (ai->st.state >= 51) {
//				ai->st.state = 50;
//				return;

				const double BARRIER_RADIUS = 30.0;
				const double BARRIER_ANGLE = 15.0;

				if (is_psuedo_sleeping() == 1) {
					fprintf(stderr, "[State %d] Psuedo Sleeping (Camera Lag)\n", ai->st.state);  // bot, opponent, and ball.
					return;
				}

				// If camera can't identify the ball, drive to self goal
				if (ai->st.ball == NULL && ai->st.state != 71 && ai->st.state != 72) {
					fprintf(stderr, "[State %d] Unable to find ball. Driving to goal [State 71].\n", ai->st.state);
					ai->st.state = 71;
					return;
				}

				if (ai->st.self != NULL) {
					int defend = -1; // We start by assuming that we don't need to defend.

					// Self heading angle
					adjust_angle_by_direction(ai->st.sdx, ai->st.sdy, MAX_DPS);
					const int speed = 20;

					/**************** Check for conditions if we need to keep defending ****************/
					// This is our target location, if the ball is between our robot and our goal
          // We'll make our robot go middle of the ball and edges
          // ie:
          // *************************************************
          // *                                               *
          // *                                               *
          // *                                               *
          // g            ball                               *
          // o             |                                 *
          // a             |              our                *
          // l     tar     |             robot               *
          // *-----loc-----|                                 *
          // *             |                                 *
          // *             |                                 *
          // *************************************************
					double def_tar_px, def_tar_py;
					def_tar_px = sg_px; // We want to head to our "own goal" by default.

					if (ai->st.ball->cy > 384.0) { // If ball on bottom side of the field, move self up
						def_tar_py = ai->st.ball->cy / 2.0;
					} else { // Otherwise, move self down.
						def_tar_py = ai->st.ball->cy + ((768.0 - ai->st.ball->cy) / 2.0);
					}

					// Check for ball's position. It must be between us and the goal.
          // different conditions based on our own side
					if (ai->st.side == 0 && ai->st.ball->cx < ai->st.self->cx) {
						def_tar_px = (ai->st.ball->cx) / 2.0;
						defend = 1;
						fprintf(stderr, "[State %d] Need to defend: ball is between goal and player.\n", ai->st.state);
					} else if (ai->st.side == 1 && ai->st.ball->cx > ai->st.self->cx) {
						def_tar_px = ai->st.ball->cx + (1024.0 - ai->st.ball->cx) / 2.0;
						defend = 1;
						fprintf(stderr, "[State %d] Need to defend: ball is between goal and player.\n", ai->st.state);
					}

					if (ai->st.opp == NULL) {
						fprintf(stderr, "States: Opponment is null.\n");
					} else {
						fprintf(stderr, "States: (%.2f, %.2f) (%.2f, %.2f) (%.2f, %.2f)\n", ai->st.ball->cx, ai->st.ball->cy, ai->st.self->cx, ai->st.self->cy, ai->st.opp->cx, ai->st.opp->cy);
					}

          // variables for self and target location
          double tar_self_dx = def_tar_px - ai->st.self->cx;
          double tar_self_dy = def_tar_py - ai->st.self->cy;
          double dist_tar_self = magnitude(tar_self_dx, tar_self_dy);
          double dist_tar_self_cm = translate_distance(dist_tar_self); // translate pixel to cm
          double tar_self_angle = vector_to_angle(tar_self_dx, tar_self_dy);
          double tar_self_rotation = min_rotation(angle_prediction, tar_self_angle);

          // variables for self and self goal (sg)
          double sg_self_x = sg_px - ai->st.self->cx;
          double sg_self_y = sg_py - ai->st.self->cy;
          double dist_sg_self = magnitude(sg_self_x, sg_self_y);
          double dist_sg_self_cm = translate_distance(dist_sg_self); // translate pixel to cm
          double sg_self_angle = vector_to_angle(sg_self_x, sg_self_y);
          double sg_self_rotation = min_rotation(angle_prediction, sg_self_angle);

					// Variables for ball and self status (vector and distance)
					double delta_x = ai->st.ball->cx - ai->st.self->cx;
					double delta_y = ai->st.ball->cy - ai->st.self->cy;
					double dist_ball_self = magnitude(delta_x, delta_y);
					double dist_ball_self_cm = translate_distance(dist_ball_self);
          double ball_self_angle = vector_to_angle(delta_x, delta_y);
          double ball_self_rotation = min_rotation(angle_prediction, ball_self_angle);


					// variables for ball and opponent status (vector and distance)
          double ball_opp_x = -1;
          double ball_opp_y = -1;
					double dist_ball_opp = -1;
          double dist_ball_opp_cm = -1;
          double ball_opp_angle = -1;
          double ball_opp_rotation = -1;

          // variables for self and oppponent status (vector and distance)
          double self_opp_x = -1;
          double self_opp_y = -1;
          double dist_self_opp; = -1;
          double dist_self_opp_cm = -1;
					double self_opp_angle = -1;
          double self_opp_rotation = -1;

          if (ai->st.opp != NULL) {
            // compute ball and opponent variables
            ball_opp_x = ai->st.ball->cx - ai->st.opp->cx;
            ball_opp_y = ai->st.ball->cy - ai->st.opp->cy;
						dist_ball_opp = magnitude(ball_opp_x, ball_opp_y);
            ball_opp_angle = vector_to_angle(ball_opp_x, ball_opp_y);
            ball_opp_rotation = min_rotation(angle_prediction, ball_opp_angle);

            // compute self and opponent variables
            self_opp_x = ai->st.opp->cx - ai->st.self->cx;
            self_opp_y = ai->st.opp->cy - ai->st.self->cy;
            dist_self_opp = magnitude(self_opp_x, self_opp_y);
            self_opp_angle = vector_to_angle(self_opp_x, self_opp_y);
            self_opp_rotation = min_rotation(angle_prediction, self_opp_angle);

						// Translate pixels to centimeters.
						dist_ball_opp_cm = translate_distance(dist_ball_opp);
            dist_self_opp_cm = translate_distance(dist_self_opp);

						// We need to defend if the ball is closer to the opponent than it is to us by more than 10 centimeters.
						if (dist_ball_self > dist_ball_opp + 10) {
							defend = 1;
							fprintf(stderr, "[State %d] Need to defend: ball closer to opponent (%.2f) than to self (%.2f).\n", ai->st.state, dist_ball_opp, dist_ball_self);
						} else {
							fprintf(stderr, "[State %d] Distance from ball: S (%.2f cm) | O (%.2f cm)\n", ai->st.state, dist_ball_self, dist_ball_opp);
						}
					} else {
						defend = 1; // switch to attack when there is no opponent?
					}

					// If we don't need to defend, switch to attack mode.
					if (defend != 1) {
						BT_all_stop(0);
						sound_attack();
						ai->st.state = 50;
						fprintf(stderr, "[State %d] Don't need to defend. Switch to attack mode [State 50].\n", ai->st.state);
						return;
					}

					// Check states for defending.
					if (ai->st.state == 51) {
						// Turning to the target location.

						// // This is the vector to the target location.
						// double tar_self_dx = def_tar_px - ai->st.self->cx;
						// double tar_self_dy = def_tar_py - ai->st.self->cy;

						fprintf(stderr, "[State 51] Target: (%.2f, %.2f) -> (%.2f, %.2f)\n", ai->st.self->cx, ai->st.self->cy, def_tar_px, def_tar_py);

						// This is the distance from the destination.
						// double tar_mag = magnitude(tar_self_dx, tar_self_dy);

						// This is the angle to the target location.
						// double tar_angle = vector_to_angle(tar_self_dx, tar_self_dy);
						// double rotation = min_rotation(angle_prediction, tar_angle);

						if (fabs(tar_self_rotation) < BARRIER_ANGLE) { // If we are facing the target location, drive forward.
							// drive to the location

							int travel_time = estimate_travel_delay(dist_tar_self, CPS_FAST, 0);
							BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 100);

							msleep(travel_time);
							BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 0);

							psuedo_sleep(CAMERA_DELAY);

							ai->st.state = 61;
							fprintf(stderr, "[State 51] Facing target destination. Switch to [State 61].\n");
						} else { // We are not facing the target location. Turn the robot.
							fprintf(stderr, "[State 51] Need to turn to the destination (%.2f -> %.2f). Remain [State 51].\n", angle_prediction, tar_self_rotation);
							int angle_time = estimate_angle_delay(tar_self_rotation);

							if (rotation < 0) {
								// rotate right
								BT_turn(LEFT_MOTOR, speed, RIGHT_MOTOR, -speed);
							} else {
								BT_turn(LEFT_MOTOR, -speed, RIGHT_MOTOR, speed);
							}

							fprintf(stderr, "[State 51] Sleeping for %d ms.\n", angle_time);

							msleep(angle_time);
							BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 0);

							angle_prediction = tar_self_angle;
							psuedo_sleep(CAMERA_DELAY);

							// Keep turning. The state remains the same.
							ai->st.state = 51;
						}

					} else if (ai->st.state == 61) {
						// Drive to target location.

						// Check if the bot still facing the correct angle.
						// double tar_self_dx = def_tar_px - ai->st.self->cx;
						// double tar_self_dy = def_tar_py - ai->st.self->cy;

						// Retrieve the angle that we need to face.
						// double tar_self_angle = vector_to_angle(tar_self_dx, tar_self_dy);
						// tar_self_angle = min_rotation(angle_prediction, tar_self_angle);

						// Check if we reached the target
						// double dist_tar_self = magnitude(tar_self_dx, tar_self_dy);
						// dist_tar_self = translate_distance(dist_tar_self);

						// Now we are between. Less than X centimeters.
						if (dist_tar_self_cm < BARRIER_RADIUS) {
							BT_all_stop(0);

							if (ai->st.side == 0) { // Own side on left, switch to attack mode.
								if (ai->st.ball->cx > (512 + 10)) {
									sound_attack();
									ai->st.state = 50;
									fprintf(stderr, "[State 61] Ball is on opponent's side. Switch to [State 50].\n");
								} else { // keep defending
									ai->st.state = 71;
									fprintf(stderr, "[State 61] We are at destination. Drive to goal from [State 71].\n");
								}
							} else { // Own side on right, switch to attack mode.
								if (ai->st.ball->cx < (512 - 10)) {
									sound_attack();
									ai->st.state = 50;
									fprintf(stderr, "[State 61] Ball is on opponent's side. Switch to [State 50].\n");
								} else {
									ai->st.state = 71;
									fprintf(stderr, "[State 61] We are at destination. Drive to goal from [State 71].\n");
								}
							}
						} else if (fabs(tar_self_angle) < BARRIER_ANGLE) { // If still in the angle, keep driving.
							// This is the distance from the destination.
							// double tar_mag = magnitude(tar_self_dx, tar_self_dy);
							int travel_time = estimate_travel_delay(dist_tar_self, CPS_FAST, 0);

							// Drive to the location
							BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 100);

							msleep(travel_time);
							BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 0);

							psuedo_sleep(CAMERA_DELAY);

							ai->st.state = 61;
							fprintf(stderr, "[State 61] Keep driving to the target location. Remain on [State 61].\n");
						} else { // If not, stop and correct angle
							BT_all_stop(0);
							ai->st.state = 71;
							fprintf(stderr, "[State 61] Arrive at target location. Switch to [State 71].\n");
						}

					} 
          // TODO: drive to goal
          else if (ai->st.state == 71) { // Defending the ball when it's on own side, move to the goal.
						// double dx = sg_px - ai->st.self->cx;
						// double dy = sg_py - ai->st.self->cy;

						// double target_angle = vector_to_angle(dx, dy);
						// double rotation = min_rotation(angle_prediction, target_angle);

						if (fabs(sg_self_rotation) < BARRIER_ANGLE) {
							// This is the distance from the destination.
							// double tar_mag = magnitude(dx, dy);
							int travel_time = estimate_travel_delay(dist_sg_self, CPS_FAST, 0);

							// Drive to the location
							BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 100);

							msleep(travel_time);
							BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 0);

							psuedo_sleep(CAMERA_DELAY);

							ai->st.state = 72;
							fprintf(stderr, "[State 71] Angle is aligned to destination. Switch to [State 72].\n");
						} else {
							int angle_time = estimate_angle_delay(sg_self_rotation);

							if (rotation < 0) { // Rotate right
								BT_turn(LEFT_MOTOR, speed, RIGHT_MOTOR, -speed);
							} else { // Rotate left
								BT_turn(LEFT_MOTOR, -speed, RIGHT_MOTOR, speed);
							}

							msleep(angle_time);
							BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 0);

							angle_prediction = sg_self_angle;
							psuedo_sleep(CAMERA_DELAY);

							// Keep turning.
							ai->st.state = 71;
							fprintf(stderr, "[State 71] Keep driving to the target location. Remain on [State 71].\n");
						}
					} else if (ai->st.state == 72) {
						// Drive to the goal.
						// double dx = sg_px - ai->st.self->cx;
						// double dy = sg_py - ai->st.self->cy;

						// double tar_angle = vector_to_angle(dx, dy);
						// tar_angle = min_rotation(angle_prediction, tar_angle);

						// Check if we reached the target
						// double dist_tar_self = magnitude(dx, dy);
						// dist_tar_self = translate_distance(dist_tar_self);

						// We are at the goal
						if (dist_sg_self_cm < BARRIER_RADIUS) {
							BT_all_stop(0);

							// Own side on left
							ai->st.state = 81;
							fprintf(stderr, "[State 72] We are at the goal. Switch to [State 81].\n");
						} else if (fabs(sg_self_angle) < BARRIER_ANGLE) { // If still in the angle, keep driving
							// double tar_mag = magnitude(dx, dy);
							int travel_time = estimate_travel_delay(dist_sg_self, CPS_FAST, 0);

							// Drive to the location
							BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 100);

							msleep(travel_time);
							BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 0);

							psuedo_sleep(CAMERA_DELAY);

							ai->st.state = 72;
							fprintf(stderr, "[State 72] Keep driving to the target location. Remain on [State 72].\n");
						} else {// not close and not in angle
							BT_all_stop(0);
							ai->st.state = 71;

							fprintf(stderr, "[State 72] Angle is not aligned. Switch back to [State 71].\n");
						}
					} else if (ai->st.state == 81) {
						// Reached to our goal in defend mode
						// Turn to the ball
						// double dx = ai->st.ball->cx - ai->st.self->cx;
						// double dy = ai->st.ball->cy - ai->st.self->cy;

						// double target_angle = vector_to_angle(dx, dy);
						// double rotation = min_rotation(angle_prediction, target_angle);

						// // Check if we reached the target
						// double dist_tar_self = magnitude(dx, dy);
						// dist_tar_self = translate_distance(dist_tar_self);

						if (fabs(ball_self_angle) < BARRIER_ANGLE) {
							// double tar_mag = magnitude(dx, dy);
							int travel_time = estimate_travel_delay(dist_ball_self, CPS_FAST, 0);

							// Drive to the location
							BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 100);

							// Move to ball in increments.
							travel_time = (int) fmin(travel_time, 500);
							msleep(travel_time);

							BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 0);

							psuedo_sleep(CAMERA_DELAY);

							ai->st.state = 82;
							fprintf(stderr, "[State 81] Drive to the ball. Switch to [State 82].\n");
						} else {
							int angle_time = estimate_angle_delay(ball_self_angle);

							if (ball_self_angle < 0) {
								// rotate right
								BT_turn(LEFT_MOTOR, speed, RIGHT_MOTOR, -speed);
							} else {
								BT_turn(LEFT_MOTOR, -speed, RIGHT_MOTOR, speed);
							}

							msleep(angle_time);
							BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 0);

							angle_prediction = ball_self_angle;
							psuedo_sleep(CAMERA_DELAY);

							// keep turnning
							ai->st.state = 81;
							fprintf(stderr, "[State 81] We need to turn to the ball. Remain on [State 81].\n");
						}
					} else if (ai->st.state == 82) {
						// double dx = ai->st.ball->cx - ai->st.self->cx;
						// double dy = ai->st.ball->cy - ai->st.self->cy;

						// double tar_angle = vector_to_angle(dx, dy);

						fprintf(stderr, "[State 82] C: %.2f T: %.2f\n", angle_prediction, ball_self_angle);

						// tar_angle = min_rotation(angle_prediction, tar_angle);

						// // Check if we reached the target
						// double d_pixel = magnitude(dx, dy);
						// double d_cm = translate_distance(d_pixel);

						if (dist_ball_self_cm < BARRIER_RADIUS) { // If we are close to the ball, switch to attack.
							BT_all_stop(0);
							// Own side on left.
							sound_attack();
							ai->st.state = 50;
						} else if (fabs(ball_self_angle) < BARRIER_ANGLE) { // If still in the angle, keep driving
							int travel_time = estimate_travel_delay(dist_ball_self, CPS_FAST, 0);

							// Drive to the location
							BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 100);

							travel_time = (int) fmin(500, travel_time);
							msleep(travel_time);

							BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 0);

							psuedo_sleep(CAMERA_DELAY);

							ai->st.state = 82;
							fprintf(stderr, "[State 81] Drive to the ball. Remain on [State 82].\n");
						} else { // if not, stop and correct angle
							BT_all_stop(0);
							ai->st.state = 81;
							fprintf(stderr, "[State 81] We need to turn to the ball. Switch to [State 81].\n");
						}
					} else {
						fprintf(stderr, "What state is that? Switching from Defense [State %d] to Attack [State 50].\n", ai->st.state);
						sound_attack();
						ai->st.state = 50;
					}
				}
			}
		}
		// penalty kick mode
		else if (ai->st.state < 200) {
			if (ai->st.self != NULL && ai->st.ball != NULL) {
				fprintf(stderr, "penalty kick mode, state: %d\n", ai->st.state);
				// thresholds
				double dis_tar_th = 200;
				// positions
				// identify opponents goal side
				int goal_px, goal_py;
				goal_py = 384;
				if (ai->st.side == 0) {
					goal_px = 1024;
				} else {
					goal_px = 0;
				}
				// find location to kick the ball
				double tar_px, tar_py;
				double tar_dx, tar_dy;
				tar_dx = ai->st.old_bcx - goal_px;
				tar_dy = ai->st.old_bcy - goal_py;
				normalize_v(&tar_dx, &tar_dy);
				tar_px = ai->st.old_bcx + 400 * tar_dx;
				tar_py = ai->st.old_bcy + 400 * tar_dy;
				// if(!valid_location(tar_px, tar_py)){
				// tar_px = 125;
				// tar_py = 384;
				// }
				double tar_self_dx, tar_self_dy;
				tar_self_dx = tar_px - ai->st.old_scx;
				tar_self_dy = tar_py - ai->st.old_scy;
				normalize_v(&tar_self_dx, &tar_self_dy);
				double c_theta_tar, dist_tar;
				c_theta_tar = dottie(tar_self_dx, tar_self_dy, old_dx, old_dy);
				dist_tar = get_distance(tar_px, tar_py, ai->st.old_scx, ai->st.old_scy);
				fprintf(stderr, "goal loc: %d %d\n", goal_px, goal_py);
				fprintf(stderr, "tar_loc: (%f %f) | c_tar_theta: %f  | tar_dist: %f\n", tar_px, tar_py, c_theta_tar, dist_tar);
				double s_h_dx, s_h_dy;
				s_h_dx = ai->st.sdx;
				s_h_dy = ai->st.sdy;
				// check if it's correctted heading direction
				if (dottie(old_dx, old_dy, ai->st.sdx, ai->st.sdy) < 0) {
					fprintf(stderr, "suck hd reading, correcting\n");
					ai->st.self->dx *= -1.0;
					ai->st.self->dy *= -1.0;
					ai->st.sdx *= -1;
					ai->st.sdy *= -1;
					old_dx = ai->st.sdx;
					old_dy = ai->st.sdy;
					s_h_dx = ai->st.sdx;
					s_h_dy = ai->st.sdy;
				}
				switch (ai->st.state) {

				case 101: // turn to target location
					if (c_theta_tar < 0) {

					}
					break;
				case 111: // drive to target location
					break;
				default:
					break;
				}
			}
		}
		// chase ball mode
		else if (ai->st.state < 300) {
			if (ai->st.self != NULL && ai->st.ball != NULL) {
				fprintf(stderr, "chasing ball, state: %d\n", ai->st.state);
				// thresholds
				double theta_th = 0.95;
				double dis_th = 200;
				// variables needed for turn to the ball
				double self_px, self_py, ball_px, ball_py;
				// ball/self position (robust needed)
				self_px = ai->st.old_scx;
				self_py = ai->st.old_scy;
				ball_px = ai->st.old_bcx;
				ball_py = ai->st.old_bcy;
				// ball_self dirction and self heading direction
				double b_s_dx, b_s_dy, s_h_dx, s_h_dy;
				b_s_dx = ball_px - self_px;
				b_s_dy = ball_py - self_py;
				s_h_dx = ai->st.sdx;
				s_h_dy = ai->st.sdy;
				if (dottie(old_dx, old_dy, ai->st.sdx, ai->st.sdy) < 0) {
					fprintf(stderr, "suck hd reading, correcting\n");
					ai->st.self->dx *= -1.0;
					ai->st.self->dy *= -1.0;
					ai->st.sdx *= -1;
					ai->st.sdy *= -1;
					old_dx = ai->st.sdx;
					old_dy = ai->st.sdy;
					s_h_dx = ai->st.sdx;
					s_h_dy = ai->st.sdy;
				}
				normalize_v(&b_s_dx, &b_s_dy);
				normalize_v(&s_h_dx, &s_h_dy);
				// angle and distance to the ball
				double c_bs_theta, dis_ball_self;
				c_bs_theta = dottie(b_s_dx, b_s_dy, s_h_dx, s_h_dy);
				dis_ball_self = get_distance(self_px, self_py, ball_px, ball_py);
				// start state transition
				switch (ai->st.state) {
				case 201: // turn to the ball
					fprintf(stderr, "turning to the ball, parameters:\n");
					fprintf(stderr, "self p: (%f, %f) | ball p: (%f, %f)\n", self_px, self_py, ball_px, ball_py);
					fprintf(stderr, "hd v: (%f, %f) | bs v: (%f, %f)\n", s_h_dx, s_h_dy, b_s_dx, b_s_dy);
					fprintf(stderr, "angle: %f \n", c_bs_theta);

					if (c_bs_theta < 0) {
						count = 0;
						chase_ball_turn_180_s202(18);
						ai->st.state = 202;
					}

					else if (c_bs_theta > theta_th) {
						fprintf(stderr, "facing the ball--------c_theta: %f\n", c_bs_theta);
						// // uncomment for debuging turn to the ball behavior
						// BT_all_stop(0);
						// ai->st.state = 201;

						chase_ball_drive_s211(15);
						ai->st.state = 211;
					} else {
						// keep turning towards ball, stay in same state
						chase_ball_turn_s201(ai, s_h_dx, s_h_dy, b_s_dx, b_s_dy);
						ai->st.state = 201;
					}
					break;
				case 202: // turn around, when cos theta < -0.2 (<0)
					if (count < 15) {
						count += 1;
						chase_ball_turn_180_s202(18);
						ai->st.state = 202;
					} else {
						count = 0;
						chase_ball_mv_s241(8);
						ai->st.state = 241;
					}
					break;
				case 211: // drive to the ball
					// run off angle
					if (c_bs_theta < theta_th) {
						chase_ball_turn_s201(ai, s_h_dx, s_h_dy, b_s_dx, b_s_dy);
						ai->st.state = 201;
					}
					// reached the ball
					else if (dis_ball_self < dis_th) {
						chase_ball_reached_s221();
						ai->st.state = 221;
					}
					// keep driving
					else {
						chase_ball_drive_s211(15);
						ai->st.state = 211;
					}
					break;
				case 221: // kick the ball
					chase_ball_reached_s221();
					ai->st.state = 201;
					break;
				case 241: // moving forward to check hd direction
					if (count < 5) {
						count += 1;
						chase_ball_mv_s241(8);
						ai->st.state = 241;
					} else {
						count = 0;
						adjust_heading_v(ai, &old_dx, &old_dy);
						ai->st.state = 242;
					}
					break;
				case 242:
					count = 0;
					chase_ball_mv_bk_s243(8);
					ai->st.state = 243;
					break;
				case 243:
					if (count < 5) {
						count += 1;
						chase_ball_mv_bk_s243(8);
						ai->st.state = 243;
					} else {
						count = 0;
						chase_ball_turn_s201(ai, s_h_dx, s_h_dy, b_s_dx, b_s_dy);
						ai->st.state = 201;
					}
					break;
				default:
					fprintf(stderr, "chasing -> unkown state....default to turnning ball\n");
					chase_ball_turn_s201(ai, s_h_dx, s_h_dy, b_s_dx, b_s_dy);
					ai->st.state = 201;
					break;
				} // end of switch state checking for chase ball
			} // end of self/ball existance check for chase ball
		} // end of chase ball
		  // unexpected state
		else {
			fprintf(stderr, "invalid state detected, state: %d\n", ai->st.state);
			exit(1);
		}
	}
}

/**********************************************************************************
 TO DO:

 Add the rest of your game playing logic below. Create appropriate functions to
 handle different states (be sure to name the states/functions in a meaningful
 way), and do any processing required in the space below.

 AI_main() should *NOT* do any heavy lifting. It should only call appropriate
 functions based on the current AI state.

 You will lose marks if AI_main() is cluttered with code that doesn't belong
 there.
 **********************************************************************************/

/**************   Chasing ball mode state functions *****************/
void chase_ball_turn_s201(struct RoboAI *ai, double shdx, double shdy, double bsx, double bsy) {
	if (crossie_sign(shdx, shdy, bsx, bsy) > 0) {
		fprintf(stderr, "turn left\n");
		turn_left(8);
	} else {
		fprintf(stderr, "turn right\n");
		turn_right(8);
	}
}

void chase_ball_turn_180_s202(int pw) {
	turn_right(pw);
}

void chase_ball_drive_s211(int pw) {
	move_forward(pw);
}

void chase_ball_reached_s221() {
	move_forward(100);
}

void chase_ball_mv_s241(int pw) {
	move_forward(pw);
}

void chase_ball_mv_bk_s243(int pw) {
	move_forward(-pw);
}

void adjust_angle_by_direction(double dir_x, double dir_y, double thresh) {
	double angle_a = vector_to_angle(dir_x, dir_y);
	double angle_b = inverse_angle(angle_a);

	double dist_a = fabs(angle_a - angle_prediction);
	double dist_b = fabs(angle_b - angle_prediction);

	if (dist_a <= thresh) {
		fprintf(stderr, "Adjusting Angle Prediction %.2f -> %.2f (%.2f)\n", angle_prediction, angle_a, thresh);
		angle_prediction = angle_a;
	} else if (dist_b <= thresh) {
		fprintf(stderr, "Adjusting Angle Prediction %.2f -> %.2f (%.2f)\n", angle_prediction, angle_b, thresh);
		angle_prediction = angle_b;
	} else {
		fprintf(stderr, "Angle Prediction Unchanged %.2f -> (%.2f, %.2f)\n", angle_prediction, angle_a, angle_b);
	}
}
