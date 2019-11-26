/***************************************************
 CSC C85 - UTSC RoboSoccer AI core

 This file contains the definition of the AI data
 structure which holds the state of your bot's AI.

 You must become familiar with this structure and
 its contents.

 You will need to modify this file to add headers
 for any functions you added to implemet the
 soccer playing functionality of your bot.

 Be sure to document everything you do thoroughly.

 AI scaffold: Parker-Lee-Estrada, Summer 2013
 Updated by F. Estrada, Aug 2019

***************************************************/

#ifndef _ROBO_AI_H
#define _ROBO_AI_H

#include "imagecapture/imageCapture.h"
#include "API/btcomm.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

//update to actual motors
#define LEFT_MOTOR MOTOR_A
#define RIGHT_MOTOR MOTOR_B

#define AI_SOCCER 0   // Play soccer!
#define AI_PENALTY 1    // Go score some goals!
#define AI_CHASE 2  // Kick the ball around and chase it!

#define NOISE_VAR 5.0                     // Minimum amount of displacement considered NOT noise.
#define ANGLE_DIFF_THRESH   0.077479      // Successful alignment threshold in Radians - About 4.7 degrees
#define DRIVE_INIT_THRESH   0.043633      // About 10 degrees
#define LOC_THRESHOLD       25            // Location accuracy error
#define MIN_DRIVE_POWER     25
#define MAX_DRIVE_POWER     100
#define TURN_ADJ_POWER      25

struct AI_data {
  // This data structure is used to hold all data relevant to the state of the AI.
  // This includes, of course, the current state, as well as the status of
  // our own bot, the opponent (if present), and the ball (if present).
  // For each agent in the game we keep a pointer to the blob that corresponds
  // to the agent (see the blob data structure in imageCapture.h), and data
  // about its old position, as well as current velocity and heading vectors.
  //
  // MIND THE NOISE.

  // Robot's playfield side id (w.r.t. the viepoint of the camera).
  int side;   // side=0 implies the robot's own side is the left side
  // side=1 implies the robot's own side is the right side
  // This is set based on the robot's initial position
  // on the field
  int botCol;   // Own bot's colour. 0 - green, 1 - red

  int state;    // Current AI state

  // Object ID status for self, opponent, and ball. Just boolean
  // values indicating whether blobs have been found for each of these
  // entities.
  int selfID;
  int oppID;
  int ballID;

  // Blob track data. Ball likely needs to be detected at each frame
  // separately. So we keep old location to estimate v
  struct blob *ball;    // Current ball blob
  double old_bcx, old_bcy;  // Previous ball cx,cy
  double bvx, bvy;      // Ball velocity vector
  double bmx, bmy;      // Ball motion vector
  double bdx, bdy;                // Ball heading direction (from blob shape)

  // Self track data. Done separately each frame
  struct blob *self;    // Current self blob
  double old_scx, old_scy;  // Previous self (cx,cy)
  double svx, svy;      // Current self [vx vy]
  double smx, smy;      // Self motion vector
  double sdx, sdy;                // Self heading direction (from blob shape)

  // Opponent track data. Done separately each frame
  struct blob *opp;   // Current opponent blob
  double old_ocx, old_ocy;  // Previous opponent (cx,cy)
  double ovx, ovy;      // Current opponent [vx vy]
  double omx, omy;      // Opponent motion vector
  double odx, ody;                // Opponent heading direction (from blob shape)
};

struct RoboAI {
  // Main AI data container. It allows us to specify which function
  // will handle the AI, and sets up a data structure to store the
  // AI's data (see above).
  void (* runAI)(struct RoboAI *ai, struct blob *, void *state);
  void (* calibrate)(struct RoboAI *ai, struct blob *);
  struct AI_data st;
  struct displayList *DPhead;
};

/**
 * \brief Set up an AI structure for playing roboSoccer
 *
 * Set up an AI structure for playing roboSoccer. Must be
 * called before using the AI structure during gameplay.
 * \param[in] mode The operational mode for the AI
 * \param[out] ai A structure containing data necessary for
 *    AI algorithms
 * \pre ai is uninitialized
 * \post ai is set up for use, and must be cleaned up using
 *    cleanupAI
 */
int setupAI(int mode, int own_col, struct RoboAI *ai);

/**
 * \brief Top-level AI loop.
 *
 * Decides based on current state and blob configuration what
 * the bot should do next, and calls the appropriate behaviour
 * function.
 *
 * \param[in] ai, pointer to the data structure for the running AI
 * \param[in] blobs, pointer to the current list of tracked blobs
 * \param[out] void, but the state description in the AI structure may have changed
 * \pre ai is not NULL, blobs is not NULL
 * \post ai is not NULL, blobs is not NULL
 */
void AI_main(struct RoboAI *ai, struct blob *blobs, void *state);

// Calibration stub
void AI_calibrate(struct RoboAI *ai, struct blob *blobs);

/* PaCode - just the function headers - see the functions for descriptions */
void id_bot(struct RoboAI *ai, struct blob *blobs);
struct blob *id_coloured_blob2(struct RoboAI *ai, struct blob *blobs, int col);
void track_agents(struct RoboAI *ai, struct blob *blobs);

// Display List functions
// the AI data structure provides a way for you to add graphical markers on-screen,
// the functions below add points or lines at a specified location and with the
// colour you want. Items you add will remain there until cleared. Do not mess
// with the list directly, use the functions below!
// Colours are specified as floating point values in [0,255], black is [0,0,0]
// white is [255,255,255].
struct displayList *addPoint(struct displayList *head, int x, int y, double R, double G, double B);
struct displayList *addLine(struct displayList *head, int x1, int y1, int x2, int y2, double R, double G, double B);
struct displayList *addVector(struct displayList *head, int x1, int y1, double dx, double dy, int length, double R, double G, double B);
struct displayList *addCross(struct displayList *head, int x, int y, int length, double R, double G, double B);
struct displayList *clearDP(struct displayList *head);

/****************************************************************************
 TO DO:
   Add headers for your own functions implementing the bot's soccer
   playing functionality below.
*****************************************************************************/

// v x u > 0 -> v on the right of u
// v x u < 0 -> v on the left of u
inline double crossie_sign(double vx, double vy, double ux, double uy) {
  if ((vx * uy) - (ux * vy) < 0) return -1;
  else return 1;
}

inline double dottie(double vx, double vy, double ux, double uy) {
  // Returns the dot product of the two vectors [vx,vy] and [ux,uy]
  return (vx * ux) + (vy * uy);
}

/***** state functions  *****/
// chase ball states
void chase_ball(struct RoboAI *ai, struct blob *blobs, void *state);
void chase_ball_turn_s201(struct RoboAI *ai, double shdx, double shdy, double bsx, double bsy);
void chase_ball_turn_180_s202(int pw);
void chase_ball_drive_s211(int pw);
void chase_ball_reached_s221();
void chase_ball_mv_forward_s231(struct RoboAI *ai, struct blob *blobs, void *state);
void chase_ball_check_hd_s232(struct RoboAI *ai, struct blob *blobs, void *state);
void chase_ball_mv_s241(int pw);
void chase_ball_mv_bk_s243(int pw);

// penalty kick states
void p_kick_turn_to_tar_s101(struct RoboAI *ai, double shdx, double shdy, double bsx, double bsy);
void p_kick_drive_to_tar_s111(struct RoboAI *ai, double tarx, double tary, int *drive_fdc);
void p_kick_turn_mv_frame_s131(struct RoboAI *ai);
void p_kick_adj_hd_s132(struct RoboAI *ai);
void p_kick_turn_mv_bk_s131(struct RoboAI *ai);
void p_kick_turn_around_s141(struct RoboAI *ai);
// action functions
inline void move_forward(int pw) {
  BT_drive(LEFT_MOTOR, RIGHT_MOTOR, pw);
}

inline void turn_left(int pw) {
  BT_turn(LEFT_MOTOR, -pw, RIGHT_MOTOR, pw);
}

inline void turn_right(int pw) {
  BT_turn(LEFT_MOTOR, pw, RIGHT_MOTOR, -pw);
}

// helper functions
/**
 * normalize_v a givien vector [x, y]
 * @param x x coordinate
 * @param y y coordinate
 */
inline void normalize_v(double *x, double *y) {
  double vl = 1 / (sqrt(pow(*x, 2) + pow(*y, 2)));
  *x = (*x) * vl;
  *y = (*y) * vl;
}
/**
 * predicts soccor's position in the next frame
 * @param ai ai
 * @param x  x coordinate in next frame
 * @param y  y coordinate in next frame
 */
inline void pred_ball_position(struct RoboAI *ai, double *x, double *y) {
  *x = ai->st.ball->cx + (ai->st.ball->cx - ai->st.old_bcx);
  *y = ai->st.ball->cy + (ai->st.ball->cy - ai->st.old_bcy);
}

inline double get_distance(double cx1, double cy1, double cx2, double cy2) {
  return sqrt(pow((cx1 - cx2), 2) + pow((cy1 - cy2), 2));
}

inline void self_ball_angle(struct RoboAI *ai, double *theta) {
  double self_ball_vx = ai->st.old_bcx - ai->st.old_scx;
  double self_ball_vy = ai->st.old_bcy - ai->st.old_scy;
  // normalize ball to self position vector
  normalize_v(&self_ball_vx, &self_ball_vy);

  *theta = dottie(self_ball_vx, self_ball_vy, ai->st.sdx, ai->st.sdy);
}

inline void ball_self_vector(struct RoboAI *ai, double *x, double *y) {
  *x = ai->st.old_bcx - ai->st.old_scx;
  *y = ai->st.old_bcy - ai->st.old_scy;
}



inline double dis_threshold(struct RoboAI *ai) {
  if(ai->st.self != NULL && ai->st.ball != NULL) {
    return fabs(((ai->st).ball->x2) - ((ai->st).ball->x1)) + fabs(((ai->st).self->x2) - ((ai->st).self->x1));
  } else {
    return -1;
  }
}

/**
 * check if a given x y coordinate is in the play field
 * play field in range of 1024 x 768
 * @param  x x coordinate
 * @param  y y coordinate
 * @return   if (x,y) in (1024, 768)
 */
inline int valid_location(double x, double y) {
  return x >= 0 && x <= 1024 && y >= 0 && y <= 768;
}


/**
 * adjust heading direction of self robot by checking moving direction with heading direction
 * @param ai ai
 */
inline void adjust_heading_v(struct RoboAI *ai, double *old_dx, double *old_dy) {
  if (ai->st.self != NULL) {
    fprintf(stderr, ">adjusting heading<\n");
    fprintf(stderr, "before-> hd: (%f, %f), md: (%f, %f)\n", ai->st.sdx, ai->st.sdy, ai->st.smx, ai->st.smy);
    if (((ai->st.smx * ai->st.sdx) + (ai->st.smy * ai->st.sdy)) < 0) {
      ai->st.self->dx *= -1.0;
      ai->st.self->dy *= -1.0;
      ai->st.sdx *= -1;
      ai->st.sdy *= -1;
    }
    *old_dx = ai->st.sdx;
    *old_dy = ai->st.sdy;
  }
  fprintf(stderr, "after-> hd: (%f, %f), md: (%f, %f)\n", ai->st.sdx, ai->st.sdy, ai->st.smx, ai->st.smy);
  if (ai->st.opp != NULL) {
    // Checks motion vector and blob direction for opponent. See above.
    if (((ai->st.omx * ai->st.odx) + (ai->st.omy * ai->st.ody)) < 0) {
      ai->st.opp->dx *= -1;
      ai->st.opp->dy *= -1;
      ai->st.odx *= -1;
      ai->st.ody *= -1;
    }
  }

}

/*  queue functions */
inline void resetQ(double *q, int size, double value) {
  for (int i = 0; i < size; i++) {
    q[i] = value;
  }
}


inline void updateQ(double *queue, int size, double n) {
  for (int i = 1; i < size; i++) {
    queue[i - 1] = queue[i];
  }
  queue[size - 1] = n;
}

inline int initializedQ(double *q, int size){
  for (int i = 0; i < size; i++)
  {
    if(q[i] < 10){
      return -1;
    }
  }
  return 1;
}
inline double slopeQ(double *q, int size) {
  double sum = 0;
  for (int i = (size - 1); i > 0; i--) {
    sum += (q[i] - q[i - 1]);
  }
  return sum / (size - 1);
}
/**
 * predict the next n incoming value based on the rate of change of elements in the quene
 * @param  queue  quene
 * @param  size   quene size
 * @param  pred_n next n'th value to be predicted
 * @return        predicted value
 */
inline double predictQ(double *queue, int size, int pred_n, double lb, double ub) {
  // find slop in the queue
  double slope = slopeQ(queue, size);
  double prd = queue[size - 1] + slope;
  if(prd < lb) {
    return lb;
  } else if(prd > ub) {
    return ub;
  } else {
    return prd;
  }
}

inline double robustQ(double *q, int size, double v) {
  // find slop in the queue
  double prd = predictQ(q, size, 1, -1, 1);
  if((prd - v) > (1.5 * slopeQ(q, size))) {
    fprintf(stderr, "?noisy reading %f, return %f \n", v, prd);
    return prd;
  } else {
    return v;
  }
}
#endif
