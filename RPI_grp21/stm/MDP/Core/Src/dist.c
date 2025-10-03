#include "dist.h"

static KalmanParams kParams;
static DistState state;

void dist_track_init() {
	dist_reset(0);
}

void dist_reset(float v) {
	kalman_init(&kParams, 0, DIST_S_ACCEL, DIST_S_MOTOR);
	state.dist = 0;
	state.v = v;
	state.s_v = 0;
}

static void dist_update_state(float msElapsed, float accel) {
	state.dist += state.v * msElapsed;
	state.v += accel * msElapsed;

	//update uncertainties.
	kParams.s_est += msElapsed * msElapsed * state.s_v;
	state.s_v += msElapsed * msElapsed * DIST_S_ACCEL;
}

float dist_get_cm(float msElapsed, float accel, float motorDist) {
	//get raw estimate, and update uncertainty.
	dist_update_state(msElapsed, accel);

	//get improved estimate.
	kalman_update(&kParams, state.dist, motorDist);
	state.dist = kParams.last_est;

	return state.dist;
}
