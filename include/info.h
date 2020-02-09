// angle conversion macros
#define RAD2DEG(rad) ((rad) * 180./M_PI)
#define DEG2RAD(deg) ((deg) * M_PI/180.)

// velocity and movement limits
#define FREE_ENV_VEL 0.25   // [m/s]
#define OBST_DET_VEL 0.1    // [m/s]
#define MAX_ANG_VEL M_PI/6  // [rad/s]
#define OBST_DIST_THRESH 0.5// [m]