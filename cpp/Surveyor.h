// ==============================================================
// Some vessel parameters
// ==============================================================
const double TRI32=sqrt(3.0)/2.0;
const double LANDER_EMPTY_MASS = 289.10; //Basic bus plus payload minus AMR minus retro case
const double RETRO_EMPTY_MASS = 64.88;
const double AMR_MASS = 3.82;

const double RETRO_PROP_MASS=560.64;
const double RETRO_THRUST = 39140;
const double RETRO_BURNTIME = 40.5;
const double RETRO_ITOT   = RETRO_THRUST*RETRO_BURNTIME;
const double RETRO_ISP   = RETRO_ITOT/RETRO_PROP_MASS;
const double RETRO_STA   = -1.028;

const double VERNIER_PROP_MASS = 70.98;
const double VERNIER_ISP = 3200;
const double VERNIER_THRUST = 463;
const double VERNIER_RAD = 1.09;
const double VERNIER_STA = -0.504;

const double RCS_PROP_MASS=2;
const double RCS_ISP = 630.0;
const double RCS_THRUST = 0.25;
const double RCS_RAD = 1.852;
const double RCS_STA = -0.653;
const double RCS_SPACE = 0.1;

const double LEG_RAD = 2.052;
const double LEG_STA =-0.792;
const double LEG_PIVOT_RAD =  1.119;
const double LEG_PIVOT_STA = -0.33;

