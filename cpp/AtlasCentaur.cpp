// ==============================================================
// AtlasCentaur.cpp
// Implementation of the AtlasCentaur launcher
// ==============================================================

#define STRICT
#include "orbitersdk.h"
#include <math.h>
#include <stdio.h>
#include <fstream>
#include "tables.h"
#include "VesselMass.h"

PARTICLESTREAMSPEC rp1_flame = {
	0,		// flag
	3.2,	// size
	7000,	// rate
	180.0,	// velocity
	0.15,	// velocity distribution
	0.33,	// lifetime
	4.0,	// growthrate
	0.0,	// atmslowdown 
	PARTICLESTREAMSPEC::EMISSIVE,
	PARTICLESTREAMSPEC::LVL_PSQRT, 0, 1.0,
	PARTICLESTREAMSPEC::ATM_PLOG, 1e-1140, 1.0
};

PARTICLESTREAMSPEC srb_exhaust = {
	0,		// flag
	2.75,	// size
	2000,	// rate
	60.0,	// velocity
	0.1,	// velocity distribution
	0.4,	// lifetime
	2.0,	// growthrate
	0.0,	// atmslowdown 
	PARTICLESTREAMSPEC::EMISSIVE,
	PARTICLESTREAMSPEC::LVL_PSQRT, 0, 0.5,
	PARTICLESTREAMSPEC::ATM_PLOG, 1e-1140, 1.0
};


#define midpoint(x,y) (((x)+(y))/2)

// ==========================================================
// Some Orbiter-related parameters
// ==========================================================

const double BOOSTER_EMPTY_MASS = 3329;
const double BOOSTER_ISP0 = 2824;
const double BOOSTER_ISP1 = 2469;
const double BOOSTER_THR0 = 834955.7; //Vacuum thrust per engine, N
const double GROUND_RUN_TIME=2.05;

const double STAGE1_EMPTY_MASS = 4306;
const double STAGE1_PROP_MASS = 112237.2+1305; //Liftoff prop mass plus startup prop mass
const double STAGE1_ISP0 = 3009;
const double STAGE1_ISP1 = 2089;
const double STAGE1_THR0 = 358135; //Vacuum Thrust per engine, N

const double VERNIER_THR0 = 4448; //Vacuum Thrust per engine, N

const double STAGE2_EMPTY_MASS = 1866;
const double STAGE2_PROP_MASS = 14200;
const double STAGE2_ISP0 = 4354;
const double STAGE2_ISP1 = STAGE2_ISP0*0.5;
const double STAGE2_THR0 = 66700;

// Vacuum thrust rating for attitude thrusters (Reaction Control System) [N]
const double RCS_PROP_MASS = 100;
const double RCS_THRUST = 1000.0;
const double RCS_ISP0 = 1630.0;
const double RCS_ISP1 = 1630.0;

const double RETRO_PROP_MASS = 7.25; //kg total 8 boosters
const double RETRO_THRUST = 2250;   //N each
const double RETRO_ISP=2450;        //m/s

// ==========================================================
// Some Booster-related parameters
// ==========================================================
const double FAIRING_MASS = 891;
const double INSULATION_MASS = 532;

const double MAX_ATT_LAUNCH = 1e5;

const double BECO_ACC = 5.7*G;

// ==========================================================
// Mesh offsets for various configurations
// ==========================================================

const double INCHES=0.0254; //Inches to meters conversion. Multiply a number of inches by this to get meters.
const double LB_M=0.45359237; //Pounds mass to kilograms conversion
const double LB_F=LB_M*G; //Pounds force to newtons conversion

const double BOOSTER_AFT=-1310.6*INCHES;
const double BOOSTER_FWD=-1133.0*INCHES;
const double BOOSTER_LEN=BOOSTER_FWD-BOOSTER_AFT;
const double BOOSTER_MID=midpoint(BOOSTER_AFT,BOOSTER_FWD);

const double SUST_AFT=BOOSTER_AFT;
const double SUST_FWD=- 412.72*INCHES;
const double SUST_LEN=SUST_FWD-SUST_AFT;
const double SUST_MID=midpoint(SUST_AFT,SUST_FWD);

const double CENT_AFT=- 517.61*INCHES;
const double CENT_FWD=- 125.82*INCHES;
const double CENT_LEN=CENT_FWD-CENT_AFT;
const double CENT_MID=midpoint(CENT_AFT,CENT_FWD);

const double FAIR_AFT=- 219.00*INCHES;
const double FAIR_FWD=   46.49*INCHES;
const double FAIR_LEN=FAIR_FWD-FAIR_AFT;
const double FAIR_MID=midpoint(FAIR_AFT,FAIR_FWD);

const double INSU_AFT=SUST_FWD;
const double INSU_FWD=FAIR_AFT;
const double INSU_LEN=INSU_FWD-INSU_AFT;
const double INSU_MID=midpoint(INSU_AFT,INSU_FWD);

const double LEG_RAD=3;
const double COG0_STA=midpoint(BOOSTER_AFT,FAIR_FWD);
const double COG1_STA=midpoint(BOOSTER_AFT,FAIR_FWD);
const double COG2_STA=midpoint(CENT_AFT,FAIR_FWD);
const double BOOSTER_ENGINE_STA=BOOSTER_AFT;
const double VERNIER_ENGINE_STA=-1133.0*INCHES;
const double RETRO_STA=VERNIER_ENGINE_STA;
const double LEG_STA=BOOSTER_ENGINE_STA;
const double STAGE2_ENGINE_STA=CENT_AFT;
const double PAYLOAD_STA=CENT_FWD;
const double BOOSTER_RAD=1.5;
const double VERNIER_RAD=1.6;
const double STAGE2_ENGINE_RAD=0.663;

// ==============================================================
// Some vessel parameters
// ==============================================================
const double TRI32=sqrt(3.0)/2.0;

// ==============================================================
// AtlasCentaur class interface
// ==============================================================

class AtlasCentaur: public VESSEL2 {
public:
  AtlasCentaur (OBJHANDLE hVessel, int flightmodel):VESSEL2 (hVessel, flightmodel) {
    LoadMeshes();
  }
  void clbkSetClassCaps (FILEHANDLE cfg);
  void clbkPreStep(double SimT, double SimDT, double MJD);
  void LoadMeshes();
  int  clbkConsumeBufferedKey(DWORD key, bool down, char *kstate);
  THRUSTER_HANDLE th_sustainer, th_booster[2], th_vernier[2], th_centaur[2], th_rcs[16];
  PROPELLANT_HANDLE ph_atlas, ph_centaur, ph_rcs;
  int hBooster, hSustainer, hCentaur, hFairing[2], hInsul[4];
  void DropBooster();
  void DropSustainer();
  void Jettison();
  void Reconfigure();
  void HoldDown();
  void BaseSuction();
  int status;

  void CreateAttControls(PROPELLANT_HANDLE ph, double MaxThr, VECTOR3 AttPoint, double Rad);
  void SetupRotControls();
  void SetupLinControls();
  void SetRCSThrust(double thr);
  void GetMainThrustParm(double &FF, double &isp);
  double GetMainThrust();

  MESHHANDLE mBooster,mSustainer,mFairing1,mFairing2;
  MESHHANDLE mInsulationA,mInsulationB,mInsulationC,mInsulationD,mCentaur;
  bool FairingAttached;
  bool InsulationAttached;
  bool PayloadAttached;
  bool bOnGround;
  double met,met0;
  double oldrcsThr;
  double TLight;
  double groundBurnTime;
  void Sequencer(double SimT);
  int SequenceMode;
  double TSequence;
  
  VesselMass *vesselMass;
  AttachMass *attachMass;
};

// ==============================================================
// Overloaded callback functions
// ==============================================================

//Aerodynamics tables
#define N_CA_MACH 25
const double CaMach[N_CA_MACH]={
   0.00, 0.25, 0.50, 0.60, 0.70, 0.80, 0.85, 0.90, 0.93, 0.95, 1.00, 1.05, 1.10, 1.15, 1.25, 1.40, 1.50, 1.75, 2.00, 2.50, 3.50, 4.50, 6.00, 8.00,10.00
};
const double Ca0[N_CA_MACH]={
  0.373,0.347,0.345,0.350,0.365,0.391,0.425,0.481,0.565,0.610,0.725,0.760,0.773,0.770,0.740,0.665,0.622,0.530,0.459,0.374,0.303,0.273,0.259,0.267,0.289
};
const double CaSustainer=0.272;
const double CaCentaur=2.200;
const int N_CN_MACH=23;
const double CnMach[N_CN_MACH]={
  0.00, 0.20, 0.50, 0.70, 0.80, 0.90, 0.95, 1.00, 1.05, 1.15, 1.20, 1.25, 1.50, 1.75, 2.00, 2.25, 2.50, 3.00, 3.50, 4.00, 5.00, 7.00, 10.00
};
const double Cn0[N_CN_MACH]={
  0.0052,0.0052,0.0052,0.0100,0.0085,0.0079,0.0062,0.0057,0.0053,0.0051,0.0050,0.0050,0.0055,0.0072,0.0063,0.0059,0.0060,0.0062,0.0064,0.0060,0.0050,0.0036,0.0030
};
const double Cm0[N_CN_MACH]={
  -0.0060,-0.0060,-0.0068,-0.0088,-0.0074,-0.0060,-0.0060,-0.0062,-0.0065,-0.0070,-0.0065,-0.0058,-0.0028,-0.0083,-0.0060,-0.0036,-0.0032,-0.0040,-0.0043,-0.0040,-0.0030,-0.0020,-0.0020
};
const int N_CN_ALPHA=7; 
const double CnAlpha[N_CN_ALPHA]={
  0,	2,	4,	6,	8,	60,	90
};
const double CnStarOverAlpha[N_CN_ALPHA][N_CN_MACH]={
  {0.0556,0.0556,0.0557,0.0564,0.0586,0.0650,0.0692,0.0743,0.0731,0.0677,0.0678,0.0672,0.0690,0.0733,0.0732,0.0720,0.0736,0.0769,0.0789,0.0795,0.0803,0.0797,0.0743},
  {0.0556,0.0556,0.0557,0.0566,0.0593,0.0660,0.0709,0.0754,0.0741,0.0699,0.0699,0.0691,0.0706,0.0741,0.0741,0.0790,0.0746,0.0787,0.0819,0.0803,0.0816,0.0811,0.0756},
  {0.0556,0.0556,0.0559,0.0606,0.0643,0.0702,0.0770,0.0795,0.0781,0.0754,0.0743,0.0740,0.0744,0.0779,0.0785,0.0784,0.0796,0.0823,0.0843,0.0851,0.0869,0.0865,0.0810},
  {0.0556,0.0556,0.0581,0.0658,0.0706,0.0760,0.0812,0.0846,0.0838,0.0817,0.0810,0.0806,0.0811,0.0843,0.0859,0.0861,0.0869,0.0909,0.0932,0.0942,0.0955,0.0950,0.0894},
  {0.0556,0.0556,0.0625,0.0718,0.0774,0.0817,0.0899,0.0909,0.0897,0.0882,0.0874,0.0872,0.0894,0.0947,0.0970,0.0979,0.1002,0.1047,0.1086,0.1101,0.1120,0.1110,0.1049},
  {0.1348,0.1348,0.1486,0.1866,0.2179,0.2506,0.2699,0.2878,0.3008,0.3057,0.2990,0.2872,0.2676,0.2605,0.2549,0.2517,0.2461,0.2413,0.2391,0.2358,0.2348,0.2308,0.2261},
  {0.1198,0.1198,0.1320,0.1658,0.1938,0.2241,0.2423,0.2595,0.2704,0.2714,0.2648,0.2551,0.2374,0.2312,0.2265,0.2229,0.2183,0.2131,0.2105,0.2095,0.2089,0.2077,0.2065}
};
const double Cn0Sust=0.0030;
const double CnStarOverAlphaSust[N_CN_ALPHA]={
  0.0743,0.0756,0.0810,0.0894,0.1049,0.2261,0.2065
};
const double XcpLref[N_CN_ALPHA][N_CN_MACH]={
  {0.1492,0.1493,0.1498,0.1632,0.1917,0.2114,0.2174,0.2169,0.1956,0.1084,0.1063,0.1089,0.1416,0.1540,0.1756,0.1800,0.1804,0.1847,0.1983,0.2231,0.2586,0.2830,0.2941},
  {0.1718,0.1719,0.1724,0.2125,0.2411,0.2401,0.2382,0.2477,0.2470,0.2227,0.1948,0.1881,0.1810,0.1703,0.1628,0.1637,0.1685,0.1801,0.2114,0.2154,0.2402,0.2632,0.2741},
  {0.1718,0.1689,0.1784,0.2235,0.2381,0.2485,0.2546,0.2623,0.2600,0.2395,0.2257,0.2203,0.2094,0.1997,0.1904,0.1839,0.1861,0.1969,0.2101,0.2256,0.2504,0.2735,0.2845},
  {0.1718,0.1719,0.1987,0.2433,0.2574,0.2661,0.2685,0.2686,0.2675,0.2465,0.2389,0.2337,0.2265,0.2203,0.2132,0.2053,0.2043,0.1942,0.2326,0.2534,0.2766,0.2975,0.3056},
  {0.1718,0.1719,0.2158,0.2552,0.2678,0.2749,0.2773,0.2756,0.2724,0.2534,0.2468,0.2421,0.2439,0.2422,0.2357,0.2338,0.2364,0.2481,0.2720,0.2912,0.3130,0.3308,0.3363},
  {0.3846,0.3846,0.3829,0.3794,0.3775,0.3764,0.3761,0.3762,0.3772,0.3822,0.3830,0.3841,0.3843,0.3838,0.3839,0.3838,0.3840,0.3838,0.3836,0.3834,0.3836,0.3837,0.3826},
  {0.4038,0.4038,0.4021,0.3989,0.3977,0.3981,0.4003,0.4022,0.4022,0.4022,0.4024,0.4027,0.4027,0.4027,0.4027,0.4027,0.4028,0.4028,0.4028,0.4028,0.4027,0.4024,0.4015}
};
const double Sref=(12*12*78.5)*INCHES*INCHES;
const double Lref=1500*INCHES;

void BodyLiftCoeff(VESSEL* v, double beta, double M, double Re,void* context, double *cl, double *cm, double *cd) {
  double Ca,Cn;
  AtlasCentaur* ac=(AtlasCentaur*)v;
  if(ac->status==0) {
    double cn0=listerp(CnMach,Cn0,N_CN_MACH,M);
    double cnStarOverAlpha=tableterp((double*)CnStarOverAlpha,CnAlpha,N_CN_ALPHA,CnMach,N_CN_MACH,beta*DEG,M);
    Cn=cn0+beta*DEG*cnStarOverAlpha;
    Ca=listerp(CaMach,Ca0,N_CA_MACH,M);
  } else if(ac->status==1) {
    double cn0=Cn0Sust;
    double cnStarOverAlpha=listerp(CnAlpha,CnStarOverAlphaSust,N_CN_ALPHA,beta*DEG);
    Cn=cn0+beta*DEG*cnStarOverAlpha;
    Ca=CaSustainer;
  } else if(ac->status==2) {
    Cn=0;
  	Ca=CaCentaur;
  }
  *cl = Cn*cos(beta)-Ca*sin(beta);
  *cd = (Cn*sin(beta)+Ca*cos(beta))/2;
  *cm = 0.0;
}

// --------------------------------------------------------------
// Set the capabilities of the vessel class
// --------------------------------------------------------------
void AtlasCentaur::clbkSetClassCaps(FILEHANDLE cfg) {
  vesselMass=new VesselMass(this,"Main");
    
  //Ship structure
  VesselMass *vmBooster=  new VesselMass(this,"Booster");
  VesselMass *vmSustainer=new VesselMass(this,"Sustainer");
  VesselMass *vmCentaur=  new VesselMass(this,"Centaur");
  VesselMass *vmFairingL= new VesselMass(this,"FairingL");
  VesselMass *vmFairingR= new VesselMass(this,"FairingR");
  VesselMass *vmInsulA=   new VesselMass(this,"InsulA");
  VesselMass *vmInsulB=   new VesselMass(this,"InsulB");
  VesselMass *vmInsulC=   new VesselMass(this,"InsulC");
  VesselMass *vmInsulD=   new VesselMass(this,"InsulD");
                                       //Name       Color      Mass           base             axis   length      radius
  vmBooster->attach(  new CylinderShell("Booster",  _V(0,0,0), 3329,     _V( 0,0,-1310.60*INCHES),2,  117.60*INCHES, 60*INCHES)); //Booster including engines
  vmSustainer->attach(new CylinderShell("Sustainer",_V(0,0,0), 4306,     _V( 0,0,-1310.60*INCHES),2,  897.88*INCHES, 60*INCHES)); //Sustainer including engines and interstage
  vmCentaur->attach(  new CylinderShell("Centaur",  _V(0,0,0), 1866,     _V( 0,0,- 412.72*INCHES),2,  286.90*INCHES, 60*INCHES)); //Centaur including engines, length is centaur tank only
  vmFairingL->attach( new CylinderShell("FairingL", _V(0,0,0),  891.0/2, _V( 0,0,- 219.00*INCHES),2,     30.9,       60*INCHES)); 
  vmFairingR->attach( new CylinderShell("FairingR", _V(0,0,0),  891.0/2, _V( 0,0,- 219.00*INCHES),2,     30.9,       60*INCHES)); 
  vmInsulA->attach(   new CylinderShell("InsulA",   _V(0,0,0),  532.0/4, _V( 0,0,- 412.72*INCHES),2,     30.9,       60*INCHES)); 
  vmInsulB->attach(   new CylinderShell("InsulB",   _V(0,0,0),  532.0/4, _V( 0,0,- 412.72*INCHES),2,     30.9,       60*INCHES)); 
  vmInsulC->attach(   new CylinderShell("InsulC",   _V(0,0,0),  532.0/4, _V( 0,0,- 412.72*INCHES),2,     30.9,       60*INCHES)); 
  vmInsulD->attach(   new CylinderShell("InsulD",   _V(0,0,0),  532.0/4, _V( 0,0,- 412.72*INCHES),2,     30.9,       60*INCHES)); 

  ph_rcs     = CreatePropellantResource(RCS_PROP_MASS);
  //Propellant
  const double ATLAS_RP1_LOAD= 76951*LB_M;
  const double ATLAS_LOX_LOAD=173426*LB_M;
  const double ATLAS_RP1_DENS= 796.91;
  const double ATLAS_LOX_DENS=1109.60;
  ph_atlas=CreatePropellantResource(ATLAS_RP1_LOAD+ATLAS_LOX_LOAD); //Liftoff prop mass plus startup prop mass
  vmSustainer->attach(ph_atlas);
  vmSustainer->attach(new TankLiquid("SustainerRP1",_V(0,0,0),ATLAS_RP1_LOAD/(ATLAS_RP1_LOAD+ATLAS_LOX_LOAD),this,ph_atlas,ATLAS_RP1_DENS,_V(0,0,-1160.54),2,60*INCHES));
  vmSustainer->attach(new TankLiquid("SustainerLOX",_V(0,0,0),ATLAS_LOX_LOAD/(ATLAS_RP1_LOAD+ATLAS_LOX_LOAD),this,ph_atlas,ATLAS_LOX_DENS,_V(0,0,- 928.99),2,60*INCHES));

  
  const double CENTAUR_LH2_LOAD=  5271*LB_M;
  const double CENTAUR_LOX_LOAD= 25434*LB_M;
  const double CENTAUR_LH2_DENS=  67.52;
  const double CENTAUR_LOX_DENS=ATLAS_LOX_DENS;
  ph_centaur=CreatePropellantResource(CENTAUR_LH2_LOAD+CENTAUR_LOX_LOAD); //Liftoff prop mass
  vmCentaur->attach(ph_centaur);
  vmCentaur->attach(new TankLiquid("CentaurLH2",_V(0,0,0),CENTAUR_LH2_LOAD/(CENTAUR_LH2_LOAD+CENTAUR_LOX_LOAD),this,ph_centaur,CENTAUR_LH2_DENS,_V(0,0,- 421),2,60*INCHES));
  vmCentaur->attach(new TankLiquid("CentaurLOX",_V(0,0,0),CENTAUR_LOX_LOAD/(CENTAUR_LH2_LOAD+CENTAUR_LOX_LOAD),this,ph_centaur,CENTAUR_LOX_DENS,_V(0,0,- 321),2,60*INCHES));
  
  //Payload Attach Point
  attachMass=new AttachMass("Payload",_V(0,0,0),this, _V(0,0,PAYLOAD_STA), _V(0,0,1), _V(0,1,0));
  vesselMass->attach(attachMass);
  
  //Meshes
  vmBooster->attach(mBooster,_V(0,0,0),_V(0,0,0));
  vmSustainer->attach(mSustainer,_V(0,0,0),_V(0,0,0));
  vmCentaur->attach(mCentaur,_V(0,0,0),_V(0,0,0));
  vmFairingL->attach(mFairing1,_V(0,0,0),_V(0,0,0));
  vmFairingR->attach(mFairing2,_V(0,0,0),_V(0,0,0));
  vmInsulA->attach(mInsulationA,_V(0,0,0),_V(0,0,0));
  vmInsulB->attach(mInsulationB,_V(0,0,0),_V(0,0,0));
  vmInsulC->attach(mInsulationC,_V(0,0,0),_V(0,0,0));
  vmInsulD->attach(mInsulationD,_V(0,0,0),_V(0,0,0));
  
  met=0;
  met0=0;
  status=0;
  // physical specs
  SetSize (15);
  SetCameraOffset (_V(0,BOOSTER_RAD*1.3,0));
  SetTouchdownPoints( _V( 0,LEG_RAD,LEG_STA), _V( TRI32*LEG_RAD,-0.5*LEG_RAD,LEG_STA), _V(-TRI32*LEG_RAD,-0.5*LEG_RAD,LEG_STA));

  CreateAirfoil3(LIFT_VERTICAL,  _V(0,0,0),BodyLiftCoeff,NULL,Lref,Sref,BOOSTER_RAD*2/Sref);
  CreateAirfoil3(LIFT_HORIZONTAL,_V(0,0,0),BodyLiftCoeff,NULL,Lref,Sref,BOOSTER_RAD*2/Sref);

  th_sustainer = CreateThruster(_V( 0,           0,BOOSTER_ENGINE_STA), _V(0,0,1), STAGE1_THR0, ph_atlas, STAGE1_ISP0,STAGE1_ISP1);
  th_booster[0] = CreateThruster(_V(0, BOOSTER_RAD,BOOSTER_ENGINE_STA), _V(0,0,1), BOOSTER_THR0, NULL, BOOSTER_ISP0, BOOSTER_ISP1);
  th_booster[1] = CreateThruster(_V(0,-BOOSTER_RAD,BOOSTER_ENGINE_STA), _V(0,0,1), BOOSTER_THR0, NULL, BOOSTER_ISP0, BOOSTER_ISP1);
  th_vernier[0] = CreateThruster(_V( VERNIER_RAD,0,VERNIER_ENGINE_STA), _V(-0.5,0,1), VERNIER_THR0, ph_atlas, BOOSTER_ISP0, BOOSTER_ISP1);
  th_vernier[1] = CreateThruster(_V(-VERNIER_RAD,0,VERNIER_ENGINE_STA), _V( 0.5,0,1), VERNIER_THR0, ph_atlas, BOOSTER_ISP0, BOOSTER_ISP1);
  th_centaur[0] = CreateThruster(_V(0, STAGE2_ENGINE_RAD,STAGE2_ENGINE_STA), _V(0,0,1), STAGE2_THR0, ph_centaur, STAGE2_ISP0);
  th_centaur[1] = CreateThruster(_V(0,-STAGE2_ENGINE_RAD,STAGE2_ENGINE_STA), _V(0,0,1), STAGE2_THR0, ph_centaur, STAGE2_ISP0);
  SURFHANDLE tex_stage2 = oapiRegisterExhaustTexture ("Exhaust_atsme");
  SURFHANDLE tex_stage1 = oapiRegisterExhaustTexture("Exhaust2");
  AddExhaust(th_sustainer,   7, 1,tex_stage1);
  AddExhaust(th_booster[0], 10, 1,tex_stage1);
  AddExhaust(th_booster[1], 10, 1,tex_stage1);
  AddExhaust(th_vernier[0],  3, 0.1,tex_stage1);
  AddExhaust(th_vernier[1],  3, 0.1,tex_stage1);
  AddExhaust(th_centaur[0], 10, 1,tex_stage2);
  AddExhaust(th_centaur[1], 10, 1,tex_stage2);
  AddExhaustStream(th_booster[0],_V(0, BOOSTER_RAD,BOOSTER_ENGINE_STA-COG0_STA)+_V(0,0,-5), &rp1_flame);
  AddExhaustStream(th_booster[1],_V(0,-BOOSTER_RAD,BOOSTER_ENGINE_STA-COG0_STA)+_V(0,0,-5), &rp1_flame);
  AddExhaustStream(th_sustainer, _V(0,           0,BOOSTER_ENGINE_STA-COG0_STA)+_V(0,0,-5), &rp1_flame);

  CreateAttControls(ph_centaur,0,_V(0,0,0),BOOSTER_RAD);

  SetCameraRotationRange(PI,PI,PI/2.0,PI/2.0);
  Reconfigure();
  TSequence=0.0;

  hBooster=vesselMass->attach(vmBooster);
  hSustainer=vesselMass->attach(vmSustainer);
  hCentaur=vesselMass->attach(vmCentaur);
  hFairing[0]=vesselMass->attach(vmFairingL);
  hFairing[1]=vesselMass->attach(vmFairingR);
  hInsul[0]=vesselMass->attach(vmInsulA);
  hInsul[1]=vesselMass->attach(vmInsulB);
  hInsul[2]=vesselMass->attach(vmInsulC);
  hInsul[3]=vesselMass->attach(vmInsulD);
}

void AtlasCentaur::CreateAttControls(PROPELLANT_HANDLE ph, double MaxThr, VECTOR3 AttPoint, double Rad) {
  // we should only require pitch and bank, but yaw is also defined to make KILLROT work
  // no linear attitude modes are defined at this stage
  const int A=0; const int B=4; const int C=8; const int D=12;
  const int forward=0; const int left=1; const int up=1; const int backward=2; const int right=3; const int down=3;
  //Quad A (top)
  th_rcs[A+forward ] = CreateThruster (AttPoint+_V(0, Rad,0), _V( 0, 0, 1), MaxThr, ph, RCS_ISP0, RCS_ISP1); //Forward
  th_rcs[A+left    ] = CreateThruster (AttPoint+_V(0, Rad,0), _V(-1, 0, 0), MaxThr, ph, RCS_ISP0, RCS_ISP1); //Left
  th_rcs[A+backward] = CreateThruster (AttPoint+_V(0, Rad,0), _V( 0, 0,-1), MaxThr, ph, RCS_ISP0, RCS_ISP1); //Backward
  th_rcs[A+right   ] = CreateThruster (AttPoint+_V(0, Rad,0), _V( 1, 0, 0), MaxThr, ph, RCS_ISP0, RCS_ISP1); //Right
  //Quad B (right)
  th_rcs[B+forward ] = CreateThruster (AttPoint+_V( Rad,0,0), _V( 0, 0, 1), MaxThr, ph, RCS_ISP0, RCS_ISP1); //Forward
  th_rcs[B+up      ] = CreateThruster (AttPoint+_V( Rad,0,0), _V( 0, 1, 0), MaxThr, ph, RCS_ISP0, RCS_ISP1); //Up
  th_rcs[B+backward] = CreateThruster (AttPoint+_V( Rad,0,0), _V( 0, 0,-1), MaxThr, ph, RCS_ISP0, RCS_ISP1); //Backward
  th_rcs[B+down    ] = CreateThruster (AttPoint+_V( Rad,0,0), _V( 0,-1, 0), MaxThr, ph, RCS_ISP0, RCS_ISP1); //Down
  //Quad C (bottom)
  th_rcs[C+forward ] = CreateThruster (AttPoint+_V(0,-Rad,0), _V( 0, 0, 1), MaxThr, ph, RCS_ISP0, RCS_ISP1); //Forward
  th_rcs[C+left    ] = CreateThruster (AttPoint+_V(0,-Rad,0), _V(-1, 0, 0), MaxThr, ph, RCS_ISP0, RCS_ISP1); //Left
  th_rcs[C+backward] = CreateThruster (AttPoint+_V(0,-Rad,0), _V( 0, 0,-1), MaxThr, ph, RCS_ISP0, RCS_ISP1); //Backward
  th_rcs[C+right   ] = CreateThruster (AttPoint+_V(0,-Rad,0), _V( 1, 0, 0), MaxThr, ph, RCS_ISP0, RCS_ISP1); //Right
  //Quad D (left)
  th_rcs[D+forward ] = CreateThruster (AttPoint+_V(-Rad,0,0), _V( 0, 0, 1), MaxThr, ph, RCS_ISP0, RCS_ISP1); //Forward
  th_rcs[D+up      ] = CreateThruster (AttPoint+_V(-Rad,0,0), _V( 0, 1, 0), MaxThr, ph, RCS_ISP0, RCS_ISP1); //Up
  th_rcs[D+backward] = CreateThruster (AttPoint+_V(-Rad,0,0), _V( 0, 0,-1), MaxThr, ph, RCS_ISP0, RCS_ISP1); //Backward
  th_rcs[D+down    ] = CreateThruster (AttPoint+_V(-Rad,0,0), _V( 0,-1, 0), MaxThr, ph, RCS_ISP0, RCS_ISP1); //Down

  for(int i=0;i<16;i++) AddExhaust (th_rcs[i], 1.0,0.1);

  SetupRotControls();
  SetupLinControls();
}

void AtlasCentaur::SetupRotControls() {
  const int A=0; const int B=4; const int C=8; const int D=12;
  const int forward=0; const int left=1; const int up=1; const int backward=2; const int right=3; const int down=3;
  THRUSTER_HANDLE th_group[4];

  th_group[0]=th_rcs[A+backward];
  th_group[1]=th_rcs[C+forward];
  CreateThrusterGroup (th_group, 2, THGROUP_ATT_PITCHUP);

  th_group[0]=th_rcs[A+forward];
  th_group[1]=th_rcs[C+backward];
  CreateThrusterGroup (th_group, 2, THGROUP_ATT_PITCHDOWN);

  th_group[0]=th_rcs[A+left];
  th_group[1]=th_rcs[B+up];
  th_group[2]=th_rcs[C+right];
  th_group[3]=th_rcs[D+down];
  CreateThrusterGroup (th_group, 4, THGROUP_ATT_BANKLEFT);

  th_group[0]=th_rcs[A+right];
  th_group[1]=th_rcs[B+down];
  th_group[2]=th_rcs[C+left];
  th_group[3]=th_rcs[D+up];
  CreateThrusterGroup (th_group, 4, THGROUP_ATT_BANKRIGHT);

  th_group[0]=th_rcs[D+forward];
  th_group[1]=th_rcs[B+backward];
  CreateThrusterGroup (th_group, 2, THGROUP_ATT_YAWRIGHT);

  th_group[0]=th_rcs[D+backward];
  th_group[1]=th_rcs[B+forward];
  CreateThrusterGroup (th_group, 2, THGROUP_ATT_YAWLEFT);
}

void AtlasCentaur::SetupLinControls() {

  const int A=0; const int B=4; const int C=8; const int D=12;
  const int forward=0; const int left=1; const int up=1; const int backward=2; const int right=3; const int down=3;
  THRUSTER_HANDLE th_group[4];

  th_group[0]=th_rcs[A+forward];
  th_group[1]=th_rcs[B+forward];
  th_group[2]=th_rcs[C+forward];
  th_group[3]=th_rcs[D+forward];
  CreateThrusterGroup (th_group, 4, THGROUP_ATT_FORWARD);

  th_group[0]=th_rcs[A+backward];
  th_group[1]=th_rcs[B+backward];
  th_group[2]=th_rcs[C+backward];
  th_group[3]=th_rcs[D+backward];
  CreateThrusterGroup (th_group, 4, THGROUP_ATT_BACK);

}

void AtlasCentaur::SetRCSThrust(double Thr) {
  if(Thr==oldrcsThr) return;
  oldrcsThr=Thr;
  for(int i=0;i<16;i++) SetThrusterMax0(th_rcs[i],Thr);
}

void AtlasCentaur::Reconfigure() {
  //Hook engines to tanks
  THRUSTER_HANDLE th_group[5];
  int nEngines;
  switch(status) {
    case 0:
      SetThrusterResource(th_sustainer,ph_atlas);th_group[0]=th_sustainer;
      SetThrusterResource(th_booster[0],ph_atlas);th_group[1]=th_booster[0];
      SetThrusterResource(th_booster[1],ph_atlas);th_group[2]=th_booster[1];
      th_group[3]=th_vernier[0];
      th_group[4]=th_vernier[1];
      nEngines=5;
      break;
    case 1:
      th_group[0]=th_sustainer;
      SetThrusterResource(th_booster[0],NULL);
      SetThrusterResource(th_booster[1],NULL);
      th_group[1]=th_vernier[0];
      th_group[2]=th_vernier[1];
      nEngines=3;
      break;
    case 2:
      SetPropellantMass(ph_atlas,0);
      th_group[0]=th_centaur[0];th_group[1]=th_centaur[1];nEngines=2;
  }
  //Hook engines to throttle button
  CreateThrusterGroup(th_group, nEngines, THGROUP_MAIN);
  //Hook tank up to display
  SetDefaultPropellantResource(status<2?ph_atlas:ph_centaur);
}

double kapiGetThrusterIsp(VESSEL* vessel, OBJHANDLE Thr) {
  OBJHANDLE Prop=vessel->GetThrusterResource(Thr);
  if(Prop) {
    return vessel->GetThrusterIsp(Thr)*vessel->GetPropellantEfficiency(Prop);
  } else {
    return vessel->GetThrusterIsp(Thr);
  }
}

void AtlasCentaur::GetMainThrustParm(double &FF, double &isp) {
  VECTOR3 F,M,TotalF=_V(0,0,0);
  isp = 0;
  FF=0;
  int nthr=GetThrusterCount();
  if (nthr==0) return;
  //Assume that under max thrust of all main drive engines along the line from the center of thrust
  //to the center of gravity, the rocket is balanced, ie no rotation.
  for(int i=0; i<nthr; i++) {
    THRUSTER_HANDLE th=GetThrusterHandleByIndex(i);
    if(th!=NULL) {
      double thisMax=GetThrusterMax0(th);
      if(thisMax>20000) {
        GetThrusterMoment(th,F,M);
        TotalF+=F;
        isp += kapiGetThrusterIsp(this,th) * length(F);
      }
    }
  }
  FF=length(TotalF);
  isp /= FF;
}

double AtlasCentaur::GetMainThrust() {
  double F,isp;
  GetMainThrustParm(F,isp);
  return F;
}

void AtlasCentaur::Sequencer(double met) {
//  sprintf(oapiDebugString(),"MET %f SequenceMode %d TSequence %f TGo %f ",met,SequenceMode,TSequence,TSequence-met);
  if(TSequence>met) return; //Only look at what action to do if it't time to do the action.
  char kstate[256];
  switch(SequenceMode) {
    case 0: //Nothing to do
      break;
    case 1: //Drop booster
      vesselMass->detach(hBooster,_V(0,0,0),_V(0.1,0.1,0.01));
      SequenceMode=2;
      TSequence+=30.9;
      break;
    case 2: 
      vesselMass->detach(hInsul[0],_V(10,10,0),_V(2,2,0));
      vesselMass->detach(hInsul[1],_V(0,0,0),_V(0.1,0.1,0.01));
      vesselMass->detach(hInsul[2],_V(0,0,0),_V(0.1,0.1,0.01));
      vesselMass->detach(hInsul[3],_V(0,0,0),_V(0.1,0.1,0.01));
  	  SequenceMode=3;
	    TSequence+=26.5;
	    break;
	  case 3:
	//    JettisonFairing();
  	  SequenceMode=0;
	    break;
	  case 4:  //Drop sustainer
      vesselMass->detach(hBooster,_V(0,0,0),_V(0.1,0.1,0.01));
  	  SequenceMode=5;
	    TSequence+=9.6;
	    break;
	  case 5:  //Light Centaur
      SetThrusterGroupLevel(THGROUP_MAIN,1);
      SequenceMode=6;	  
  	  TSequence+=446+18; //MECO backup signal plus 18s
	    break;
  	case 6: //Deploy Surveyor gear
	    SequenceMode=7;
	    TSequence+=10.5;
  	  kstate[OAPI_KEY_LSHIFT]=0;
	    kstate[OAPI_KEY_RSHIFT]=0;
      clbkConsumeBufferedKey(OAPI_KEY_G, true, kstate);
	    break;
  	case 7: //Deploy Surveyor antenna
	    SequenceMode=8;
	    TSequence+=10.5;
  	  kstate[OAPI_KEY_LSHIFT]=0;
	    kstate[OAPI_KEY_RSHIFT]=0;
      clbkConsumeBufferedKey(OAPI_KEY_A, true, kstate);
	    break;
  	case 8: //Separate Surveyor 
	    Jettison();
	    SequenceMode=0;
  }
}

void AtlasCentaur::BaseSuction() {
  VECTOR3 r = _V(0,0,0), Fc = _V(0,0,0);
  if (met<10) {
    Fc.z=pow(10-met,2.3917)*41.290*LB_F;	
  } else {
    return;
  }
  AddForce (Fc, r);
}

void AtlasCentaur::HoldDown () {
  VECTOR3 F, T, r = _V(0,0,0), Fc = _V(0,0,0);
  if(bOnGround) {
    GetThrusterMoment(th_booster[0], F, T);
    Fc.z = -2*F.z;
    GetThrusterMoment(th_sustainer, F, T);
    Fc.z -= F.z;
  } else {
	return;
  }
  AddForce (Fc, r);
}

void AtlasCentaur::clbkPreStep(double SimT, double SimDT, double MJD) {
  vesselMass->step();

  double P,Y,R,M;
  P=GetThrusterGroupLevel(THGROUP_ATT_PITCHUP)-GetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN);
  Y=GetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT)-GetThrusterGroupLevel(THGROUP_ATT_YAWLEFT);
  R=GetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT)-GetThrusterGroupLevel(THGROUP_ATT_BANKLEFT);
  M=GetThrusterGroupLevel(THGROUP_MAIN);
  SetRCSThrust((M>0)?0:RCS_THRUST);
  if(bOnGround) {
	if(M<0.001) {
	  groundBurnTime=0;
	} else {
	  groundBurnTime+=SimDT;
	}
	if(groundBurnTime<GROUND_RUN_TIME) {
      if(M>0)HoldDown();
    } else {
	  bOnGround=false;
	  met0=SimT;
	  TSequence=0;
	} 
  } else {
    met=SimT-met0;
  }
  if(!GroundContact())BaseSuction();
  switch(status) {
	case 0: //Booster engine steering
      SetThrusterDir(th_booster[0],_V(-0.087*(Y-R),-0.087*P,1));
      SetThrusterDir(th_booster[1],_V(-0.087*(Y+R),-0.087*P,1));
	  break;
	case 1: //Sustainer engine steering
	  SetThrusterDir(th_sustainer,_V(-0.087*Y,-0.087*P,1));
      SetThrusterDir(th_vernier[0],_V(-0.5,-0.5*R,1));
      SetThrusterDir(th_vernier[1],_V( 0.5, 0.5*R,1));
	  break;
	case 2: //Centaur engine steering
      SetThrusterDir(th_centaur[0],_V(-0.087*(Y-R),-0.087*P,1));
      SetThrusterDir(th_centaur[1],_V(-0.087*(Y+R),-0.087*P,1));
	  break;
  }
  if(status==0 && GetMainThrust()/GetMass()>BECO_ACC) {
    SequenceMode=1;
    SetThrusterResource(th_booster[0],NULL); //Disconnect boosters from fuel tank
    SetThrusterResource(th_booster[1],NULL); //This causes BECO
	THRUSTER_HANDLE th_group[3];
    th_group[0]=th_sustainer;
    th_group[1]=th_vernier[0];
    th_group[2]=th_vernier[1];
    CreateThrusterGroup(th_group, 3, THGROUP_MAIN); //Set just the sustainer as main engine...
    SetThrusterGroupLevel(THGROUP_MAIN,1);               //and light it up!
	TSequence=met+3.1;
  }
  if(status==1 && SequenceMode!=4 &&  GetPropellantMass(ph_atlas)<1) {
	SequenceMode=4;
    TSequence=met+1.9;
  }
  Sequencer(met);
}

void AtlasCentaur::LoadMeshes() {
  mBooster=oapiLoadMeshGlobal("Surveyor/AtlasBooster");
  mSustainer=oapiLoadMeshGlobal("Surveyor/AtlasSustainer");
  mCentaur=oapiLoadMeshGlobal("Surveyor/Centaur");
  mFairing1=oapiLoadMeshGlobal("Surveyor/ACFairing_1");
  mFairing2=oapiLoadMeshGlobal("Surveyor/ACFairing_2");
  mInsulationA=oapiLoadMeshGlobal("Surveyor/CentaurInsulationA");
  mInsulationB=oapiLoadMeshGlobal("Surveyor/CentaurInsulationB");
  mInsulationC=oapiLoadMeshGlobal("Surveyor/CentaurInsulationC");
  mInsulationD=oapiLoadMeshGlobal("Surveyor/CentaurInsulationD");
}

void AtlasCentaur::Jettison() {
  double throttle=GetThrusterGroupLevel(THGROUP_MAIN);
  SetThrusterGroupLevel(THGROUP_MAIN,0);
  switch(status) {
    case 0:
//   	  DropBooster();
      status++;
      break;
    case 1:
  //	  DropSustainer();
      status++;
      break;
    case 2:
//      if(GetAttachmentStatus (sat_attach)) {
//        OBJHANDLE child=GetAttachmentStatus(sat_attach);
//        DetachChild (sat_attach, 1);
//        oapiSetFocusObject(child);
//        Reconfigure();
//      }
      break;
  }
  Reconfigure();
  if(status==1) {
    SetThrusterGroupLevel(THGROUP_MAIN,throttle);
  }
}


int AtlasCentaur::clbkConsumeBufferedKey(DWORD key, bool down, char *kstate) {
  if (down) { // only process keydown events
    if (KEYMOD_SHIFT (kstate)) {
    } else { // unmodified keys
      switch (key) {
        case OAPI_KEY_J:  // Jettison current stage
          Jettison();
          return 1;
        case OAPI_KEY_K:  // Jettison fairings
          return 1;
      }
    }
  }
  //If we haven't processed the key, send it along to the payload
//  if(GetAttachmentStatus (sat_attach)) {
//    OBJHANDLE child=GetAttachmentStatus(sat_attach);
//  	VESSEL2* surveyor=(VESSEL2*)oapiGetVesselInterface(child);
//  	return surveyor->clbkConsumeBufferedKey(key,down,kstate);
//  }
  return 0;
}

// ==============================================================
// API callback interface
// ==============================================================

// --------------------------------------------------------------
// Vessel initialisation
// --------------------------------------------------------------
DLLCLBK VESSEL *ovcInit (OBJHANDLE hvessel, int flightmodel) {
  return new AtlasCentaur (hvessel, flightmodel);
}

// --------------------------------------------------------------
// Vessel cleanup
// --------------------------------------------------------------
DLLCLBK void ovcExit (VESSEL *vessel) {
  if (vessel) delete (AtlasCentaur*)vessel;
}

