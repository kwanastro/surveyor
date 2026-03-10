// ==============================================================
//                Surveyor Landing Guidance
//             Copyright (C) 2005 Chris Jeppesen
//                   All rights reserved
//
// SLG.cpp
//
// Guidance system for the Surveyor Lunar Lander
// ==============================================================

#define STRICT
#define ORBITER_MODULE
#include "windows.h"
#include "orbitersdk.h"
#include "Surveyor.h"

const double PRE_RETRO_T=1.1;
const double R10=0.294784580498866; //Thrust curve curvature
const double SAFETY_MARGIN=1000;   //Design retro such that it ends this many
                                   //meters above the thrust curve

#define RED RGB(255, 0, 0)
#define GREEN RGB(0, 255, 0)
#define YELLOW RGB(255, 255, 0)
#define DARK_YELLOW RGB(128, 128, 0)
#define WHITE RGB(255, 255, 255)
#define BLUE RGB(0, 0, 255)
#define GRAY RGB(160, 160, 160)
#define BRIGHTERGRAY RGB(200, 200, 200)

class SLG: public MFD {
public:
  VESSEL *vessel;
  SLG (DWORD w, DWORD h, VESSEL *v);
  static int MsgProc (UINT msg, UINT mfd, WPARAM wparam, LPARAM lparam);
  ~SLG ();
  bool ConsumeKeyBuffered (DWORD key);
  void Update (HDC hDC);

  int Line( int );
  void PrintEngUnit(HDC hDC, char* label, double value, int x, int l);
  int TPIR( double );
  int DX( double, int );
  int DY( double, int );
  double linterp(double x1, double y1, double x2, double y2, double x);
  void calcAMR();

  int width,height,mid;
  double simtNextDraw;
  double SlantFactor;
  double Slant,TGoImp;
  double pitchCmd,yawCmd;
  VECTOR3 spd;
  double SeqSimT;
  double g;
  double AltMark,Delay,RetroThr,vImp,DebugVar;
  double NextP,NextY;
  double RetroEndV,RetroEndH;
  bool hasAMR;

  void TimeStep(double simt);

  int sequence;
  OBJHANDLE Planet;
  double ThrLevel,DesiredAcc;
  bool IsEngaged;

  //User-configurable variables
  double *Vars[25];
  double VarMax[25];
  double VarMin[25];
  double VarBigDelta[25];
  double VarSmallDelta[25];
  char VarNames[25][256];
  int NVars;
  int CurrentVar;
};


// ==============================================================
// Global variables

int g_MFDmode;
SLG* thisSLG;

// ==============================================================
// API interface

DLLCLBK void opcDLLInit (HINSTANCE hDLL) {
  static char *name = "SLG";   // MFD mode name
  MFDMODESPEC spec;
  spec.name = name;
  spec.key = OAPI_KEY_E;                // MFD mode selection key
  spec.msgproc = SLG::MsgProc;  // MFD mode callback function

  // Register the new MFD mode with Orbiter
  g_MFDmode = oapiRegisterMFDMode (spec);
  thisSLG=NULL;

}

DLLCLBK void opcDLLExit (HINSTANCE hDLL) {
  // Unregister the custom MFD mode when the module is unloaded
  oapiUnregisterMFDMode (g_MFDmode);
}

DLLCLBK void opcPreStep (double simt, double simdt, double mjd) {
  // Unregister the custom MFD mode when the module is unloaded
  if(thisSLG!=NULL) {
    thisSLG->TimeStep(simt);
  }
}

// ==============================================================
// MFD class implementation

// Constructor
SLG::SLG (DWORD w, DWORD h, VESSEL *v): MFD (w, h, v),pitchCmd(0),yawCmd(0) {
  vessel=v;
  // Add MFD initialisation here
  width=w;
  height=h;
  mid=w/2;
  simtNextDraw=-1;
  thisSLG=this;
  sequence=0;
  IsEngaged=false;

  AltMark=92000;
  Vars[0]=&AltMark;
  VarMax[0]=120000;
  VarMin[0]=60000;
  VarBigDelta[0]=1000;
  VarSmallDelta[0]=100;
  strncpy(VarNames[0],"Altitude Mark: %7.3f%cm",255);

  Delay=7;
  Vars[1]=&Delay;
  VarMax[1]=30;
  VarMin[1]=0;
  VarBigDelta[1]=1;
  VarSmallDelta[1]=0.1;
  strncpy(VarNames[1],"Delay:         %7.3f%cs",255);

  RetroThr=0.8;
  Vars[2]=&RetroThr;
  VarMax[2]=1;
  VarMin[2]=0;
  VarBigDelta[2]=0.1;
  VarSmallDelta[2]=0.01;
  strncpy(VarNames[2],"Retro VThrust:  %7.3f%c",255);

  NVars=3;
  CurrentVar=0;

  hasAMR=false;
}

// Destructor
SLG::~SLG ()
{
  // Add MFD cleanup code here
}
void SLG::calcAMR() {
  PROPELLANT_HANDLE ph_vernier=vessel->GetPropellantHandleByIndex(0);
  PROPELLANT_HANDLE ph_retro  =vessel->GetPropellantHandleByIndex(1);
  double InertMass=RETRO_EMPTY_MASS+LANDER_EMPTY_MASS+vessel->GetPropellantMass(ph_retro);
  double RetroT=PRE_RETRO_T+RETRO_BURNTIME;
  double RetroMDot=RETRO_PROP_MASS/RETRO_BURNTIME;

  double VernierProp=vessel->GetPropellantMass(ph_vernier);
  double VernierThrust=3*VERNIER_THRUST*RetroThr;
  double VernierMDot=VernierThrust/VERNIER_ISP;
  double VernierPropUsedRetro=VernierMDot*RetroT;
  double VernierPropAtRetroStart=VernierProp-VernierMDot*PRE_RETRO_T;
  double VernierPropAtRetroEnd=VernierPropAtRetroStart-VernierMDot*RETRO_BURNTIME;
  double TotalMassAtPreRetroStart=InertMass+RETRO_PROP_MASS+VernierProp+AMR_MASS;
  double TotalMassAtRetroStart=InertMass+RETRO_PROP_MASS+VernierPropAtRetroStart;
  double TotalMassAtRetroEnd=InertMass+VernierPropAtRetroEnd;
  double RetroTotalThr=RETRO_THRUST+VernierThrust;
  double RetroEquivIsp=(RETRO_ISP*RETRO_THRUST+VERNIER_ISP*VernierThrust)/RetroTotalThr;
  double RetroEquivMDot=RetroMDot+VernierMDot;

  OBJHANDLE Moon=vessel->GetGravityRef();
  double MoonM=oapiGetMass(Moon);
  double MoonGM=GGRAV*MoonM;
  double MoonR=oapiGetSize(Moon);
  double MoonG=-MoonGM/MoonR/MoonR;
  
  double PreretroDist=vImp*PRE_RETRO_T;
  double t=RETRO_BURNTIME;
  double v0=vImp-MoonG*26;
  double x0=0;
  double m=TotalMassAtRetroStart-RetroEquivMDot*t;
  double a=RetroEquivIsp*RetroEquivMDot/m+MoonG;
  double v=RetroEquivIsp*log(TotalMassAtRetroStart/m)+MoonG*t+v0;
  RetroEndV=-v;
  double x=RetroEquivIsp*(t-m*log(TotalMassAtRetroStart/m)/RetroEquivMDot)+v0*t+MoonG*t*t/2+x0;
  double DesiredEndAlt=R10*v*v+SAFETY_MARGIN;
  RetroEndH=DesiredEndAlt;
  double DesiredPreRetroStartAlt=-x+DesiredEndAlt-PreretroDist;
  double FallDist=-AltMark+DesiredPreRetroStartAlt;
  Delay=FallDist/vImp;
  hasAMR=true;
}


const double DescentCurveSpd[]={ 1.5, 3, 30, 120,  210,  250};
const double DescentCurveAlt[]={13.0,15,300,4000,13000,20000};
const int DescentCurveNPoints=6;
const char *title[]={"Wait for AMR lock","Turn to retrograde","Have Alt Mark","Light Verniers","Main Retro","Jettison retro","Minimum Acc","Descent Curve","Constant Spd","Freefall"};
const double MaxSpd=250;
const double MaxAlt=25000;
const double MaxSpd2=40;
const double MaxAlt2=800;

int SLG::TPIR( double X) {
  for(int i=0;i<DescentCurveNPoints-1;i++) {
    if(DescentCurveSpd[i]<X && DescentCurveSpd[i+1]>=X) return i;
  }
  return -1;
}

int SLG::Line( int i ) {
  return (int)((float)i*((float)height/26.0));
}

int SLG::DX( double x, int scale ) {
  if(scale == 1) {
    return (int)(((float)height)*x/MaxSpd);
  } else {
    return (int)(((float)height)*x/MaxSpd2);
  }
}

int SLG::DY( double y, int scale ) {
  if(scale == 1) {
    return (int)(((float)height)*(1-y/MaxAlt));
  } else {
    return (int)(((float)height)*(1-y/MaxAlt2));
  }
}

int DisplayEngUnit(char* buffer, char* pattern,double x) {
  char Big[]=  " kMGTPEZYXWV";
  char Small[]=" munpfazyxwv";
  int ptr=0;
  if(fabs(x)>1e-24) {
    if(fabs(x)>1) {
      while(fabs(x)>1000) {
        ptr++;
        x=x/1000.0;
      }
      return sprintf(buffer,pattern,x,Big[ptr]);
    } else {
      while(fabs(x)<1) {
        ptr++;
        x=x*1000.0;
      }
      return sprintf(buffer,pattern,x,Small[ptr]);
    }
  } else {
    return sprintf(buffer,pattern,x,Small[0]);
  }
}

void SLG::PrintEngUnit(HDC hDC, char* label, double value, int x, int l) {
  char buf[256];
  int len=DisplayEngUnit(buf,label,value);
  TextOut(hDC,x,Line(l),buf,len);
}

double asinh(double x) {
  return log(x+sqrt(1+x*x));
}

double calcTA(double a,double e,double dist) {
  return acos(a*(1-e*e)/(dist*e)-1/e);
}

double calcEA(double e,double ta) {
  if(e<1) {
    return 2*atan(sqrt((1-e)/(1+e))*tan(ta/2));
  } else {
    double sE=-sin(ta)*sqrt(e*e-1)/(1+e*cos(ta));
    return asinh(sE);
  }
}

double calcM(double e, double E) {
  if(e<1) {
    return E-e*sin(E);
  } else {
    return e*sinh(E)-E;
  }
}

// Repaint the MFD
void SLG::Update (HDC hDC) {
  double simt=oapiGetSimTime();
  Title (hDC, "Surveyor Landing Guidance");
  // Draws the MFD title

  // Add MFD display routines here.
  // Use the device context (hDC) for Windows GDI paint functions.
  char buffer[256];
//  char namebuffer[256];
  int len;

  Planet=vessel->GetGravityRef();
  double PlanetRad=oapiGetSize(Planet);
  double mu=6.67259e-11*oapiGetMass(Planet);
  g=mu/(PlanetRad*PlanetRad);
  double vcirc=sqrt(mu/PlanetRad);
  VECTOR3 r;
  ELEMENTS el;
  double epoch,Alt;
  vessel->GetRelativePos(Planet,r);
  double dist=length(r);
  vessel->GetElements(el,epoch);
  Alt=vessel->GetAltitude();
  double ta=calcTA(el.a,el.e,dist);
  SlantFactor=sin(vessel->GetPitch());
  Slant=Alt/SlantFactor;
  int scale = (-spd.z>MaxSpd2 || Slant>MaxAlt2)?1:2;
  int line=1;
  PrintEngUnit(hDC,"Slant Range:      %7.3f%c",Slant,5,line++);
  PrintEngUnit(hDC,"XSpeed:            %7.3f%c",spd.x,5,line++);
  PrintEngUnit(hDC,"YSpeed:            %7.3f%c",spd.y,5,line++);
  PrintEngUnit(hDC,"ZSpeed:            %7.3f%c",spd.z,5,line++);

  len=sprintf(buffer,"Sequence: %d %s",sequence,title[sequence]);
  TextOut(hDC,5,Line(line++),buffer,len);
  MoveToEx(hDC,DX(DescentCurveSpd[0],scale),DY(DescentCurveAlt[0],scale),NULL);
  for(int i=1;i<DescentCurveNPoints;i++) {
    LineTo(hDC,DX(DescentCurveSpd[i],scale),DY(DescentCurveAlt[i],scale));
  }
  if(hasAMR) {
    MoveToEx(hDC,DX(RetroEndV,scale)-5,DY(RetroEndH,scale)-5,NULL);
    LineTo(hDC,DX(RetroEndV,scale)+6,DY(RetroEndH,scale)+6);
    MoveToEx(hDC,DX(RetroEndV,scale)-5,DY(RetroEndH,scale)+6,NULL);
    LineTo(hDC,DX(RetroEndV,scale)+6,DY(RetroEndH,scale)-5);
  }
  MoveToEx(hDC,DX(-spd.z,scale)-5,DY(Alt,scale),NULL);
  LineTo(hDC,DX(-spd.z,scale)+6,DY(Alt,scale));
  MoveToEx(hDC,DX(-spd.z,scale),DY(Alt,scale)-5,NULL);
  LineTo(hDC,DX(-spd.z,scale),DY(Alt,scale)+6);
  int Idx=TPIR(-spd.z);
  if(Idx>=0) {
    double DesiredAlt=linterp(DescentCurveSpd[Idx],DescentCurveAlt[Idx],DescentCurveSpd[Idx+1],DescentCurveAlt[Idx+1],-spd.z);
    PrintEngUnit(hDC,"Alt Error:        %7.3f%c",Alt-DesiredAlt,5,line++);
  } else {
    double DesiredAlt=DescentCurveAlt[DescentCurveNPoints-1];
  }
  PrintEngUnit(hDC,"Commanded Thrust: %7.3f%c",ThrLevel,5,line++);
  if(sequence>=4 && sequence <=6) {
    PrintEngUnit(hDC,"Desired Acc:      %7.3f%c",DesiredAcc,5,line++);
  }
  double taImp=calcTA(el.a,el.e,PlanetRad);
//  if(taImp<ta) taImp=2*PI-taImp;
  double n=sqrt((el.e>1?-1:1)*mu/(el.a*el.a*el.a));
  double E=calcEA(el.e,ta);
  double EImp=calcEA(el.e,taImp);
  double M=calcM(el.e,E);
  if(el.e<1 && M<0)M+=2*PI;
  double MImp=calcM(el.e,EImp);
  if(el.e<1 && MImp<0)MImp+=2*PI;
  vImp=-sqrt(2*mu/PlanetRad-mu/el.a);
  double sfpaImp,cfpaImp;
  if(el.e<1) {
    sfpaImp=el.e*sin(EImp)/sqrt(1-pow(el.e*cos(EImp),2));
    cfpaImp=sqrt((1-el.e*el.e)/(1-pow(el.e*cos(EImp),2)));
  } else {
    sfpaImp=-el.e*sinh(EImp)/sqrt(pow(el.e*cosh(EImp),2)-1);
    cfpaImp=sqrt((el.e*el.e-1)/(pow(el.e*cosh(EImp),2)-1));
  }
  double fpaImp=-atan2(sfpaImp,cfpaImp);
  TGoImp=(MImp-M)/n;
  PrintEngUnit(hDC,"Impact TGo:  %7.3f%cs",TGoImp,5,line++);
  PrintEngUnit(hDC,"       fpa:  %7.3f%c°",fpaImp*DEG,5,line++);
  PrintEngUnit(hDC,"       v:    %7.3f%cm/s",vImp,5,line++);
  line++;
  if(IsEngaged) {
    SetTextColor(hDC,GREEN);
    PrintEngUnit(hDC,"Autopiolt Engaged",0,5,line++);
  } else {
    SetTextColor(hDC,GRAY);
    PrintEngUnit(hDC,"Autopilot Not Engaged",0,5,line++);
  }
  for(int i=0;i<NVars;i++) {
    SetTextColor(hDC,(CurrentVar==i)?YELLOW:GRAY);
    PrintEngUnit(hDC,VarNames[i],*Vars[i],5,line++);
  }
}

double SLG::linterp(double x1, double y1, double x2, double y2, double x) {
  double deltax=(x2-x1);
  double deltay=(y2-y1);
  double frac=(x-x1)/deltax;
  return y1+deltay*frac;
}

void SLG::TimeStep(double simt) {
  vessel->GetGroundspeedVector(FRAME_LOCAL,spd);
  Slant=vessel->GetAltitude()/SlantFactor;
  double mass,Thr;//,DesiredAlt;
  int Idx;
  ThrLevel=0;
  bool AlignDV1=false;
  switch(sequence) {
    case 0: //Wait
      if(fabs(TGoImp)<1800) {
        //Turn to retrograde
        sequence++;
      }
      break;
    case 1: //Wait some more
	    AlignDV1=true;
      if(Slant<AltMark) {
        //go to inertial hold
        if(IsEngaged)vessel->ActivateNavmode(NAVMODE_KILLROT);
		    AlignDV1=false;
        vessel->SetAttitudeRotLevel(0,0);
        vessel->SetAttitudeRotLevel(1,0);
        sequence++;
        SeqSimT=simt+Delay;
      }
      break;
    case 2: //Have Alt Mark
      if(simt>SeqSimT) {
        sequence++;
        SeqSimT=simt+PRE_RETRO_T;
      }
      break;
    case 3: //Light the verniers
      ThrLevel=RetroThr;
      if(simt>SeqSimT) {
        //Light the retro
        THRUSTER_HANDLE th_retro=vessel->GetThrusterHandleByIndex(0);
        if(IsEngaged)vessel->SetThrusterLevel(th_retro,1);
        sequence++;
        SeqSimT=simt+39;
      }
      break;
    case 4:
      ThrLevel=RetroThr;
      if(simt>SeqSimT) {
        AlignDV1=true;
        sequence++;
        SeqSimT=simt+5;
      }
      break;
    case 5:
      ThrLevel=1.0;
      if(simt>SeqSimT) {
        AlignDV1=true;
        //Retro has dropped, back to retrograde, turn up verniers
        sequence++;
      }
      break;
    case 6:
      AlignDV1=true;
      Idx=TPIR(-spd.z);
      double DesiredAlt;
      if(Idx>=0) {
        DesiredAlt=linterp(DescentCurveSpd[Idx],DescentCurveAlt[Idx],DescentCurveSpd[Idx+1],DescentCurveAlt[Idx+1],-spd.z);
      } else {
        DesiredAlt=DescentCurveAlt[DescentCurveNPoints-1];
      }
      if(Slant-DesiredAlt<0.05*Slant) {
        sequence++;
      }
      mass=vessel->GetMass();
      DesiredAcc=0.9*g;
      Thr=DesiredAcc*mass;
      ThrLevel=Thr/(VERNIER_THRUST*3);
      break;
    case 7:
      AlignDV1=true;
      if(-spd.z<DescentCurveSpd[0]) {
        sequence++;
      } else {
        Idx=TPIR(-spd.z);
        double DesiredAlt=linterp(DescentCurveSpd[Idx],DescentCurveAlt[Idx],DescentCurveSpd[Idx+1],DescentCurveAlt[Idx+1],-spd.z);
        //Change in speed with respect to height
        double ddxdh=(DescentCurveSpd[Idx+1]-DescentCurveSpd[Idx])/(DescentCurveAlt[Idx+1]-DescentCurveAlt[Idx]);
        DesiredAcc=ddxdh*-spd.z;

        mass=vessel->GetMass();
        Thr=(DesiredAcc+g/SlantFactor)*mass;
        ThrLevel=Thr/(VERNIER_THRUST*3)-3*(Slant-DesiredAlt)/Slant;
      }
      break;
    case 8:
      AlignDV1=true;
      if(Slant<4) {
        sequence++;
      } else {
        DesiredAcc=0;
        mass=vessel->GetMass();
        Thr=(DesiredAcc+g/SlantFactor)*mass;
        ThrLevel=Thr/(VERNIER_THRUST*3);
      }
      break;
    case 9:
      //Cut everything off
      ThrLevel=0;
      if(IsEngaged)vessel->SetThrusterGroupLevel(THGROUP_MAIN,ThrLevel);
      IsEngaged=false;
      vessel->SetAttitudeRotLevel(0,0);
      vessel->SetAttitudeRotLevel(1,0);
      AlignDV1=false;
      break;
  }
  if(IsEngaged) {
  	vessel->SetThrusterGroupLevel(THGROUP_MAIN,ThrLevel);
	double P,D;
    VECTOR3 rate;
    vessel->GetAngularVel(rate);
    if(AlignDV1) {
	  P=200.0;
	  D=-200;
  	} else {
	  P=0;
	  D=-200;
    }
    double pitchCmd=P*(spd.y/ spd.z)+D*rate.x;
    double yawCmd=  P*(spd.x/-spd.z)+D*rate.y;
    vessel->SetAttitudeRotLevel(0,pitchCmd);
    vessel->SetAttitudeRotLevel(1,yawCmd);
  }
}

// MFD message parser
int SLG::MsgProc (UINT msg, UINT mfd, WPARAM wparam, LPARAM lparam) {
  switch (msg) {

  case OAPI_MSG_MFD_OPENED:
    // Our new MFD mode has been selected, so we create the MFD and
    // return a pointer to it.
    return (int)(new SLG (LOWORD(wparam), HIWORD(wparam), (VESSEL*)lparam));
  }
  return 0;
}

bool SLG::ConsumeKeyBuffered (DWORD key) {
  switch(key) {
    case OAPI_KEY_A:
      IsEngaged=!IsEngaged;
	  if(!IsEngaged) {
	    //This cancels all attitude commands for this timestep, manual and auto.
		//But, once IsEngaged is false, no new auto cmds are generated, so only
		//any and all manual cmds will be in effect after this step.
	    vessel->SetAttitudeRotLevel(0,pitchCmd);
        vessel->SetAttitudeRotLevel(1,yawCmd);
	  }
	  calcAMR();
      return true;
    case OAPI_KEY_PERIOD:
      CurrentVar++;
      if(CurrentVar>=NVars) CurrentVar=0;
      return true;
    case OAPI_KEY_COMMA:
      CurrentVar--;
      if(CurrentVar<0) CurrentVar=NVars-1;
      return true;
    case OAPI_KEY_MINUS:
      *Vars[CurrentVar]=max(*Vars[CurrentVar]-VarBigDelta[CurrentVar],VarMin[CurrentVar]);
      return true;
    case OAPI_KEY_EQUALS:
      *Vars[CurrentVar]=min(*Vars[CurrentVar]+VarBigDelta[CurrentVar],VarMax[CurrentVar]);
      return true;
    case OAPI_KEY_LBRACKET:
      *Vars[CurrentVar]=max(*Vars[CurrentVar]-VarSmallDelta[CurrentVar],VarMin[CurrentVar]);
      return true;
    case OAPI_KEY_RBRACKET:
      *Vars[CurrentVar]=min(*Vars[CurrentVar]+VarSmallDelta[CurrentVar],VarMax[CurrentVar]);
      return true;
  }
  return false;
}


