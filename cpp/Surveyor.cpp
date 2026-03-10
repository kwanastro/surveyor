// ==============================================================
//                 ORBITER MODULE: Surveyor
//                Copyright (C) 2005 Kwan3217
//       Released under the Gnu Free Documentation License
//
// Evolved from:
//                 ORBITER MODULE: ShuttlePB
//                  Part of the ORBITER SDK
//          Copyright (C) 2002-2004 Martin Schweiger
//                   All rights reserved
//
// Surveyor.cpp
// Control module for Surveyor vessel class
//
// ==============================================================

#define STRICT
#define ORBITER_MODULE

#include "orbitersdk.h"
#include "Surveyor.h"

#include <float.h>

// ==============================================================
// Surveyor class interface
// ==============================================================

class Surveyor: public VESSEL2 {
public:
  Surveyor (OBJHANDLE hVessel, int flightmodel)
    : VESSEL2 (hVessel, flightmodel) { DefineAnimations();}
  void clbkSetClassCaps (FILEHANDLE cfg);
  void clbkPreStep(double SimT, double SimDT, double MJD);
  void clbkSaveState (FILEHANDLE scn);
  int  clbkConsumeBufferedKey(DWORD key, bool down, char *kstate);
  void LoadState (FILEHANDLE scn, void *vs);
  THRUSTER_HANDLE th_vernier[3], th_retro, th_rcs[12], th_group[2];
  double CalcEmptyMass();
  PROPELLANT_HANDLE ph_vernier, ph_rcs, ph_retro;
  VESSEL* SpawnObject(MESHHANDLE mesh, char* ext, VECTOR3 &ofs, VECTOR3 &spd);
  void Jettison();
  int status;

  void SetupMeshes();
  void AddLanderMesh();
  void AddRetroMesh();
  void AddAMRMesh();
  MESHHANDLE mLander,mRetro,mAMR;

  int anim_status[3]; //0 - Stowed
                      //1 - Deploying
                      //2 - Deployed

  double anim_progress[3];
  double anim_time[3];


  void DefineAnimations();
  UINT anim[3];
};

double Surveyor::CalcEmptyMass() {
  double EmptyMass=0;
  if(GetPropellantMass(ph_retro)>0.999*RETRO_PROP_MASS) {
    EmptyMass+=AMR_MASS;
  }
  if(GetPropellantMass(ph_retro)>1) {
    EmptyMass+=RETRO_EMPTY_MASS;
  }
  EmptyMass+=LANDER_EMPTY_MASS;
  return EmptyMass;
}

void Surveyor::DefineAnimations() {
  static UINT Leg1Group[1]={17};
  static UINT Leg2Group[1]={18};
  static UINT Leg3Group[1]={19};
  static UINT OmniAGroup[1]={15};
  static UINT OmniBGroup[1]={16};
  static UINT ScoopGroup[1]={14};
  static UINT SolarGroup[1]={13};
  static UINT AntennaGroup[1]={12};
  static MGROUP_ROTATE Leg1(
    0,
    Leg1Group,
    1,
    _V(0,LEG_PIVOT_RAD,LEG_PIVOT_STA),
    _V(1,0,0),
    (float)(130*PI/180.0)
  );
  static MGROUP_ROTATE AntennaB(
    0,
    OmniBGroup,
    1,
    _V(0,LEG_PIVOT_RAD,LEG_PIVOT_STA),
    _V(1,0,0),
    (float)(45*PI/180.0)
  );
  static MGROUP_ROTATE AntennaPanel(
    0,
    AntennaGroup,
    1,
    _V(0,-0.236,2.105),
    _V(1,0,0),
    (float)(45*PI/180.0)
  );
  static MGROUP_ROTATE SolarPanel(
    0,
    SolarGroup,
    1,
    _V(0,-0.02,1.92),
    _V(-1,0,0),
    (float)(30*PI/180.0)
  );
  static MGROUP_ROTATE Leg2(
    0,
    Leg2Group,
    1,
    _V(-TRI32*LEG_PIVOT_RAD,-0.5*LEG_PIVOT_RAD,LEG_PIVOT_STA),
    _V(-0.5,TRI32,0),
    (float)(130*PI/180.0)
  );
  static MGROUP_ROTATE AntennaA(
    0,
    OmniAGroup,
    1,
    _V(TRI32*LEG_PIVOT_RAD,-0.5*LEG_PIVOT_RAD,LEG_PIVOT_STA),
    _V(-0.5,-TRI32,0),
    (float)(55*PI/180.0)
  );
  static MGROUP_ROTATE Leg3(
    0,
    Leg3Group,
    1,
    _V(TRI32*LEG_PIVOT_RAD,-0.5*LEG_PIVOT_RAD,LEG_PIVOT_STA),
    _V(-0.5,-TRI32,0),
    (float)(130*PI/180.0)
  );
  static MGROUP_ROTATE Scoop(
    0,
    ScoopGroup,
    1,
    _V(-0.115,-0.756,0),
    _V(0,0,1),
    (float)(105*PI/180.0)
  );
  anim[0]=CreateAnimation(0.0);
  AddAnimationComponent(anim[0],0,1,&Leg1);
  AddAnimationComponent(anim[0],0,1,&Leg2);
  AddAnimationComponent(anim[0],0,1,&Leg3);
  anim_time[0]=4.0;
  anim[1]=CreateAnimation(0.0);
  AddAnimationComponent(anim[1],0,1,&AntennaB);
  AddAnimationComponent(anim[1],0,1,&AntennaA);
  AddAnimationComponent(anim[1],0,1,&AntennaPanel);
  AddAnimationComponent(anim[1],0,1,&SolarPanel);
  anim_time[1]=5.0;
  anim[2]=CreateAnimation(0.0);
  AddAnimationComponent(anim[2],0,1,&Scoop);
  anim_time[2]=15.0;
  SetAnimation(anim[0],1.0);
  SetAnimation(anim[1],1.0);
  SetAnimation(anim[2],1.0);
}


// ==============================================================
// Overloaded callback functions
// ==============================================================

// --------------------------------------------------------------
// Set the capabilities of the vessel class
// --------------------------------------------------------------
void Surveyor::clbkSetClassCaps (FILEHANDLE cfg) {

  // physical specs
  SetSize (1.0);
  SetPMI (_V(0.5,0.6,0.5));
  SetCameraOffset (_V(0,1,1));
  SetTouchdownPoints( _V( 0,LEG_RAD,LEG_STA), _V( TRI32*LEG_RAD,-0.5*LEG_RAD,LEG_STA), _V(-TRI32*LEG_RAD,-0.5*LEG_RAD,LEG_STA));

  status=0;
  for(int i=0;i<3;i++) {
    anim_progress[i]=0;
    anim_status[i]=0;
  }

  // propellant resources
  ph_vernier = CreatePropellantResource(VERNIER_PROP_MASS);
  ph_rcs     = CreatePropellantResource(RCS_PROP_MASS);
  ph_retro   = CreatePropellantResource(RETRO_PROP_MASS);
  SetDefaultPropellantResource(ph_vernier);

  th_retro = CreateThruster(_V(0.0,0.0,RETRO_STA), _V(0,0,1), RETRO_THRUST, ph_retro, RETRO_ISP);
  AddExhaust(th_retro, 2, 0.3);

  th_vernier[0] = CreateThruster(_V(   0.0*VERNIER_RAD, 1.0*VERNIER_RAD,VERNIER_STA), _V(0,0,1), VERNIER_THRUST, ph_vernier, VERNIER_ISP);
  th_vernier[1] = CreateThruster(_V( TRI32*VERNIER_RAD,-0.5*VERNIER_RAD,VERNIER_STA), _V(0,0,1), VERNIER_THRUST, ph_vernier, VERNIER_ISP);
  th_vernier[2] = CreateThruster(_V(-TRI32*VERNIER_RAD,-0.5*VERNIER_RAD,VERNIER_STA), _V(0,0,1), VERNIER_THRUST, ph_vernier, VERNIER_ISP);
  CreateThrusterGroup(th_vernier, 3, THGROUP_MAIN);
  for(int i=0;i<3;i++) {
    AddExhaust(th_vernier[i], 1, 0.1);
  }

  //Roll (Leg1) jets
  th_rcs[ 0] = CreateThruster (_V(-RCS_SPACE,RCS_RAD,RCS_STA), _V( 1,0,0), RCS_THRUST, ph_rcs, RCS_ISP);
  th_rcs[ 1] = CreateThruster (_V( RCS_SPACE,RCS_RAD,RCS_STA), _V(-1,0,0), RCS_THRUST, ph_rcs, RCS_ISP);
  //Leg2 jets
  th_rcs[ 2] = CreateThruster (_V( TRI32*RCS_RAD,-0.5*RCS_RAD,RCS_STA-RCS_SPACE), _V(0, 0, 1), RCS_THRUST, ph_rcs, RCS_ISP);
  th_rcs[ 3] = CreateThruster (_V( TRI32*RCS_RAD,-0.5*RCS_RAD,RCS_STA+RCS_SPACE), _V(0, 0,-1), RCS_THRUST, ph_rcs, RCS_ISP);
  //Leg3 jets
  th_rcs[ 4] = CreateThruster (_V(-TRI32*RCS_RAD,-0.5*RCS_RAD,RCS_STA-RCS_SPACE), _V(0, 0, 1), RCS_THRUST, ph_rcs, RCS_ISP);
  th_rcs[ 5] = CreateThruster (_V(-TRI32*RCS_RAD,-0.5*RCS_RAD,RCS_STA+RCS_SPACE), _V(0, 0,-1), RCS_THRUST, ph_rcs, RCS_ISP);
  //Master jets
  th_rcs[ 6] = CreateThruster (_V( 0,1,0), _V( 0,0, 1), 1e-9, ph_rcs, RCS_ISP);
  th_rcs[ 7] = CreateThruster (_V( 0,1,0), _V( 0,0,-1), 1e-9, ph_rcs, RCS_ISP);
  th_rcs[ 8] = CreateThruster (_V( 0,1,0), _V( 1,0, 0), 1e-9, ph_rcs, RCS_ISP);
  th_rcs[ 9] = CreateThruster (_V( 0,1,0), _V(-1,0, 0), 1e-9, ph_rcs, RCS_ISP);
  th_rcs[10] = CreateThruster (_V( 1,0,0), _V( 0,0, 1), 1e-9, ph_rcs, RCS_ISP);
  th_rcs[11] = CreateThruster (_V(-1,0,0), _V( 0,0,-1), 1e-9, ph_rcs, RCS_ISP);


  th_group[0] = th_rcs[ 6];
  CreateThrusterGroup (th_group, 1, THGROUP_ATT_PITCHDOWN);
  th_group[0] = th_rcs[ 7];
  CreateThrusterGroup (th_group, 1, THGROUP_ATT_PITCHUP);
  th_group[0] = th_rcs[ 8];
  CreateThrusterGroup (th_group, 1, THGROUP_ATT_BANKRIGHT);
  th_group[0] = th_rcs[ 9];
  CreateThrusterGroup (th_group, 1, THGROUP_ATT_BANKLEFT);
  th_group[0] = th_rcs[10];
  CreateThrusterGroup (th_group, 1, THGROUP_ATT_YAWRIGHT);
  th_group[0] = th_rcs[11];
  CreateThrusterGroup (th_group, 1, THGROUP_ATT_YAWLEFT);

  for (int i=0;i<6;i++) {
    AddExhaust(th_rcs[i],0.1,0.05);
  }

  // visual specs
  SetupMeshes();
  SetCameraRotationRange(PI,PI,PI/2.0,PI/2.0);
}

double Sproing(double progress) {
  return cos(PI*progress)/2.0+0.5;
}

void Surveyor::clbkPreStep(double SimT, double SimDT, double MJD) {
  SetEmptyMass(CalcEmptyMass());
  double P,Y,R;
  double PP,PY,PR,MP,MY,MR;
  PP=GetThrusterGroupLevel(THGROUP_ATT_PITCHUP);
  MP=GetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN);
  PY=GetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT);
  MY=GetThrusterGroupLevel(THGROUP_ATT_YAWLEFT);
  PR=GetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT);
  MR=GetThrusterGroupLevel(THGROUP_ATT_BANKLEFT);
  P=PP-MP;
  Y=PY-MY;
  R=PR-MR;
  if(PP!=0 || !_finite(PP) ||
	 MP!=0 || !_finite(MP) ||
	 PY!=0 || !_finite(PY) ||
	 MY!=0 || !_finite(MY) ||
	 PR!=0 || !_finite(PR) ||
	 MR!=0 || !_finite(MR) 
	 ) {
	PP=PP;
  }

  SetThrusterDir(th_vernier[0],_V(0.087*R,0,1));
  SetThrusterDir(th_vernier[1],_V(0,0,1.0+0.05*P-0.05*Y));
  SetThrusterDir(th_vernier[2],_V(0,0,1.0+0.05*P+0.05*Y));
  if(GetThrusterLevel(th_vernier[0])<0.01) {
    SetThrusterLevel(th_rcs[0],PR);
    SetThrusterLevel(th_rcs[1],MR);
    SetThrusterLevel(th_rcs[2],PP+MY);
    SetThrusterLevel(th_rcs[3],MP+PY);
    SetThrusterLevel(th_rcs[4],PP+PY);
    SetThrusterLevel(th_rcs[5],MP+MY);
  } else {
    SetThrusterLevel(th_rcs[0],0);
    SetThrusterLevel(th_rcs[1],0);
    SetThrusterLevel(th_rcs[2],0);
    SetThrusterLevel(th_rcs[3],0);
    SetThrusterLevel(th_rcs[4],0);
    SetThrusterLevel(th_rcs[5],0);
  }

  if(status == 1 && GetPropellantMass(ph_retro)<1) {
    //Jettison the spent main retro
    Jettison();
  }
  if(status == 0 && GetPropellantMass(ph_retro)<0.999*RETRO_PROP_MASS) {
    //Jettison the AMR if the retro has started burning
    Jettison();
    //Relight the retro if needed
    SetThrusterLevel(th_retro,1);
  }
  //Gear animation
  for(int i=0;i<3;i++) {
    if(anim_status[i] == 1) {
      double da=SimDT/anim_time[i];
      if(anim_progress[i]<1.0) {
        anim_progress[i]=min(anim_progress[i]+da,1);
        SetAnimation(anim[i],Sproing(anim_progress[i]));
      } else {
        anim_status[i] = 2;
      }
    }
    if(anim_status[i] == 3) {
      double da=SimDT/anim_time[i];
      if(anim_progress[i]>0.0) {
        anim_progress[i]=max(anim_progress[i]-da,0);
        SetAnimation(anim[i],Sproing(anim_progress[i]));
      } else {
        anim_status[i] = 0;
      }
    }
  }
}

void Surveyor::AddLanderMesh() {
  VECTOR3 ofs = _V(0,0.0,0);
  SetMeshVisibilityMode(AddMesh(mLander,&ofs),MESHVIS_ALWAYS);
}
void Surveyor::AddRetroMesh() {
  VECTOR3 ofs = _V(0,0,0);
  SetMeshVisibilityMode(AddMesh(mRetro,&ofs),MESHVIS_ALWAYS);
}
void Surveyor::AddAMRMesh() {
  VECTOR3 ofs = _V(0,0,0);
  SetMeshVisibilityMode(AddMesh(mAMR,&ofs),MESHVIS_ALWAYS);
}

void Surveyor::SetupMeshes() {
  ClearMeshes();
  mLander=oapiLoadMeshGlobal("Surveyor/Surveyor-Lander");
  mRetro=oapiLoadMeshGlobal("Surveyor/Surveyor-Retro");
  mAMR=oapiLoadMeshGlobal("Surveyor/Surveyor-AMR");
  AddLanderMesh();
  if(status<2) AddRetroMesh();
  if(status<1) AddAMRMesh();
}

VESSEL* Surveyor::SpawnObject(MESHHANDLE mesh, char* ext, VECTOR3 &ofs, VECTOR3 &spdofs) {
  VESSELSTATUS vs;
  char name[256];
  GetStatus(vs);
  Local2Rel(ofs, vs.rpos);

  VECTOR3 rot;
  VECTOR3 rvel;

  rot=_V( 0.0, 0.0, 0.0 );

  GlobalRot(spdofs, rvel);

  vs.vrot.x += rot.x;
  vs.vrot.y += rot.y;
  vs.vrot.z += rot.z;

  vs.rvel.x += rvel.x;
  vs.rvel.y += rvel.y;
  vs.rvel.z += rvel.z;

// Create the boosters as separate vessels

  vs.eng_main = vs.eng_hovr = 0.0;
  vs.status = 0;
  strcpy (name, GetName()); strcat (name, ext);
  VESSEL* drop=oapiGetVesselInterface(oapiCreateVessel(name, "VesselMassStage", vs));
  drop->AddMesh(mesh);
  return drop;
}

void Surveyor::Jettison() {
  VESSEL* drop;
  double r,h,PMI_A,PMI_C;
  switch(status) {
    case 0:
      status=1;
      drop=SpawnObject(mAMR,"-AMR",_V(0,0,-0.8),_V(0,0,-50));
      drop->SetSize(1);
      drop->SetEmptyMass(AMR_MASS);
      r=1;
      h=0.5;
      PMI_A=(3.0*r*r+h*h)/12.0;
      PMI_C=r*r/2.0;
      drop->SetPMI(_V(PMI_A,PMI_A,PMI_C));

	  SetDefaultPropellantResource(ph_retro);
      break;
    case 1:
      status=2;
      drop=SpawnObject(mRetro,"-Retro",_V(0,0,-0.5),_V(0,0,0));
      drop->SetSize(1);
      drop->SetEmptyMass(RETRO_EMPTY_MASS);
      r=1;
      h=1;
      PMI_A=(3.0*r*r+h*h)/12.0;
      PMI_C=r*r/2.0;
      drop->SetPMI(_V(PMI_A,PMI_A,PMI_C));

      SetDefaultPropellantResource(ph_vernier);
      break;
  }
  SetupMeshes();
}

int Surveyor::clbkConsumeBufferedKey(DWORD key, bool down, char *kstate) {
  if (!down) return 0; // only process keydown events

  if (KEYMOD_SHIFT (kstate)) {

  } else { // unmodified keys
    switch (key) {
      case OAPI_KEY_L:  // Fire Retro
        SetThrusterLevel(th_retro,1);
        return 1;
      case OAPI_KEY_G:  // Deploy landing gear
        anim_status[0] = 1;
        return 1;
      case OAPI_KEY_A:  // Deploy Antennas
        anim_status[1] = 1;
        return 1;
      case OAPI_KEY_S:  // Deploy Scoop
        anim_status[2] = anim_status[2]==0?1:3;
        return 1;
    }
  }
  return 0;
}

int has(char* line, char* it) {
  return !strnicmp(line,it,strlen(it));
}

void get(char*line, char* it, char* format, void* stuff) {
  sscanf(line+strlen(it),format,stuff);
}

int getIt(char* line, char* it, char* format, void* stuff) {
  if (has(line,it)) {
    get(line,it,format,stuff);
    return 1;
  } else {
    return 0;
  }
}

bool getBo(char* line, char* it, bool* stuff) {
  if (has(line,it)) {
    int thing;
    get(line,it,"%d",&thing);
    *stuff=(thing>0);
    return 1;
  } else {
    return 0;
  }
}

// Read status from scenario file
void Surveyor::LoadState (FILEHANDLE scn, void *vs) {
  char *line;

  while (oapiReadScenario_nextline (scn, line)) {
    if(!getIt(line, "CONFIGURATION","%d", &status))
    if(!getIt(line, "GEAR_STATE","%d", &anim_status[0]))
    if(!getIt(line, "GEAR_PROGRESS","%lf", &anim_progress[0]))
    if(!getIt(line, "ANTENNA_STATE","%d", &anim_status[1]))
    if(!getIt(line, "ANTENNA_PROGRESS","%lf", &anim_progress[1]))
    if(!getIt(line, "SCOOP_STATE","%d", &anim_status[2]))
    if(!getIt(line, "SCOOP_PROGRESS","%lf", &anim_progress[2]))
    ParseScenarioLineEx (line, vs);
  }

  for(int i=0;i<3;i++) SetAnimation(anim[i],cos(PI*anim_progress[i])/2.0+0.5);
}

// Write status to scenario file
void Surveyor::clbkSaveState (FILEHANDLE scn) {
  char cbuf[256];

  // default vessel parameters
  VESSEL2::clbkSaveState(scn);

  // custom parameters
  oapiWriteScenario_int (scn, "CONFIGURATION", status);
  oapiWriteScenario_int (scn, "GEAR_STATE", anim_status[0]);
  oapiWriteScenario_float (scn, "GEAR_PROGRESS", anim_progress[0]);
  oapiWriteScenario_int (scn, "ANTENNA_STATE", anim_status[1]);
  oapiWriteScenario_float (scn, "ANTENNA_PROGRESS", anim_progress[1]);
  oapiWriteScenario_int (scn, "SCOOP_STATE", anim_status[2]);
  oapiWriteScenario_float (scn, "SCOOP_PROGRESS", anim_progress[2]);

}


// ==============================================================
// API callback interface
// ==============================================================

// --------------------------------------------------------------
// Vessel initialisation
// --------------------------------------------------------------
DLLCLBK VESSEL *ovcInit (OBJHANDLE hvessel, int flightmodel) {
  return new Surveyor (hvessel, flightmodel);
}

// --------------------------------------------------------------
// Vessel cleanup
// --------------------------------------------------------------
DLLCLBK void ovcExit (VESSEL *vessel) {
  if (vessel) delete (Surveyor*)vessel;
}

