/*! 
 * \file CtrlStruct_gr2.h
 * \brief Controller main structure
 */

#define REALBOT

#ifndef _CTRL_STRUCT_GR2_H_
#define _CTRL_STRUCT_GR2_H_

#ifndef REALBOT
#include "ctrl_io.h"
#include "namespace_ctrl.h"
#include <stdlib.h>

NAMESPACE_INIT(ctrlGr2);
#else
#include "MyApp.h"
#include <math.h>
#endif // !REALBOT

#include <stdbool.h>

#define LIMITACCELERATION 1
#define DEGtoRAD M_PI/180
#define RADtoDEG 180/M_PI
#define VOLTtoDC 100/24

#define RIGHTMOTOR  0
#define LEFTMOTOR   1
#define TOWERMOTOR  2
#ifdef REALBOT
#define RATRMOTOR   3
#define RATLMOTOR   4
#define PINCEMOTOR  5
#define NB_STORE_EDGE 10
#endif //REALBOT

#define BLUE 0
#define RED 1
#define YELLOW 2
#define WHITE  3
#define GREEN 4
#define PINK 5

#define EPSILON 0.000001

#define MAXSPEED 2*M_PI
#define MAXSPEEDROT 30*MAXSPEED
#define BEACON_POSITION_TOLERANCE 0.1
#define KIFLUSHLIMIT 1000
#define MaxGoals 15

//enum StateCalib {Cal_y_arr, Cal_y_arr2, Cal_y_av, Cal_y_av1, Cal_rot_neg, Cal_x_arr, Cal_x_av, Cal_rot_pos, Action1 };
enum StateCalib {Cal_y_arr, GoToPoint, AlignAngle, Cal_x_arr, ReturnToBase, AlignForBaseAndReturnInIt, Wait}; // GoToBlocOne, AlignBlocOne, TakeBlocOne, BringBlocOne, ReleaseBlockOne, AlignForBlockOne};
enum StateReCalib {ReCal_y_arr, ReCal_y_av, ReCal_rot1, ReCal_x_arr, ReCal_x_av, ReCal_rot2, ReCal_nextStrat };
enum StateStrat {reachPointA, reachPointB, reachPointC, reachPointD};
enum StateDyna {grap, release};
enum StateVia {backHomeViaBase, backHomeStraight, normalPoint, viaPoint};
enum StateHomologation {PinceCalib, reachViaPoint, AlignWithTheta, ReachBlocs, ClosingPince, GoViaZone, AlignZone, GoInZone, OpeningPince};
enum StateAction1{GoToHouses, AlignedWithHouses, PushHouses, FreeHouses};
enum StateAction2{GoToBlocOne, AlignForBlocOne, TakeBlocOne, BringBlockOne, ReleaseBlockOne, AlignForBlockOne,AlignForCalibAction2, Calib_y};
enum StateAction3{GoToBlocTwoCalib, AlignForCalibAction3, Calib_x, GoToBlocTwo, AlignForBlocTwo, AvanceForBlockTwo, ReculeForBlockTwo, BringBlockTwoViaPoint, TakeBlocTwo, BringBlockTwo, ReleaseBlockTwo, AlignForBlockTwo};
enum StateAction4{GoToFish, AlignForCalibFishes, CalibFishes, DecaleBordFishes, AlignedWithFishes,RatGoTopStartFish, DyntakeFish, MoveWithFish, ReleaseFish};
enum StateStrategy{GoCalibration, GoAction1, GoAction2, GoAction3,GoAction4, GoBase};


typedef struct Potential {
	double katt;
	double krep;
	double kFV;
	double FXRob;
	double FYRob;
	double kw;
	double minDistance;
    double thresholdAligned;
} Potential;

typedef struct Odometry {
	double timein;
	int timeDelay;
	double x;
	double y;
	double theta;
    double speedL;
    double speedR;
    double bufferTime;
    double flagBufferPosition;
#ifdef REALBOT
    double clicNumber;
#endif // REALBOT
} Odometry;

typedef struct Parametres {
	double radiusBot;
	double width;
	double lengthTower;
    double wheelRRadius;
    double wheelLRadius;
	double wheelRadius;
	double KpRot;
	double KiRot;
	double totalErrorRot;
	double speedDifThreshold;
	double KiAngleThreshold;
	double rayonBeacon;
    double maxAcceleration;
    bool MotorCommandByHand;
    double PasFiletVisRat;
    double PasFiletVisPince;
} Parametres;

typedef struct Sensors {
	bool uSwitchLeft;
	bool uSwitchRight;
#ifdef REALBOT
    bool uSwitchPinceOut;
    bool uSwitchRatL;
    bool uSwitchRatR; 
#endif // REALBOT
} Sensors;

typedef struct Motor {
	double Kp;
	double Ki;
	double KpPos;
	double Kphi;
	double speed;
	double position;
	double R;
	int reverseRotation;
	double totalError;
	double dutyCycle;
    double previousDutyCycle;
    bool areBrakesEnabled;
	int ID;
#ifdef REALBOT
    int PWMReg; 
    int timerReg;
    int perReg; 
    int brakesMask;
    int SID;
    double clicNumber;
#endif // REALBOT
} Motor;

typedef struct Dyna{
    double timer;
    enum StateDyna stateDyna;
    bool enable;
} Dyna;

typedef struct Rectangle {
	bool isActive;
	double x;
	double y;
	double width;
	double height;
} Rectangle;

typedef struct Circle {
	bool isActive;
	double x;
	double y;
	double radius;
} Circle;

typedef struct QuarterOfCircle {
	bool isActive;
	double x;
	double y;
	double radius;
	int quadrant;
} QuarterOfCircle;
typedef struct Obstacles {
	int NumberOfCircles;
	int NumberOfRectangles;
	int NumberOfQuarterOfCircle;
	Circle *CircleList;
	Rectangle *RectangleList;
	QuarterOfCircle *QuarterOfCircleList;
} Obstacles;

typedef struct Points {
	double X;
	double Y;
	bool taken;
} Points;

typedef struct Goals {
	int NumberOfGoals;
	int CurrentGoal;
	int lockState;
	int goalIN;
	int previousGoal;
	int nbr_target_prev;
	double precision;
	double timeIN;
	double maxtimewait;
	bool endParcourt;
	bool backHome;
	bool isblocked;
	bool via;
	bool endConstr;
	bool inConstr;
	bool over;
	Points *ListOfGoals;
} Goals;

typedef struct Tower {
    double tower_pos; 
	double last_rising[NB_STORE_EDGE];  
	double last_falling[NB_STORE_EDGE]; 
	int rising_index;  
	int falling_index;
	int nb_rising;  
	int nb_falling; 
	int nb_opponents; 
	double last_rising_fixed[NB_STORE_EDGE];
	double last_falling_fixed[NB_STORE_EDGE];
	int rising_index_fixed;  
	int falling_index_fixed; 
	int nb_rising_fixed;  
	int nb_falling_fixed; 
	double angle;
	double distance;
} Tower;


/// Main controller structure
typedef struct CtrlStruct
{
    double previousTimeCAN;
    int robotID;
    double previousTime;
    double timeStep;
    double time;
    double timeOffset;
#ifndef REALBOT
	CtrlIn *inputs;   ///< controller inputs
	CtrlOut *outputs; ///< controller outputs
#endif // !REALBOT
	enum StateCalib stateCalib;
	enum StateReCalib stateReCalib;
	enum StateStrat stateStrat;
	enum StateVia stateVia;
        enum StateHomologation stateHomologation;
        enum StateAction1 stateAction1;
        enum StateAction2 stateAction2;
        enum StateAction3 stateAction3;
        enum StateAction4 stateAction4;
        enum StateStrategy stateStrategy;
	Parametres *Param;
	Potential *Poto;
	Odometry *Odo;
	Sensors *Sensors;
	Motor *MotorR;
	Motor *MotorL;
	Motor *MotorTower;
#ifdef REALBOT
    Motor *MotorRatL;
    Motor *MotorRatR;
    Motor *MotorPince;
    Dyna *DynaLeft;
    Dyna *DynaRight;
#endif // REALBOT
	Obstacles *Obstacles;
	Tower *Tower;
	Goals *Goals;
} CtrlStruct;


// function prototypes
#ifndef REALBOT
CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs);
void free_CtrlStruct(CtrlStruct *cvs);

NAMESPACE_CLOSE();
#endif // !REALBOT

#endif
