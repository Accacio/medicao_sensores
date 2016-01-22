
/*  Forearm Project movement with Damping Control by Collision (DCC)
  is a project to intervene in the movement trajectory of the exoesqueleton,
  in the moment that and an obstacle or disturbe exist over the trajectory the systems detects
  the disturbance and enters in a DCC control state to avoid injurance to the exosqueleton user. */

// Acronyms
//  -DCC  -> Damping Control by Collision
//  -LS   -> Least Square method/calculations




//------Libraries Section----
#include <EEPROMex.h>
#include <Servo.h>
#include <Filters.h>

//------Definitions Section---
#define NUM_READS 100       // number of read measures to process the signal
#define NUM_READS_FILTER 10 // number of filter only measures to process the signal

// Hardware Pins over Arduino
#define VPOT_IN A11
#define LOADCELL_IN A15

// Geral Constants definition
#define PI 3.14
#define G 9.8   //Gravidade
#define COMPARADOR 5

#define MPOS_DC 32//29// xxx
#define MPOS_MAX 970//604 //668
#define const_time 20


// Load Cell Constant Definitions
#define LC_BIT_MIN  23
#define LC_BIT_MAX  1023
#define LC_NEWTON_MIN 1.475*G
#define LC_NEWTON_MAX 50*G

//Definitions for the Exoeskeleton measures
#define DCA 0.0575    //Distance of the arm clamping
#define DCF 0.0475    //Distance of the forearm clamping
#define DCMF 0.682    //Forearm center of mass

//Utilisar read
int   FULL_OPEN_ELBOW   = EEPROM.readInt(0);  // 41;  // value in PWM to full open the elbow angle
int   FULL_OPEN_COMPEN  = EEPROM.readInt(2);  // 3 ;// to compensante the Hysteresis in the elbow
int   FULL_CLOSE_ELBOW  = 0;                  // value in PWM to close the elbow angle
float MAX_ELBOW_ANGLE = EEPROM.readInt(4)*PI/180;  //120*PI/180;   // Max aperture of the elbow angle measure externally
float MIN_ELBOW_ANGLE = EEPROM.readInt(6)*PI/180;  //40*PI/180;    // Min aperture of the elbow angle, measured exernally
int   ANGLE_VPOT_MAX    = EEPROM.readInt(8);  // 165 ;      // Value in bits of the vpot when is the maximum angle on the elbow
int   ANGLE_VPOT_COMPEN = EEPROM.readInt(10); // 6 ;    // to compensate the Hysteresis no angle measure
int   ANGLE_VPOT_MIN    = EEPROM.readInt(12); // 23  ;     // Value in bits of the vpot when is the min angle on the elbow



//----- Global Variables-------

//Angular Variables
float angle_array [3] = {0,0,0};
float speed_array [3] = {0,0,0};
float accel_array [3] = {0,0,0};
unsigned long angular_time[3] = {0,0,0};

// Hysteresis function variables
float Pwm_array [3] = {0,0,0};
float h1_array [3] = {0,0,0};
float h2_array [3] = {1,1,1};

// To define
unsigned long t0_time;
unsigned long t1_time;
unsigned long t_time;
unsigned long tfilter;
int cont_high=100;
int cont_low=100;
int pos_actual;
int pos_required;
int percent_high=20;
int flag=0;
int cont_cycle=0;
int cont_frvar=0;
float PWM_value=FULL_OPEN_ELBOW;
int menu_var=-1;

//subject defintions
float La=0.28;
float Lf=0.26;
float Lh=0.10;

//elbow movement global variables
float loadcell_value=0;
float T_theor=0;
float Controlled_elbow_angle=0;
float elbow_angle_required=FULL_OPEN_ELBOW/2;
int force_outbound_flag=1;
int angle_step=1;

//definitions to calibrate control law
float Sgm_left_lim=-2;    //defintion about the left limit of the sigmoid function on the control law
float Sgm_right_lim=3;    //defintion about the right limit of the sigmoid function on the control law
float LS_param_array[7]=
{ EEPROM.readFloat(14),
  EEPROM.readFloat(18),
  EEPROM.readFloat(22),
  EEPROM.readFloat(26),
  EEPROM.readFloat(30),
  EEPROM.readFloat(34),
  EEPROM.readFloat(38)
};   //= {-2.4885,-0.20436,0.54144,0.11204,-0.4336,-7.259,8.8801}; //{0.068796,-0.041852,0.50591,0.28639,-0.39691,-6.1495,3.9878};    //vector of parameters definitions obtained after LS calibration function

float Sgm_slope=2.0;
float Sgm_slope2=0.5; //2.5;
float tolerance;


//initialization of x_max for the extension of the arm tensor
const float  Traj_x_min=sqrt(pow(DCA,2)+pow(DCF,2)-2*DCA*DCF*cos(MIN_ELBOW_ANGLE));
const float  Traj_x_max=sqrt(pow(DCA,2)+pow(DCF,2)-2*DCA*DCF*cos(MAX_ELBOW_ANGLE))-Traj_x_min;
int    Traj_angle=ANGLE_VPOT_MAX-ANGLE_VPOT_MIN;

//---- Filters Components Initialization----
float fs_sensor=0.2;
FilterOnePole vpot_filter( LOWPASS, fs_sensor );
FilterOnePole loadcell_filter( LOWPASS, fs_sensor );

float fs_angle=1;
FilterOnePole angle_filter( LOWPASS, fs_angle );

float fs_speed=0.25;
FilterOnePole speed_filter( LOWPASS, fs_speed );

float fs_accel=0.05;
FilterOnePole accel_filter( LOWPASS, fs_accel );

float fs_hyster=0.08;
FilterOnePole h1_filter( LOWPASS, fs_hyster );
FilterOnePole h2_filter( LOWPASS, fs_hyster );

float fs_theorical=1;
//float fs_theorical=5;
FilterOnePole T_theorical_filter( LOWPASS, fs_theorical );

float fs_DCCangle=0.5;
FilterOnePole DCCv1_angle_filter( LOWPASS, fs_DCCangle );
FilterOnePole DCCv2_angle_filter( LOWPASS, fs_DCCangle );

//---- Servomotor Initialization-----
Servo servooldg;

void setup()
{
  servooldg.attach(2,MPOS_DC,MPOS_MAX);
  servooldg.write(PWM_value);

//initializing serial communication at 115200 bps
  Serial.begin(115200);

//initializing hysteresis filters inputs
  h1_filter.input(0);
  h2_filter.input(1);

//initializing Output LED DCC control state
  pinMode(51,OUTPUT);
  digitalWrite(51,HIGH);
}

void selection_menu()
{
  Serial.println("Press C to Calibration, A for Forearm movement, M to Measurement, Q for LS calibration, W to save parameters");
  while (menu_var==-1||(menu_var!=99 && menu_var!=67 && menu_var!=77 && menu_var!=109 && menu_var!=97 && menu_var!=65 && menu_var!=84 && menu_var!=116 && menu_var!=81 && menu_var!=113 && menu_var!=87 && menu_var!=119))
  {
    menu_var=Serial.read();
  }
  if (menu_var==99||menu_var==67)
  {
    Serial.println("Press L to calibrate Load Cell, P to calibrate Motor Pot, or E to calibrate elbow angle");
  while (menu_var==-1||(menu_var!=76 && menu_var!=108 && menu_var!=80 && menu_var!=112 && menu_var!=69 && menu_var!=101))
    {
      menu_var=Serial.read();
    }
  }
}

void loop()
{
  switch (menu_var) {
    case 84:
    case 116:
      Theorical_Model_fun(MAX_ELBOW_ANGLE);
      selection_menu();
      break;
    case 69:
    case 101:
      calibrate_elbow_angle();
      selection_menu();
      break;
    case 77:
    case 109:
    measurement();
    break;
    case 76:
    case 108:
      calibrate_loadcell();
      selection_menu();
      break;
    case 80:
    case 112:
      calibrate_pot();
      selection_menu();
      break;
    case 97:
    case 65:
      elbow_menu_movement();
      break;
    case 81:
    case 113:
      LS_parameters_finder();
      selection_menu();
      break;
    case 87:
    case 119:
      LS_parameters_set_to_eeprom();
      selection_menu();
      break;

    default:
      selection_menu();
  }
}

void serialEvent()
{
  while(Serial.available())
  {
      if(menu_var==77||menu_var==109||menu_var==71||menu_var==103)
      {
        cont_high = cont_low = Serial.parseInt();
        if(cont_high<0)
        {
          menu_var=-1;
        }
        percent_high = Serial.parseInt();
        if (Serial.read()=='\n')
        {
          flag=1;
        }

      }
   }
}
