#include <Servo.h>

#define NUM_READS 100 // number of read measures to process the signal

// Pin Numbers
#define VPOT_IN A11
#define VREF_IN A10
#define VIM_IN A12
#define POTREF_IN A13
#define LOADCELL_IN A15


// Constants
#define MPOS_DC 32//29// xxx
#define MPOS_MAX 619//604 //668
#define CURRENTBIT_DC 109//218//109
#define CURRENT_GAIN 5.6
#define PI 3.14
#define G 10   //Gravidade
#define const_time 100
//Definitions for the Load Cell
#define LC_BIT_MIN  23
#define LC_BIT_MAX  1023
#define LC_NEWTON_MIN 1.475*G
#define LC_NEWTON_MAX 50*G
//Definitions for the elbow
#define FULL_OPEN_ELBOW 47  // value in PWM to full open the elbow angle
#define FULL_OPEN_COMPEN 2 // to compensante the Hysteresis in the elbow
#define FULL_CLOSE_ELBOW 0  // value in PWM to close the elbow angle
#define MAX_ELBOW_ANGLE 141*PI/180 // Max aperture of the elbow angle measure externally
#define MIN_ELBOW_ANGLE 40*PI/180  // Min aperture of the elbow angle, measured exernally
#define ANGLE_VPOT_MAX  185       // Value in bits of the vpot when is the maximum angle on the elbow
#define ANGLE_VPOT_COMPEN 6     // to compensate the Hysteresis no angle measure
#define ANGLE_VPOT_MIN  30       // Value in bits of the vpot when is the min angle on the elbow
//Definitions for the Arm
#define DCA 5.75    //Distance of the arm clamping
#define DCF 4.75    //Distance of the forearm clamping
#define PFCM 0.682   //Forearm center of mass
#define PFW  0.022   //Forearm proportional weight


enum sensor_array
{
  ar_vref=1,
  ar_vref_mean,
  ar_vpot,
  ar_vpot_mean,
  ar_vim,
  ar_vim_mean,
  ar_potref,
  ar_potref_mean,
  ar_vloadcell,
  ar_vloadcell_mean,
  ar_last,
};


//angular definitions
float last_angle;
float last_aspeed;


unsigned long t0_time;
unsigned long t1_time;
unsigned long t_time;
int cont_high=100;
int cont_low=100;
int percent_high=20;
int flag=0;
int cont_cycle=0;
int cont_frvar=0;
float PWM_value=FULL_OPEN_ELBOW;
int comparador=5;
int menu_var=-1;


//subject defintions
float Subject_weight=70;
float La=0.28;
float Lf=0.26;
float Lh=0.08;

//definitions to calibrate control law
float Sgm_left_lim=-2;    //defintion about the left limit of the sigmoid function on the control law
float Sgm_right_lim=3;    //defintion about the right limit of the sigmoid function on the control law 

//initialization of x_max for the extension of the arm tensor
const float  Traj_x_min=sqrt(pow(DCA,2)+pow(DCF,2)-2*DCA*DCF*cos(MIN_ELBOW_ANGLE));
const float  Traj_x_max=sqrt(pow(DCA,2)+pow(DCF,2)-2*DCA*DCF*cos(MAX_ELBOW_ANGLE))-Traj_x_min;
int    Traj_angle=ANGLE_VPOT_MAX-ANGLE_VPOT_MIN;

Servo servooldg;





void setup()
{

//analogReference(INTERNAL2V56);
  servooldg.attach(2,MPOS_DC,970); //20, 965
  servooldg.write(PWM_value);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
}

void selection_menu()
{
  Serial.println("Press c to Calibration, a for arm movement, or m to Measurement");
  while (menu_var==-1||(menu_var!=99 && menu_var!=67 && menu_var!=77 && menu_var!=109 && menu_var!=97 && menu_var!=65 && menu_var!=84 && menu_var!=116))
  {
    menu_var=Serial.read();
  }
  if (menu_var==99||menu_var==67)
  {
    Serial.println("Press l to calibrate Load Cell, p to calibrate Motor Pot, s to calibrate Current Sensor, or e to calibrate elbow angle");
  while (menu_var==-1||(menu_var!=76 && menu_var!=108 && menu_var!=80 && menu_var!=112 && menu_var!=83 && menu_var!=115 && menu_var!=69 && menu_var!=101))
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
      deterministic_model();
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
      calibrate_pot(5);
      selection_menu();
      break;
    case 83:
    case 115:
      calibrate_sensor();
      selection_menu();
      break;
    case 97:
    case 65:
      arm_movement();
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
        //      Serial.print(cont_high);
        //      Serial.print(percent_high);
        //      Serial.print(flag);
      }
   }
}
