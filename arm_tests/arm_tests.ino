#define NUM_READS 100
#define VPOT_IN A11
#define VREF_IN A10
#define VIM_IN A12
#define POTREF_IN A13
#define LOADCELL_IN A15
#define MPOS_DC 32//29// xxx
#define MPOS_MAX 619//604 //668
#define CURRENTBIT_DC 109//218//109
#define CURRENT_GAIN 5.6
#define LOADCELL_DC 1
#define LOADCELL_GAIN 220
#define PI 3.14
#define const_time 100
#define FULL_OPEN_ELBOW 53  // value in PWM to full open the elbow angle
#define FULL_CLOSE_ELBOW 0  // value in PWM to close the elbow angle
#define MAX_ELBOW_ANGLE 142*PI/180 // Max aperture of the elbow angle measure externally
#define MIN_ELBOW_ANGLE 33*PI/180  // Min aperture of the elbow angle, measured exernally
#define ANGLE_VPOT_MAX  194       // Value in bits of the vpot when is the maximum angle on the elbow
#define ANGLE_VPOT_MIN  24       // Value in bits of the vpot when is the min angle on the elbow
#define DCA 5.75    //Distance of the arm clamping
#define DCF 4.75    //Distance of the forearm clamping

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

#include<Servo.h>
unsigned long t0_time;
unsigned long t1_time;
unsigned long t_time;
int cont_high=100;
int cont_low=0;
int percent_high=100;
int flag=0;
int cont_cycle=0;
int cont_frvar=0;
float PWM_value=FULL_OPEN_ELBOW;
int comparador=5;
int menu_var=-1;
//initialization of x_max for the extension of the arm tensor
const float  Traj_x_min=sqrt(pow(DCA,2)+pow(DCF,2)-2*DCA*DCF*cos(MIN_ELBOW_ANGLE));
const float  Traj_x_max=sqrt(pow(DCA,2)+pow(DCF,2)-2*DCA*DCF*cos(MAX_ELBOW_ANGLE))-Traj_x_min;
const int    Traj_angle=ANGLE_VPOT_MAX-ANGLE_VPOT_MIN;

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
  while (menu_var==-1||(menu_var!=99 && menu_var!=67 && menu_var!=77 && menu_var!=109 && menu_var!=97 && menu_var!=65))
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

void readSensors(int returnval_int[11]){
   // read multiple values of three sensors at same time and sort them to take the mode

   int vecvalue_vref[NUM_READS];
   int vecvalue_vpot[NUM_READS];
   int vecvalue_vim[NUM_READS];
   int vecvalue_potref[NUM_READS];
   int vecvalue_vloadcell[NUM_READS];


   for(int i=0;i<NUM_READS;i++)
   {
     delayMicroseconds(10);
     vecvalue_vref[i] = analogRead(VREF_IN);
     vecvalue_vpot[i] = analogRead(VPOT_IN);
     vecvalue_potref[i] = analogRead(POTREF_IN);
     vecvalue_vim[i] = analogRead(VIM_IN);
     vecvalue_vloadcell[i] = analogRead(LOADCELL_IN);
    }

    returnval_int[ar_vref] = int(filter(vecvalue_vref)+0.5); //value for vref
    returnval_int[ar_vpot] = int(filter(vecvalue_vpot)+0.5); //value for vpot
    returnval_int[ar_vim] = int(filter(vecvalue_vim)+0.5); //value for vim
    returnval_int[ar_potref] = int(filter(vecvalue_potref)+0.5); //value for potref
    returnval_int[ar_vloadcell] = int(filter(vecvalue_vloadcell)+0.5); //value for load cell

    returnval_int[ar_vref_mean] = 0;//value for mean of vref
    returnval_int[ar_vpot_mean] = 0;//value for mean of vpot
    returnval_int[ar_vim_mean] = 0;//value for mean of vim
    returnval_int[ar_potref_mean] = 0;//value for mean of potref
    returnval_int[ar_vloadcell_mean] = 0;//value for mean of load cell

    float aux_val[ar_last];
    aux_val[ar_vref_mean] =0;
    aux_val[ar_vpot_mean] =0;
    aux_val[ar_vim_mean] =0;
    aux_val[ar_potref_mean] =0;
    aux_val[ar_vloadcell_mean] =0;

  for(int i=0;i<NUM_READS;i++){

    aux_val[ar_vref_mean] +=vecvalue_vref[i];
    aux_val[ar_vpot_mean] +=vecvalue_vpot[i];
    aux_val[ar_vim_mean] +=vecvalue_vim[i];
    aux_val[ar_potref_mean] +=vecvalue_potref[i];
    aux_val[ar_vloadcell_mean] +=vecvalue_vloadcell[i];
  }

    returnval_int[ar_vref_mean] =int(aux_val[ar_vref_mean]/NUM_READS+0.5);
    returnval_int[ar_vpot_mean] =int(aux_val[ar_vpot_mean]/NUM_READS+0.5);
    returnval_int[ar_vim_mean] =int(aux_val[ar_vim_mean]/NUM_READS+0.5);
    returnval_int[ar_potref_mean] =int(aux_val[ar_potref_mean]/NUM_READS+0.5);
    returnval_int[ar_vloadcell_mean] =int(aux_val[ar_vloadcell_mean]/NUM_READS+0.5);

   //return returnval;
}

float filter(int raw_val[]){
  //sorting the array
  for(int i=0; i<(NUM_READS-1); i++) {
        for(int j=0; j<(NUM_READS-(i+1)); j++) {
                if(raw_val[j] > raw_val[j+1]) {
                    int aux = raw_val[j];
                    raw_val[j] = raw_val[j+1];
                    raw_val[j+1] = aux;
                }
        }
    }

    // median of the 20 center values of the array
    float return_filvalue=0;
    int cont=0;
    for(int i=NUM_READS/2-10;i<(NUM_READS/2+10);i++){
      return_filvalue +=raw_val[i];
      cont++;
   }
   return_filvalue=return_filvalue/20;

   return return_filvalue;
}

void square_wave()
{
  if (cont_cycle%(cont_high+cont_low)<cont_high)
  {
    PWM_value=1.8*percent_high;
  }
//  if (cont_cycle==0||cont_cycle%(cont_high+cont_low)>=cont_high)
   if (cont_cycle==0||cont_cycle%(cont_high+cont_low)>=cont_high)
  {
    PWM_value=9;
  }
}

void sine_wave()
{
  if (cont_high==0)
  {
    PWM_value=0;
  }
  else
  {
    PWM_value=(1.8/2)*percent_high*(1-cos(cont_cycle*PI/cont_high));
  }
}

void sine_wave_fqvar()
{
  if (cont_high==0)
  {
    PWM_value=0;
  }
  else
  {
    PWM_value=(1.8/2)*percent_high*(1-cos(cont_cycle*PI/cont_high));
    if(cont_frvar>=2*cont_high){
      cont_high=cont_high/2;
      cont_frvar=0;
    }
  }
}

void measurement(){
int values_int[ar_last];
float vref, vpot, vim, potref ,vref_mean, vpot_mean, vim_mean, potref_mean, pot_raw, im, im_mean, loadcell, loadcell_mean;
int register_vpot[comparador];
int equal_mean;
do{
  servooldg.write(0);
  delay(100);
  readSensors(values_int);
  equal_mean=0;
  for(int i=0;i<comparador;i++)
  {
    equal_mean+=register_vpot[i];
  }
  equal_mean=equal_mean/comparador;

  for(int i=1;i<comparador;i++)
  {
    register_vpot[i-1]=register_vpot[i];
  }
  register_vpot[comparador-1]=values_int[ar_vpot];
}while(values_int[ar_vpot]!=equal_mean);


cont_cycle=0;
cont_frvar=0;
t0_time=millis();

do{
square_wave();
//sine_wave();
//sine_wave_fqvar();

servooldg.write(PWM_value);
cont_cycle++;
cont_frvar++;

readSensors(values_int);
// Conversion from bits to values
vref=1.989*values_int[ar_vref];
vpot=100*((values_int[ar_vpot]*1.0-MPOS_DC)/(MPOS_MAX-MPOS_DC));
im=((values_int[ar_vim]-CURRENTBIT_DC)*(5000/(CURRENT_GAIN*1023.00)))/0.167;
potref=values_int[ar_potref];
vref_mean=1.989*values_int[ar_vref_mean];
vpot_mean=100*((values_int[ar_vpot_mean]*1.0-MPOS_DC)/(MPOS_MAX-MPOS_DC));
im_mean=((values_int[ar_vim_mean]-CURRENTBIT_DC)*(5000/(CURRENT_GAIN*1023.00)))/0.167;
potref_mean=values_int[ar_potref_mean];
pot_raw=values_int[ar_vpot];
//loadcell=(values_int[ar_vloadcell]-LOADCELL_DC)*10;
//loadcell_mean=(values_int[ar_vloadcell_mean]-LOADCELL_DC)*10;
loadcell=values_int[ar_vloadcell];
loadcell_mean=values_int[ar_vloadcell_mean];

//Sending information over serial
Serial.print(PWM_value);
Serial.print(',');
//Serial.print(vref);
//Serial.print(',');
//Serial.print(pot_raw);
//Serial.print(',');
Serial.print(vpot);//vpot_int
Serial.print(',');
//Serial.print(vref_mean);
//Serial.print(',');
Serial.print(vpot_mean);//vpot_mean int
Serial.print(',');
Serial.print(im);
Serial.print(',');
Serial.print(im_mean);
Serial.print(',');
Serial.print(loadcell);
Serial.print(',');
Serial.print(loadcell_mean);
Serial.print(',');
Serial.print(t_time);
Serial.println(',');
do{
  delayMicroseconds(500);
  t1_time=millis();
  t_time=t1_time-t0_time;
}while(t_time<const_time);
t0_time=t1_time;
if (Serial.available())
{
  serialEvent();
}
}while(menu_var>0);

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


void get_pot_value(float angle)
{
  int values_int[ar_last];
  int equal_mean=0;
  int ascending=0;
  int descending=0;
  int equal=0;
  int register_vpot[comparador];

  servooldg.write(angle);
  for(int i=0;i<comparador;i++)
  {
    readSensors(values_int);
    register_vpot[i] = values_int[ar_vpot];
    delay(20);
  }

  do{
    servooldg.write(angle);
    delay(200);
    equal_mean=0;
    ascending=0;
    descending=0;
    readSensors(values_int);
    int aux=values_int[ar_vpot];
    for(int i=0;i<comparador;i++)
    {
      equal_mean+=register_vpot[i];
      ascending+=aux>=register_vpot[i];
      descending+=aux<=register_vpot[i];
    }
    equal_mean=equal_mean/comparador;

      Serial.print(values_int[ar_vpot_mean]);
      Serial.print('-');
      Serial.println(equal_mean);


    for(int i=1;i<comparador;i++)
    {
      register_vpot[i-1]=register_vpot[i];
    }
    register_vpot[comparador-1]=values_int[ar_vpot];
  }while(values_int[ar_vpot]!=equal_mean);
    //}while(ascending==comparador || descending==comparador);
  Serial.print(values_int[ar_vpot_mean]);
  Serial.print(',');
  Serial.println(equal_mean);

  for(int i=0;i<comparador;i++)
  {
    readSensors(values_int);
    register_vpot[i] = values_int[ar_vpot];
    delayMicroseconds(10);
  }

 int measure=0;
  for(int i=0;i<comparador;i++){
    measure+=register_vpot[i];
  }
  measure=measure/comparador;
  Serial.print("Measure of ");
  Serial.print(angle);
  Serial.print("degrees is: ");
  Serial.println(measure);
}

void calibrate_loadcell()
{
  Serial.println("Calibrei a celula de carga");
}


void calibrate_pot(int num_measures)
{
  servooldg.write(0);
  delay(200);
  float fract_angle=180/(num_measures-1);
  for(float i=0;i<181;i+=fract_angle){
    get_pot_value(i);
  }


}

void calibrate_sensor()
{
  int values_int[ar_last];
  float current_mean;
  float current_filtered;
  while(true)
  {

  readSensors(values_int);
  current_filtered=((values_int[ar_vim]-CURRENTBIT_DC)*(5000/(CURRENT_GAIN*1023.00)))/0.167;
  current_mean=((values_int[ar_vim_mean]-CURRENTBIT_DC)*(5000/(CURRENT_GAIN*1023.00)))/0.167;
 // Serial.print("Filtered Measure of Current Sensor is ");
  Serial.print(values_int[ar_vim]);
  Serial.print(",");
  Serial.print(values_int[ar_vim_mean]);
  Serial.print(",");
  Serial.print(current_filtered);
  Serial.print(",");

  //Serial.print("Mean Measure of Current Sensor is ");
  Serial.println(current_mean);
  }
}

//----------- Functions related to the elbow---------------------
//----------Elbow movement----------------
void arm_movement(){
  int elbow_angle;
  int values_int[ar_last];
//  PWM_value=FULL_OPEN;
  Serial.print("Define the aperture of the elbow angle (degres) between ");
  Serial.print(MAX_ELBOW_ANGLE*180/PI); 
  Serial.print(" and "); 
  Serial.println(MIN_ELBOW_ANGLE*180/PI);

  do{
  if (Serial.available()){
    elbow_angle= Serial.parseInt();
    Serial.println(elbow_angle);
    if(elbow_angle<0){
        break;
     }
    if (Serial.read()=='\n'){
      flag=1;
    }
    Serial.println(elbow_angle);
    set_elbow_angle(elbow_angle);
    Serial.print("Define the aperture of the elbow angle (degres) between ");
    Serial.print(MAX_ELBOW_ANGLE*180/PI); 
    Serial.print(" and "); 
    Serial.println(MIN_ELBOW_ANGLE*180/PI);
    
  }
  readSensors(values_int);
  float angle_calculated=read_elbow_angle(values_int[ar_vpot]);
  Serial.print(angle_calculated*180/PI);
  Serial.print(",");
  Serial.print(values_int[ar_vpot]);
  Serial.print(",");
  Serial.println(values_int[ar_vloadcell]*1.0/10);
 }while(1);
 }

//------function for elbow calibration values and constants
 void calibrate_elbow_angle(){
    PWM_value=FULL_OPEN_ELBOW;
    Serial.println(PWM_value);
    servooldg.write(PWM_value);
    delay(20);
    do{
    //menu of elbow calibration
    Serial.println("Choose an action for calibration:");
    Serial.println("  1) Free movement of elbow PWM bits, and measures");
    Serial.println("  2) Confirmation to test choosen values for elbow angle movement calibration");
    Serial.println("  3) Under construction");
    Serial.println(" -1) To exit calibration tests");
    int menu_value=0;
    do{
      if (Serial.available()){
        menu_value= Serial.parseInt();
        Serial.println(menu_value);
        if (Serial.read()=='\n'){}
        }
    }while(menu_value==0);
    if(menu_value<0){
        break;
     }
    switch (menu_value){
      case 1:
      elbow_calibration_menu1();
      break;
      case 2:
      elbow_calibration_menu2();
      break;
      case 3:

      break;
    }
    }while(1);
 }
 
//intern subfuction of elbow menu
void elbow_calibration_menu1(){
  int arm_pos;
  int values_int[ar_last];
  Serial.println("Enter the value that is required to move from 0 to 180 (bits). Always take count the position of the motor's piston:");
  Serial.println("Enter -1 to exit to the calibration menu");
  do{
    if (Serial.available()){
      arm_pos= Serial.parseInt();
      if(arm_pos<0){
        break;
      }
      if (Serial.read()=='\n'){}
      Serial.println(arm_pos);
      PWM_value=arm_pos;
      servooldg.write(PWM_value);
      delay(20);
      Serial.println("Enter the value that is required to move from 0 to 180. Always take count the position of the motor's piston:");
      Serial.println("Enter -1 to exit to the calibration menu");
    }
 readSensors(values_int);
 Serial.print("Vpot= ");
 Serial.println(values_int[ar_vpot]);
 }while(1);
}

//intern subfuction of elbow menu 
void elbow_calibration_menu2(){
  int aux_angle_max;
  int aux_angle_min;
  int aux_full_open;
  int aux_vpot_max=ANGLE_VPOT_MAX;
  int aux_vpot_min=ANGLE_VPOT_MIN;
  float aux_angle;
  int values_int[ar_last];
  float angle_read;
  int aux_Traj_angle;
  float x_tensor_read;
  Serial.println("For test enter, full open elbow value(bits), max open elbow (degrees), min open elbow (degrees) and desired angle");
  Serial.println("Enter -1 to exit to the calibration menu");
  do{
    if (Serial.available()){
      aux_full_open= Serial.parseInt();
      aux_angle_max= Serial.parseInt();
      aux_angle_min=Serial.parseInt();
      aux_vpot_max=Serial.parseInt();
      aux_vpot_min=Serial.parseInt();
      aux_angle= Serial.parseInt();
      if(aux_full_open<0){
        break;
      }
      if (Serial.read()=='\n'){} 
      Serial.print(aux_full_open);
      Serial.print(',');
      Serial.print(aux_angle_max);
      Serial.print(',');
      Serial.print(aux_angle_min);
      Serial.print(',');
      Serial.print(aux_vpot_max);
      Serial.print(',');
      Serial.print(aux_vpot_min);
      Serial.print(',');
      Serial.println(aux_angle);
    
      //Calculation of the x tensor values and the PWM value given the data entered by the user
      float Traj_x_min=sqrt(pow(DCA,2)+pow(DCF,2)-2*DCA*DCF*cos(aux_angle_min*PI/180));
      float Traj_x_max=sqrt(pow(DCA,2)+pow(DCF,2)-2*DCA*DCF*cos(aux_angle_max*PI/180))-Traj_x_min;
      float x_tensor=sqrt(pow(DCA,2)+pow(DCF,2)-2*DCA*DCF*cos(aux_angle*PI/180))-Traj_x_min;
      PWM_value=(x_tensor*aux_full_open)/(Traj_x_max);
      Serial.print("PWM setted: ");
      Serial.println(PWM_value);
    
      servooldg.write(PWM_value);
      delay(20);

      Serial.println("Set the maximum opening of the elbow angle in degres for calibration, followed by angle opening desired");
      Serial.println("Enter -1 to exit to the calibration menu");
    }
    delay(1000);
    readSensors(values_int);
    //Calculation for measure the elbow angle
    aux_Traj_angle=aux_vpot_max-aux_vpot_min;
    x_tensor_read=((values_int[ar_vpot]-aux_vpot_min)*Traj_x_max)/aux_Traj_angle+Traj_x_min;      
    angle_read=acos((pow(DCA,2)+pow(DCF,2)-pow(x_tensor_read,2))/(2*DCA*DCF));
    Serial.print(Traj_angle);
    Serial.print(", ");
    Serial.print(values_int[ar_vpot]);
    Serial.print(", ");
    Serial.print(x_tensor_read);
    Serial.print("Angle_measured: ");
    Serial.println(angle_read*180/PI);
  }while(1);
 }


 void set_elbow_angle(float angle_set){
  float aux_angle=angle_set*PI/180;
  if(aux_angle>=MIN_ELBOW_ANGLE && aux_angle<=MAX_ELBOW_ANGLE){
  float x_tensor=sqrt(pow(DCA,2)+pow(DCF,2)-2*DCA*DCF*cos(aux_angle))-Traj_x_min;
  PWM_value=(x_tensor*FULL_OPEN_ELBOW)/Traj_x_max;
  servooldg.write(PWM_value);
  delay(20);
  }
  else{
    Serial.print("Elbow angle out of limits, please enter again a value between: ");
    Serial.print(MAX_ELBOW_ANGLE*180/PI); 
    Serial.print(" and "); 
    Serial.println(MIN_ELBOW_ANGLE*180/PI);
  } 
  
 }

float read_elbow_angle(int Pot_value){
  float x_tensor=(Pot_value-ANGLE_VPOT_MIN)*Traj_x_max/Traj_angle+Traj_x_min;
  float angle_elbow=acos((pow(DCA,2)+pow(DCF,2)-pow(x_tensor,2))/(2*DCA*DCF));
  return angle_elbow; 
}

