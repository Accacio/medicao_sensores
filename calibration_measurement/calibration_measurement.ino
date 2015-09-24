#define NUM_READS 100
#define VPOT_IN A11
#define VREF_IN A10
#define VIM_IN A12
#define POTREF_IN A13
#define MPOS_DC 32//29// xxx
#define MPOS_MAX 619//604 //668
#define CURRENTBIT_DC 109//218//109
#define CURRENT_GAIN 5.6
#define PI 3.14
#define const_time 100


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
float PWM_value=0;
int comparador=5;
int menu_var=-1;

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
  Serial.println("Press c to Calibration, or m to Measurement");
  while (menu_var==-1||(menu_var!=99 && menu_var!=67 && menu_var!=77 && menu_var!=109))
  {
    menu_var=Serial.read();
  }
  if (menu_var==99||menu_var==67)
  {
    Serial.println("Press p to calibrate Motor Pot, or s to calibrate Current Sensor");
    while (menu_var==-1||(menu_var!=80 && menu_var!=112 && menu_var!=83 && menu_var!=115))
    {
      menu_var=Serial.read();
    }
  }
}

void loop()
{
  switch (menu_var) {
    case 77:
    case 109:
    measurement();
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
    default:
      selection_menu();
}





}

void readSensors(int returnval_int[9]){
   // read multiple values of three sensors at same time and sort them to take the mode

   int vecvalue_vref[NUM_READS];
   int vecvalue_vpot[NUM_READS];
   int vecvalue_vim[NUM_READS];
   int vecvalue_potref[NUM_READS];


   for(int i=0;i<NUM_READS;i++)
   {
     delayMicroseconds(10);
     vecvalue_vref[i] = analogRead(VREF_IN);
     vecvalue_vpot[i] = analogRead(VPOT_IN);
     vecvalue_potref[i] = analogRead(POTREF_IN);
     vecvalue_vim[i] = analogRead(VIM_IN);
    }

    returnval_int[1] = int(filter(vecvalue_vref)+0.5); //value for vref
    returnval_int[2] = int(filter(vecvalue_vpot)+0.5); //value for vpot
    returnval_int[3] = int(filter(vecvalue_vim)+0.5); //value for vim
    returnval_int[4] = int(filter(vecvalue_potref)+0.5); //value for potref

    returnval_int[5] = 0;//value for mean of vref
    returnval_int[6] = 0;//value for mean of vpot
    returnval_int[7] = 0;//value for mean of vim
    returnval_int[8] = 0;//value for mean of potref

    float aux_val[9];
    aux_val[5] =0;
    aux_val[6] =0;
    aux_val[7] =0;
    aux_val[8] =0;

  for(int i=0;i<NUM_READS;i++){

    aux_val[5] +=vecvalue_vref[i];
    aux_val[6] +=vecvalue_vpot[i];
    aux_val[7] +=vecvalue_vim[i];
    aux_val[8] +=vecvalue_potref[i];
  }

    returnval_int[5] =int(aux_val[5]/NUM_READS+0.5);
    returnval_int[6] =int(aux_val[6]/NUM_READS+0.5);
    returnval_int[7] =int(aux_val[7]/NUM_READS+0.5);
    returnval_int[8] =int(aux_val[8]/NUM_READS+0.5);

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
int values_int[9];
float vref, vpot, vim, potref ,vref_mean, vpot_mean, vim_mean, potref_mean, pot_raw, im, im_mean;
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
  register_vpot[comparador-1]=values_int[2];
 }while(values_int[2]!=equal_mean);


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
vref=1.989*values_int[1];
vpot=100*((values_int[2]*1.0-MPOS_DC)/(MPOS_MAX-MPOS_DC));
im=((values_int[3]-CURRENTBIT_DC)*(5000/(CURRENT_GAIN*1023.00)))/0.167;
potref=values_int[4];
vref_mean=1.989*values_int[5];
vpot_mean=100*((values_int[6]*1.0-MPOS_DC)/(MPOS_MAX-MPOS_DC));
im_mean=((values_int[7]-CURRENTBIT_DC)*(5000/(CURRENT_GAIN*1023.00)))/0.167;
potref_mean=values_int[8];
pot_raw=values_int[2];

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
  int values_int[9];
  int equal_mean=0;
  int ascending=0;
  int descending=0;
  int equal=0;
  int register_vpot[comparador];

  servooldg.write(angle);
  for(int i=0;i<comparador;i++)
  {
    readSensors(values_int);
    register_vpot[i] = values_int[2];
    delay(20);
  }

  do{
    servooldg.write(angle);
    delay(200);
    equal_mean=0;
    ascending=0;
    descending=0;
    readSensors(values_int);
    int aux=values_int[2];
    for(int i=0;i<comparador;i++)
    {
      equal_mean+=register_vpot[i];
      ascending+=aux>=register_vpot[i];
      descending+=aux<=register_vpot[i];
    }
    equal_mean=equal_mean/comparador;

      Serial.print(values_int[6]);
      Serial.print('-');
      Serial.println(equal_mean);


    for(int i=1;i<comparador;i++)
    {
      register_vpot[i-1]=register_vpot[i];
    }
    register_vpot[comparador-1]=values_int[2];
   }while(values_int[2]!=equal_mean);
    //}while(ascending==comparador || descending==comparador);
  Serial.print(values_int[6]);
  Serial.print(',');
  Serial.println(equal_mean);

  for(int i=0;i<comparador;i++)
  {
    readSensors(values_int);
    register_vpot[i] = values_int[2];
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
  int values_int[9];
  float current_mean;
  float current_filtered;
  while(true)
  {

  readSensors(values_int);
  current_filtered=((values_int[3]-CURRENTBIT_DC)*(5000/(CURRENT_GAIN*1023.00)))/0.167;
  current_mean=((values_int[7]-CURRENTBIT_DC)*(5000/(CURRENT_GAIN*1023.00)))/0.167;
 // Serial.print("Filtered Measure of Current Sensor is ");
  Serial.print(values_int[3]);
  Serial.print(",");
  Serial.print(values_int[7]);
  Serial.print(",");
  Serial.print(current_filtered);
  Serial.print(",");

  //Serial.print("Mean Measure of Current Sensor is ");
  Serial.println(current_mean);
  }
}
