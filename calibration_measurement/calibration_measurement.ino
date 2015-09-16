#define NUM_READS 100
#define VPOT_IN A11
#define VREF_IN A10
#define VIM_IN A12
#define POTREF_IN A13
#define MPOS_DC 29
#define MPOS_MAX 604 //668
#define CURRENTBIT_DC 110
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

//  analogReference(INTERNAL1V1);
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

void readSensors(int returnval[9]){
   // read multiple values of three sensors at same time and sort them to take the mode

   int vecvalue_vref[NUM_READS];
   int vecvalue_vpot[NUM_READS];
   int vecvalue_vim[NUM_READS];
   int vecvalue_potref[NUM_READS];


   for(int i=0;i<NUM_READS;i++)
   {
     vecvalue_vref[i] = analogRead(VREF_IN);
     vecvalue_vpot[i] = analogRead(VPOT_IN);
     vecvalue_potref[i] = analogRead(POTREF_IN);
     vecvalue_vim[i] = analogRead(VIM_IN);
     delayMicroseconds(10);

    }

   returnval[1] = int(filter(vecvalue_vref)+0.5); //value for vref
   returnval[2] = int(filter(vecvalue_vpot)+0.5); //value for vpot
   returnval[3] = int(filter(vecvalue_vim)+0.5); //value for vim
   returnval[4] = int(filter(vecvalue_potref)+0.5); //value for potref

   returnval[5] = 0;//value for mean of vref
   returnval[6] = 0;//value for mean of vpot
   returnval[7] = 0;//value for mean of vim
   returnval[8] = 0;//value for mean of potref

   float aux_val[9];

  for(int i=0;i<NUM_READS;i++){

    aux_val[5] +=vecvalue_vref[i];
    aux_val[6] +=vecvalue_vpot[i];
    aux_val[7] +=vecvalue_vim[i];
    aux_val[8] +=vecvalue_potref[i];
  }
    returnval[5] =int(aux_val[5]/NUM_READS+0.5);
    returnval[6] =int(aux_val[6]/NUM_READS+0.5);
    returnval[7] =int(aux_val[7]/NUM_READS+0.5);
    returnval[8] =int(aux_val[8]/NUM_READS+0.5);

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
  if (cont_cycle%(cont_high+cont_low)>=cont_high)
  {
    PWM_value=0;
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
int values[9];
float vref, vpot, vim, potref ,vref_mean, vpot_mean, vim_mean, potref_mean, pot_raw, im, im_mean;

do{
  servooldg.write(0);
  delay(10);
  readSensors(values);
//  Serial.println(values[6]);
}while(values[6]>MPOS_DC+1);

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

readSensors(values);
// Conversion from bits to values
vref=1.989*values[1];
vpot=100*((values[2]-MPOS_DC)/(MPOS_MAX-MPOS_DC));
vim=((values[3]-110)*(5000/(5.6*1023.00)))/0.167;
potref=values[4];
vref_mean=1.989*values[5];
vpot_mean=100*((values[6]-MPOS_DC)/(MPOS_MAX-MPOS_DC));
vim_mean=((values[7]-110)*(5000/(5.6*1023.00)))/0.167;
potref_mean=values[8];
pot_raw=values[2];
im=30*(vim/vref)-15;
im_mean=30*((vim_mean)/(vref_mean))-15;

//Sending information over serial
Serial.print(PWM_value);
Serial.print(',');
//Serial.print(vref);
//Serial.print(',');
//Serial.print(pot_raw);
//Serial.print(',');
//Serial.print(vim);
//Serial.print(',');
Serial.print(vpot);
Serial.print(',');
//Serial.print(vref_mean);
//Serial.print(',');
//Serial.print(vim_mean);
//Serial.print(',');
Serial.print(vpot_mean);
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
//        Serial.print(cont_high);
//        Serial.print(percent_high);
//        Serial.print(flag);
      }
   }
}


void get_pot_value(float angle)
{
  int values[9];
  int ascending=0;
  int descending=0;
  int equal=0;
  int register_vpot[comparador];
  for(int i=0;i<comparador;i++)
  {
    readSensors(values);
    register_vpot[i] = values[6];
  }

  do{
    servooldg.write(angle);
    delay(200);
    ascending=0;
    descending=0;
    readSensors(values);
    int aux=values[6];
    for(int i=0;i<comparador;i++)
    {

      ascending+=aux>register_vpot[i];
      descending+=aux<register_vpot[i];
    }

    for(int i=1;i<comparador;i++)
    {
      register_vpot[i-1]=register_vpot[i];
    }
    register_vpot[comparador-1]=values[6];
  }while(ascending==comparador || descending==comparador);
  for(int i=0;i<comparador;i++)
  {
    readSensors(values);
    register_vpot[i] = values[6];
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
  int values[9];
  float current_mean;
  float current_filtered;
  while(true)
  {

  readSensors(values);
  current_filtered=((values[3]-110)*(5000/(5.6*1023.00)))/0.167;
  current_mean=((values[7]-110)*(5000/(5.6*1023.00)))/0.167;
 // Serial.print("Filtered Measure of Current Sensor is ");
  Serial.print(values[3]);
  Serial.print(",");
  Serial.print(values[7]);
  Serial.print(",");
  Serial.print(current_filtered);
  Serial.print(",");

  //Serial.print("Mean Measure of Current Sensor is ");
  Serial.println(current_mean);
  }
}
