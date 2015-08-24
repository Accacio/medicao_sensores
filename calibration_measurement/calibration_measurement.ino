#define NUM_READS 100
#include<Servo.h>
int cont_high=0;
int cont_low=0;
int percent_high=0;
int flag=0;
int cont_cycle=0;
float PWM_value;
float define_potref=625-30;
int menu_var1=-1;

Servo servooldg;

void setup() 
{
  servooldg.attach(2,2065,974); //974
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
}

void selection_menu()
{
  Serial.println("Press c to Calibration, or m to Measurement");
  while (menu_var1==-1||(menu_var1!=99 && menu_var1!=67 && menu_var1!=77 && menu_var1!=109))
  {
    menu_var1=Serial.read();
  }

  if (menu_var1==99||menu_var1==67)
  {
    Serial.println("Press p to calibrate Motor Pot, or s to calibrate Current Sensor");
    while (menu_var1==-1||(menu_var1!=80 && menu_var1!=112 && menu_var1!=83 && menu_var1!=115))
    {
      menu_var1=Serial.read();
    }
  }
}

void loop() 
{
  selection_menu();
  switch (menu_var1) {
    case 77:
    case 109:  
      measurement();  
      break;
    case 80:
    case 112: 
      calibrate_pot(); 
      selection_menu();
      break;
    case 83:
    case 115:
      calibrate_sensor();
      selection_menu();
      break;
}
  

  


}

void readSensors(float returnval[9]){
   // read multiple values of three sensors at same time and sort them to take the mode

   int vecvalue_vref[NUM_READS];
   int vecvalue_vpot[NUM_READS];
   int vecvalue_vim[NUM_READS];
   int vecvalue_potref[NUM_READS];
      
   for(int i=0;i<NUM_READS;i++){
     vecvalue_vref[i] = analogRead(A10);
     vecvalue_vpot[i] = analogRead(A11);
     vecvalue_vim[i] = analogRead(A12);
     vecvalue_potref[i] = analogRead(A13);
     delayMicroseconds(10);     
    }

   returnval[1] = filter(vecvalue_vref); //value for vref
   returnval[2] = filter(vecvalue_vpot); //value for vpot
   returnval[3] = filter(vecvalue_vim); //value for vim
   returnval[4] = filter(vecvalue_potref); //value for potref

   returnval[5] = 0;//value for mean of vref
   returnval[6] = 0;//value for mean of vpot
   returnval[7] = 0;//value for mean of vim
   returnval[8] = 0;//value for mean of potref
   
  for(int i=0;i<NUM_READS;i++){

    returnval[5] +=vecvalue_vref[i];
    returnval[6] +=vecvalue_vpot[i];
    returnval[7] +=vecvalue_vim[i];
    returnval[8] +=vecvalue_potref[i];
  }
    returnval[5] =returnval[5]/NUM_READS;
    returnval[6] =returnval[6]/NUM_READS;
    returnval[7] =returnval[7]/NUM_READS;
    returnval[8] =returnval[8]/NUM_READS;

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

void measurement(){

if (cont_cycle%(cont_high+cont_low)<cont_high)
{
  PWM_value=1.8*percent_high;
}
if (cont_cycle%(cont_high+cont_low)>=cont_high)
{
  PWM_value=0;
}
cont_cycle++;
  
float values[9];  
readSensors(values);
float vref=1.989*values[1];
float vpot=values[2]-30;
float vim=values[3];
float potref=values[4];

float vref_mean=1.989*values[5];
float vpot_mean=values[6];
float vim_mean=values[7];
float potref_mean=values[8];

//float PWMvalue=(vpot*180)/(1023);
servooldg.write(PWM_value);
float im=30*(vim/vref)-15;
float im_mean=30*((vim_mean)/(vref_mean))-15;

float real_pot=((vpot*vref)/(define_potref*1023))*100;
float real_pot_mean=((vpot_mean*potref_mean)/(vref_mean*1023)-0.0195)*100;

Serial.print(PWM_value);
Serial.print(',');
Serial.print(vref);
Serial.print(',');
Serial.print(vim);
Serial.print(',');
Serial.print(vpot);
Serial.print(',');
Serial.print(real_pot);
Serial.print(',');
Serial.print(vref_mean);
Serial.print(',');
Serial.print(vim_mean);
Serial.print(',');
Serial.print(real_pot_mean);
Serial.print(',');
if (flag==1)
{
  Serial.print(cont_high);    
  Serial.print(',');
  Serial.print(percent_high);
  flag=0;
}
Serial.println();
}

void serialEvent() 
{
  while(Serial.available())
  {
      if(menu_var1==77||menu_var1==109)
      {
        cont_high = cont_low = Serial.parseInt();
        percent_high = Serial.parseInt();
        if (Serial.read()=='\n') 
        {
          flag=1;
        }
      }
   }        
}

void calibrate_pot()
{
  Serial.print("Just Calibrated Pot\n");
}

void calibrate_sensor()
{
  Serial.print("Just Calibrated Sensor\n");
}



