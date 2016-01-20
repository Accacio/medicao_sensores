//Set of functions to measure sensors and positions

//---- Function Read Position, a sequence of function to obtain the angle position,
//angular speed, angular acceleration, and histeresis functions filtered
void updatePosition()
{
  readSensors_byfilters();
  angle_filter.input(read_elbow_angle(vpot_filter.output()));
  angular_measures(angle_filter.output());
  hysteresis_function(PWM_value);
}

//---- Function Read Sensors using Filters to elimnate the noise over the signal
void readSensors_byfilters()
{
  // Acquire sensor values
  for(int i=0;i<NUM_READS_FILTER;i++)
  {
    delayMicroseconds(10);
    vpot_filter.input(analogRead(VPOT_IN));
    loadcell_filter.input(analogRead(LOADCELL_IN));
  }
}

//---- Function Angular Measure used to update the values of Speed, and Acceleration in the system
void angular_measures (float angle)
{
  for (int i=0;i<2;i++)
  {
    angular_time[2-i]=angular_time[1-i];
    angle_array[2-i]=angle_array[1-i];
    speed_array[2-i]=speed_array[1-i];
    accel_array[2-i]=accel_array[1-i];
  }
  angular_time[0]=millis();
  angle_array[0]=angle;
  speed_array[0]= (angle_array[0]-angle_array[1])*1000/(angular_time[0]-angular_time[1]);
  accel_array[0]= (speed_array[0]-speed_array[1])*1000/(angular_time[0]-angular_time[1]);
  speed_filter.input(speed_array[0]);
  accel_filter.input(accel_array[0]);
}


//---- Function Hysteresis Values Calculation
void hysteresis_function(float Pwm_value)
{
  for (int i=0;i<2;i++)
  {
    Pwm_array[2-i]=Pwm_array[1-i];
    h1_array[2-i]=h1_array[1-i];
    h2_array[2-i]=h2_array[1-i];
  }
  Pwm_array[0]=Pwm_value;

  if (Pwm_array[0]<Pwm_array[1])
  {
    h1_array[0]=1;
    h2_array[0]=0;
  }
  else
  {
    if(Pwm_array[0]>Pwm_array[1])
    {
      h1_array[0]=0;
      h2_array[0]=1;
    }
    else
    {
      h1_array[0]=h1_array[1];
      h2_array[0]=h2_array[1];
    }
  }
  h1_filter.input(h1_array[0]);
  h2_filter.input(h2_array[0]);
}


//*change (meaning posibility to erase the following code)
//void readSensors(int returnval_int[14])
//{
//   // read multiple values of three sensors at same time and sort them to take the mode
//
//   int vecvalue_vref[NUM_READS];
//   int vecvalue_vpot[NUM_READS];
//   int vecvalue_vim[NUM_READS];
//   int vecvalue_potref[NUM_READS];
//   int vecvalue_vloadcell[NUM_READS];
//   int vecvalue_xaccell[NUM_READS];
//   int vecvalue_yaccell[NUM_READS];
//   int vecvalue_zaccell[NUM_READS];
//
//   // Acquire sensor values
//   for(int i=0;i<NUM_READS;i++)
//   {
//     delayMicroseconds(10);
//     vecvalue_vref[i] = analogRead(VREF_IN);
//     vecvalue_vpot[i] = analogRead(VPOT_IN);
//     vecvalue_potref[i] = analogRead(POTREF_IN);
//     vecvalue_vim[i] = analogRead(VIM_IN);
//     vecvalue_vloadcell[i] = analogRead(LOADCELL_IN);
//     vecvalue_xaccell[i] = analogRead(x_accel);
//     vecvalue_yaccell[i] = analogRead(y_accel);
//     vecvalue_zaccell[i] = analogRead(z_accel);
//     vpot_filter.input(vecvalue_vpot[i]);
//     loadcell_filter.input(vecvalue_vloadcell[i]);
//   }
//
//   // Filter acquired values
//   returnval_int[ar_vref] = vecvalue_vref[99];//int(filter(vecvalue_vref)+0.5);
//   returnval_int[ar_vpot] = vecvalue_vpot[99];//int(filter(vecvalue_vpot)+0.5);
//   returnval_int[ar_vim] = vecvalue_vim[99];//int(filter(vecvalue_vim)+0.5);
//   returnval_int[ar_potref] = vecvalue_potref[99];//int(filter(vecvalue_potref)+0.5);
//   returnval_int[ar_vloadcell] = vecvalue_vloadcell[99];//int(filter(vecvalue_vloadcell)+0.5);
//
//   returnval_int[ar_vref_mean] = 0;//value for mean of vref
//   returnval_int[ar_vpot_mean] = 0;//value for mean of vpot
//   returnval_int[ar_vim_mean] = 0;//value for mean of vim
//   returnval_int[ar_potref_mean] = 0;//value for mean of potref
//   returnval_int[ar_vloadcell_mean] = 0;//value for mean of load cell
//   returnval_int[ar_xaccel_mean] = 0;
//   returnval_int[ar_yaccel_mean] = 0;
//   returnval_int[ar_zaccel_mean] = 0;
//
//   float aux_val[ar_last];
//   aux_val[ar_vref_mean] =0;
//   aux_val[ar_vpot_mean] =0;
//   aux_val[ar_vim_mean] =0;
//   aux_val[ar_potref_mean] =0;
//   aux_val[ar_vloadcell_mean] =0;
//   aux_val[ar_xaccel_mean] = 0;
//   aux_val[ar_yaccel_mean] = 0;
//   aux_val[ar_zaccel_mean] = 0;
//
//   for(int i=0;i<NUM_READS;i++)
//   {
//     aux_val[ar_vref_mean] +=vecvalue_vref[i];
//     aux_val[ar_vpot_mean] +=vecvalue_vpot[i];
//     aux_val[ar_vim_mean] +=vecvalue_vim[i];
//     aux_val[ar_potref_mean] +=vecvalue_potref[i];
//     aux_val[ar_vloadcell_mean] +=vecvalue_vloadcell[i];
//     aux_val[ar_xaccel_mean] +=vecvalue_xaccell[i];
//     aux_val[ar_yaccel_mean] +=vecvalue_yaccell[i];
//     aux_val[ar_zaccel_mean] +=vecvalue_zaccell[i];
//   }
//
//   returnval_int[ar_vref_mean] =int(aux_val[ar_vref_mean]/NUM_READS+0.5);
//   returnval_int[ar_vpot_mean] =int(aux_val[ar_vpot_mean]/NUM_READS+0.5);
//   returnval_int[ar_vim_mean] =int(aux_val[ar_vim_mean]/NUM_READS+0.5);
//   returnval_int[ar_potref_mean] =int(aux_val[ar_potref_mean]/NUM_READS+0.5);
//   returnval_int[ar_vloadcell_mean] =int(aux_val[ar_vloadcell_mean]/NUM_READS+0.5);
//   returnval_int[ar_xaccel_mean] =int(aux_val[ar_xaccel_mean]/NUM_READS+0.5);
//   returnval_int[ar_yaccel_mean] =int(aux_val[ar_yaccel_mean]/NUM_READS+0.5);
//   returnval_int[ar_zaccel_mean] =int(aux_val[ar_zaccel_mean]/NUM_READS+0.5);
//
//   //return returnval;
//}

/* Filter not used anymore due to delay
float filter(int raw_val[])
{
  // Sorting the array
  for(int i=0; i<(NUM_READS-1); i++)
  {
    for(int j=0; j<(NUM_READS-(i+1)); j++)
    {
      if(raw_val[j] > raw_val[j+1])
      {
        int aux = raw_val[j];
        raw_val[j] = raw_val[j+1];
        raw_val[j+1] = aux;
      }
    }
  }
  // Median of the 20 center values of the array
  float return_filvalue=0;
  int cont=0;
  for(int i=NUM_READS/2-10;i<(NUM_READS/2+10);i++)
  {
    return_filvalue +=raw_val[i];
    cont++;
  }
  return_filvalue=return_filvalue/20;
  return return_filvalue;
}
*/
