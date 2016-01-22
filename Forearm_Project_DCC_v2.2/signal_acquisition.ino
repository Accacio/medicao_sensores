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
