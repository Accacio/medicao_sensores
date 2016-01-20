//----------- Functions Related to the Elbow---------------------


//---- Function Elbow Movement----------------
void elbow_menu_movement(){
  do{
    //menu of elbow movement
    Serial.println("Choose an action for Forearm movement:");
    Serial.println("  1) Trejectory without control");
    Serial.println("  2) Trajectory with continuos control");
    Serial.println("  3) Trajectory with control and collision stop");
    Serial.println(" -1) To exit Forearm tests");
    Serial.println("");
    Serial.println("*******Remember always!!. calibrate the arm  before run tests*****:");
    int menu_value=0;
    do{
      if (Serial.available()){
        menu_value= Serial.parseInt();
        Serial.println(menu_value);
        if (Serial.read()=='\n'){}
      }
    }while(menu_value==0);
    if(menu_value<0){
      menu_var=-1;
      break;
    }

    switch (menu_value)
    {
      case 1:
        elbow_freemovement();
        break;
      case 2:
        elbow_continuos_control();
        break;
      case 3:
        elbow_control_stop();
        break;
    }
  }while(1);
  }


//---- Function Elbow Movement without DCC control -------------------
void elbow_freemovement()
{
  // Local Variables
  float elbow_angle_required=FULL_OPEN_ELBOW/2;
  float angle_rawfilt;
  float loadcell_rawfilt;
  float Tension_measured;
  float Controlled_elbow_angle;
  float T_theor;
  float angle_ref;
  int angle_step=1;
  int force_outbound_flag=1;

//Initialization of the angle, speed and acceleration over the angular measures function
  readSensors_filteronly();
  angle_filter.input(read_elbow_angle(vpot_filter.output()));
  angular_measures(angle_filter.output());
  readSensors_filteronly();
  angle_filter.input(read_elbow_angle(vpot_filter.output()));
  angular_measures(angle_filter.output());

  pos_actual=int(angle_filter.output()*180/PI+0.5);;
  set_elbow_angle(pos_actual*PI/180);

  Serial.println("Enter the aperture of the elbow angle desired");
  do{
    if (Serial.available())
    {
      elbow_angle_required= Serial.parseInt();
      if(elbow_angle_required<0)
      {
        menu_var=-1;
        break;
      }
      Serial.println(elbow_angle_required);

      if (Serial.read()=='\n'){}
    }

    if(pos_actual<elbow_angle_required){
      pos_actual=pos_actual+force_outbound_flag*angle_step;
    }
    else{
      if(pos_actual>elbow_angle_required){
        pos_actual=pos_actual-force_outbound_flag*angle_step;
      }
    }

    readSensors_filteronly();
    angle_filter.input(read_elbow_angle(vpot_filter.output()));
    angular_measures(angle_filter.output());
    hysteresis_function(PWM_value);
    Theorical_Model_fun(angle_filter.output());
    T_theor=T_theorical_filter.output();
    loadcell_rawfilt=((loadcell_filter.output()-LC_BIT_MIN)*(LC_NEWTON_MAX-LC_NEWTON_MIN))/(LC_BIT_MAX-LC_BIT_MIN)+LC_NEWTON_MIN;

//*change    DCC_v1(pos_actual,loadcell_rawfilt,T_theor);
//*change    Controlled_elbow_angle=DCCv1_angle_filter.output()*PI/180;
    set_elbow_angle(pos_actual*PI/180);
    force_outbound_flag=force_tolerance(pos_actual,loadcell_rawfilt,T_theor);

    Serial.print(pos_actual);
    Serial.print(",");
    Serial.print(PWM_value);
    Serial.print(",");
    Serial.print(loadcell_rawfilt);
    Serial.print(",");
    Serial.print(angle_filter.output()*180/PI);
    Serial.print(",");
//    Serial.print(T_theor);
//    Serial.print(",");
    Serial.print(loadcell_rawfilt-T_theor);
    Serial.print(",");
    Serial.print(force_outbound_flag);
    Serial.print(",");
    Serial.println(Controlled_elbow_angle*180/PI);
  }while(1);

}


//---- Function Elbow Movement with Continuos Control-------------------
void elbow_continuos_control()
{
  float elbow_angle_required=FULL_OPEN_ELBOW/2;
  float angle_rawfilt;
  float loadcell_rawfilt;
  float Tension_measured;
  float Controlled_elbow_angle;
  float T_theor;
  float angle_ref;
  int angle_step=1;
  int force_outbound_flag=1;

  Serial.println("Enter the tolerance deviation (percentage increment) value recomended 20:");
  Serial.println("Tolerance Deviation is the tolerance of disturbance took as normal by the DCC control");
  Serial.println("higher the value more force is required to stop the movement, and lower the value more suceptible to mechanical noise");
  tolerance=0;
  do
  {
    if (Serial.available()){
      tolerance= Serial.parseInt();
      if(tolerance<0){
        menu_var=-1;
        break;
      }
      if (Serial.read()=='\n'){}
      if(tolerance>1){
        tolerance=tolerance/100;
      }
      Serial.println(tolerance);
    }
  }while(tolerance==0);

  readSensors_filteronly();
  angle_filter.input(read_elbow_angle(vpot_filter.output()));
  angular_measures(angle_filter.output());
  readSensors_filteronly();
  angle_filter.input(read_elbow_angle(vpot_filter.output()));
  angular_measures(angle_filter.output());

  pos_actual=int(angle_filter.output()*180/PI+0.5);;
  set_elbow_angle(pos_actual*PI/180);

  Serial.println("Enter the aperture of the elbow angle desired");
  do
  {
    if (Serial.available())
    {

      elbow_angle_required= Serial.parseInt();
      if(elbow_angle_required<0)
      {
        menu_var=-1;
        break;
      }
      Serial.println(elbow_angle_required);

      if (Serial.read()=='\n'){}
    }

    if(pos_actual<elbow_angle_required)
    {
      pos_actual=pos_actual+force_outbound_flag*angle_step;
    }
    else
    {
      if(pos_actual>elbow_angle_required)
      {
        pos_actual=pos_actual-force_outbound_flag*angle_step;
      }
    }

    readSensors_filteronly();
    angle_filter.input(read_elbow_angle(vpot_filter.output()));
    angular_measures(angle_filter.output());

    hysteresis_function(PWM_value);
    Theorical_Model_fun(angle_filter.output());
    T_theor=T_theorical_filter.output();
    loadcell_rawfilt=((loadcell_filter.output()-LC_BIT_MIN)*(LC_NEWTON_MAX-LC_NEWTON_MIN))/(LC_BIT_MAX-LC_BIT_MIN)+LC_NEWTON_MIN;

    DCC_v2(pos_actual,loadcell_rawfilt,T_theor);
    Controlled_elbow_angle=DCCv2_angle_filter.output()*PI/180;
    set_elbow_angle(Controlled_elbow_angle);

    force_outbound_flag=force_tolerance(pos_actual,loadcell_rawfilt,T_theor);

    Serial.print(pos_actual);
    Serial.print(",");
    Serial.print(PWM_value);
    Serial.print(",");
    Serial.print(loadcell_rawfilt);
    Serial.print(",");
    Serial.print(angle_filter.output()*180/PI);
    Serial.print(",");
//    Serial.print(T_theor);
//    Serial.print(",");
    Serial.print(loadcell_rawfilt-T_theor);
    Serial.print(",");
    Serial.print(force_outbound_flag);
    Serial.print(",");
    Serial.println(Controlled_elbow_angle*180/PI);

  }while(1);

}


//---- Function Elbow Movement with Stop Control-------------------
void elbow_control_stop()
{
  Serial.println("Entering in the Elbow Movement Function, with stop control by collision");
  float elbow_angle_required=FULL_OPEN_ELBOW/2;
  float angle_rawfilt;
  float loadcell_rawfilt;
  float Tension_measured;
  float Controlled_elbow_angle;
  float T_theor;
  float angle_ref;
  int angle_step=1;
  int force_outbound_flag=1;

  Serial.println("Enter the tolerance deviation (percentage increment)");
  tolerance=0;
  do
  {
    if (Serial.available()){
      tolerance= Serial.parseInt();
      if(tolerance<0){
        menu_var=-1;
        break;
      }
      if (Serial.read()=='\n'){}
      if(tolerance>1){
        tolerance=tolerance/100;
      }
      Serial.println(tolerance);
    }
  }while(tolerance==0);

  tolerance=tolerance+1;
  readSensors_filteronly();
  angle_filter.input(read_elbow_angle(vpot_filter.output()));
  angular_measures(angle_filter.output());
  readSensors_filteronly();
  angle_filter.input(read_elbow_angle(vpot_filter.output()));
  angular_measures(angle_filter.output());
  pos_actual=int(angle_filter.output()*180/PI+0.5);;
  set_elbow_angle(pos_actual*PI/180);

  Serial.println("Enter the aperture of the elbow angle desired");
 // Serial.println("Ready");
  do
  {
    if (Serial.available())
    {
      elbow_angle_required= Serial.parseInt();
      if(elbow_angle_required<0)
      {
        menu_var=-1;
        break;
      }
      Serial.println(elbow_angle_required);
      if (Serial.read()=='\n'){}
    }

    if(pos_actual<elbow_angle_required)
    {
      pos_actual=pos_actual+force_outbound_flag*angle_step;
    }
    else
    {
      if(pos_actual>elbow_angle_required)
      {
        pos_actual=pos_actual-force_outbound_flag*angle_step;
      }
    }

    readSensors_filteronly();
    angle_filter.input(read_elbow_angle(vpot_filter.output()));
    angular_measures(angle_filter.output());
    hysteresis_function(PWM_value);
    Theorical_Model_fun(angle_filter.output());
    T_theor=T_theorical_filter.output();
    loadcell_rawfilt=((loadcell_filter.output()-LC_BIT_MIN)*(LC_NEWTON_MAX-LC_NEWTON_MIN))/(LC_BIT_MAX-LC_BIT_MIN)+LC_NEWTON_MIN;

    DCC_v1(pos_actual,loadcell_rawfilt,T_theor);
    Controlled_elbow_angle=DCCv1_angle_filter.output()*PI/180;
    set_elbow_angle(Controlled_elbow_angle);
    force_outbound_flag=force_tolerance(pos_actual,loadcell_rawfilt,T_theor);

    if(force_outbound_flag==0)
    {
      elbow_angle_required=Controlled_elbow_angle*180/PI;
      Serial.println("Obstacle over the trajectory detected, the movement stops");
    }

    Serial.print(pos_actual);
    Serial.print(",");
    Serial.print(PWM_value);
    Serial.print(",");
    Serial.print(loadcell_rawfilt);
    Serial.print(",");
    Serial.print(angle_filter.output()*180/PI);
    Serial.print(",");
//    Serial.print(T_theor);
//    Serial.print(",");
    Serial.print(loadcell_rawfilt-T_theor);
    Serial.print(",");
    Serial.print(force_outbound_flag);
    Serial.print(",");
    Serial.print(elbow_angle_required);
    Serial.print(",");
    Serial.println(Controlled_elbow_angle*180/PI);
  }while(1);

}


//---- Function Set the Elbow Angle to set the position of the elbow given the angle
void set_elbow_angle(float angle_set)
{
  float full_open_function=FULL_OPEN_ELBOW;     //value without compensation
  float x_tensor;
//*change  int values_int[ar_last];
//*change  float aux_angle=angle_set;

  if(angle_set>=MIN_ELBOW_ANGLE && angle_set<=MAX_ELBOW_ANGLE)
  {
  Traj_angle=ANGLE_VPOT_MAX-ANGLE_VPOT_MIN;     //without compensation for the hysteresys in the vpot measure
  if (h1_filter.output()>0)
  {
    full_open_function+=FULL_OPEN_COMPEN;
    Traj_angle+=ANGLE_VPOT_COMPEN;              //compensation for the hysteresys in the vpot measure
  }

  x_tensor=sqrt(pow(DCA,2)+pow(DCF,2)-2*DCA*DCF*cos(angle_set))-Traj_x_min;
  PWM_value=(x_tensor*full_open_function)/Traj_x_max;
  servooldg.write(PWM_value);
  delay(5);
  }
  else
  {
    Serial.println("Error, elbow angle required out of limits");
  }
}


//---- Function Read the angle over the Elbow to read the elbow angle given the vpot (position of the piston of the motor)
float read_elbow_angle(int Pot_value)
{
  float x_tensor=(Pot_value-ANGLE_VPOT_MIN)*Traj_x_max/Traj_angle+Traj_x_min;
  float angle_elbow=acos((pow(DCA,2)+pow(DCF,2)-pow(x_tensor,2))/(2*DCA*DCF));
  return angle_elbow;
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


//---- Function Deterministic Model Calculation
void Theorical_Model_fun(float angle)
{
    //Definitions
    float factor_I=LS_param_array[0];
    float factor_Friction=LS_param_array[1];
    float factor_Weight=LS_param_array[2];
    float factor_h1=LS_param_array[3];
    float factor_h2=LS_param_array[4];

    //angular calculations
    float x_tensor=sqrt(pow(DCA,2)+pow(DCF,2)-2*DCA*DCF*cos(angle));
    float Beta=asin((DCF*sin(angle))/x_tensor);

    //forces calculation
    float tension_I=factor_I*accel_filter.output();
    float tension_Friction=factor_Friction*G*sin(angle)*(Lf+Lh)*DCMF*speed_filter.output();
    float tension_Force=factor_Weight*G*sin(angle)*(Lf+Lh)*DCMF;
    float h1=factor_h1*h1_filter.output();
    float h2=factor_h2*h2_filter.output();

    // Theorical tension calculation
    float result=(tension_I+tension_Friction+tension_Force+h1+h2)/(DCF*sin(Beta));
    T_theorical_filter.input(result);
}


//---- Function Damping Control by Colision function first version
void DCC_v1(float teta_ref, float Tension_measure, float T_theorical)
{
  //definitions
  float et=Tension_measure-T_theorical;
  Sgm_left_lim=LS_param_array[5]*(1+tolerance);
  Sgm_right_lim=LS_param_array[6]*(1+tolerance);

  //function calculations
  float sgm_low=1-1/(1+exp(-((et+4)/Sgm_slope2-Sgm_left_lim)));
  float sgm_up=1/(1+exp(-((et-4)/Sgm_slope2-Sgm_right_lim)));
  float u_aux=teta_ref-30*sgm_low+30*sgm_up;
  float u=max(MIN_ELBOW_ANGLE*180/PI,min(MAX_ELBOW_ANGLE*180/PI,u_aux));
  DCCv1_angle_filter.input(u);
}


//---- Function Damping Control by Colision function second version
void DCC_v2(float teta_ref, float Tension_measure, float T_theorical)
{
  //definitions
  float et=Tension_measure-T_theorical;
  Sgm_left_lim=LS_param_array[5]*(1+tolerance);
  Sgm_right_lim=LS_param_array[6]*(1+tolerance);

  //function calculations
  float sgm_low=1-1/(1+exp(-((et+4)/Sgm_slope-Sgm_left_lim)));
  float sgm_up=1/(1+exp(-((et-4)/Sgm_slope-Sgm_right_lim)));
  float u=teta_ref+(MIN_ELBOW_ANGLE*180/PI-teta_ref)*sgm_low+(MAX_ELBOW_ANGLE*180/PI-teta_ref)*sgm_up;
  DCCv2_angle_filter.input(u);
}


//---- Function Force Tolerance to calculate the tolereance over the control function
int force_tolerance (float teta_ref, float Tension_measure, float T_theorical)
{
  bool flag_alarm=1;
  float et=Tension_measure-T_theorical;
  digitalWrite(51,HIGH);
  Sgm_left_lim=LS_param_array[5]*(1+tolerance);
  Sgm_right_lim=LS_param_array[6]*(1+tolerance);
  if(et<Sgm_left_lim || et>Sgm_right_lim){
    flag_alarm=0;
    digitalWrite(51,LOW);
  }

  return flag_alarm;
}
