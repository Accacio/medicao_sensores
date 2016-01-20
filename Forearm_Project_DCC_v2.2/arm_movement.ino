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
  Serial.println("**Entering in the Elbow Movement Function, without control. Free movement**");
  Serial.println("Notice: In this mode collisions are no took in acount, if exist an obstacle the user can be injured, ");
  Serial.println("or the mechanical part can be broken.");
  Serial.println("");

/*change readSensors_byfilters();
  angle_filter.input(read_elbow_angle(vpot_filter.output()));
  angular_measures(angle_filter.output());
  readSensors_byfilters();
  angle_filter.input(read_elbow_angle(vpot_filter.output()));
  angular_measures(angle_filter.output());
  */

//Initialization of the angle, speed and acceleration over the angular measures function
  updatePosition();
  pos_actual=int(angle_filter.output()*180/PI+0.5);;
  set_elbow_angle(pos_actual*PI/180);
  updatePosition();

  Serial.println("->Enter the aperture of the elbow angle desired: ");
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

//*change    readSensors_byfilters();
//*change    angle_filter.input(read_elbow_angle(vpot_filter.output()));
//*change    angular_measures(angle_filter.output());
//*change    hysteresis_function(PWM_value);
    updatePosition();
    Theorical_Model_fun(angle_filter.output());
    T_theor=T_theorical_filter.output();
    loadcell_value=((loadcell_filter.output()-LC_BIT_MIN)*(LC_NEWTON_MAX-LC_NEWTON_MIN))/(LC_BIT_MAX-LC_BIT_MIN)+LC_NEWTON_MIN;
    Controlled_elbow_angle=0;
    set_elbow_angle(pos_actual*PI/180);
    force_outbound_flag=force_tolerance(pos_actual,loadcell_value,T_theor);

    print_Movementinfo();
/*change    Serial.print(pos_actual);
    Serial.print(",");
    Serial.print(PWM_value);
    Serial.print(",");
    Serial.print(loadcell_value);
    Serial.print(",");
    Serial.print(angle_filter.output()*180/PI);
    Serial.print(",");
//    Serial.print(T_theor);
//    Serial.print(",");
    Serial.print(loadcell_value-T_theor);
    Serial.print(",");
    Serial.print(force_outbound_flag);
    Serial.print(",");
    Serial.println(Controlled_elbow_angle*180/PI);
    */
  }while(1);
}


//---- Function Elbow Movement with Continuos Control-------------------
void elbow_continuos_control()
{
  Serial.println("**Entering in the Elbow Movement Function, with Damping control by collision**");
  Serial.println("");
  Serial.println("(Tolerance Deviation is the tolerance of disturbance took as normal by the DCC control, higher the value");
  Serial.println(" more force is required to stop the movement, and lower the value more suceptible to mechanical noise)");
  Serial.println("->Enter the tolerance deviation (percentage increment) value recomended 20: ");
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

  /*change readSensors_byfilters();
  angle_filter.input(read_elbow_angle(vpot_filter.output()));
  angular_measures(angle_filter.output());
  readSensors_byfilters();
  angle_filter.input(read_elbow_angle(vpot_filter.output()));
  angular_measures(angle_filter.output());
  */
  updatePosition();
  pos_actual=int(angle_filter.output()*180/PI+0.5);;
  set_elbow_angle(pos_actual*PI/180);
  updatePosition();

  Serial.println("->Enter the aperture of the elbow angle desired: ");
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

//*change    readSensors_byfilters();
//*change    angle_filter.input(read_elbow_angle(vpot_filter.output()));
//*change    angular_measures(angle_filter.output());
//*change    hysteresis_function(PWM_value);
    updatePosition();
    Theorical_Model_fun(angle_filter.output());
    T_theor=T_theorical_filter.output();
    loadcell_value=((loadcell_filter.output()-LC_BIT_MIN)*(LC_NEWTON_MAX-LC_NEWTON_MIN))/(LC_BIT_MAX-LC_BIT_MIN)+LC_NEWTON_MIN;

    DCC_v2(pos_actual,loadcell_value,T_theor);
    Controlled_elbow_angle=DCCv2_angle_filter.output()*PI/180;
    set_elbow_angle(Controlled_elbow_angle);
    force_outbound_flag=force_tolerance(pos_actual,loadcell_value,T_theor);

    print_Movementinfo();
/*change    Serial.print(pos_actual);
    Serial.print(",");
    Serial.print(PWM_value);
    Serial.print(",");
    Serial.print(loadcell_value);
    Serial.print(",");
    Serial.print(angle_filter.output()*180/PI);
    Serial.print(",");
//    Serial.print(T_theor);
//    Serial.print(",");
    Serial.print(loadcell_value-T_theor);
    Serial.print(",");
    Serial.print(force_outbound_flag);
    Serial.print(",");
    Serial.println(Controlled_elbow_angle*180/PI);
    */
  }while(1);

}


//---- Function Elbow Movement with Stop Control-------------------
void elbow_control_stop()
{
  Serial.println("**Entering in the Elbow Movement Function, with stop control by collision**");
  Serial.println("");
  Serial.println("(Tolerance Deviation is the tolerance of disturbance took as normal by the DCC control, higher the value");
  Serial.println(" more force is required to stop the movement, and lower the value more suceptible to mechanical noise)");
  Serial.println("->Enter the tolerance deviation (percentage increment): ");
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

/*change  readSensors_byfilters();
  angle_filter.input(read_elbow_angle(vpot_filter.output()));
  angular_measures(angle_filter.output());
  readSensors_byfilters();
  angle_filter.input(read_elbow_angle(vpot_filter.output()));
  angular_measures(angle_filter.output());
  */
  updatePosition();
  pos_actual=int(angle_filter.output()*180/PI+0.5);;
  set_elbow_angle(pos_actual*PI/180);
  updatePosition();

  Serial.println("->Enter the aperture of the elbow angle desired ");
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

/*change    readSensors_byfilters();
    angle_filter.input(read_elbow_angle(vpot_filter.output()));
    angular_measures(angle_filter.output());
    hysteresis_function(PWM_value);
    */
    updatePosition();
    Theorical_Model_fun(angle_filter.output());
    T_theor=T_theorical_filter.output();
    loadcell_value=((loadcell_filter.output()-LC_BIT_MIN)*(LC_NEWTON_MAX-LC_NEWTON_MIN))/(LC_BIT_MAX-LC_BIT_MIN)+LC_NEWTON_MIN;

    DCC_v1(pos_actual,loadcell_value,T_theor);
    Controlled_elbow_angle=DCCv1_angle_filter.output()*PI/180;
    set_elbow_angle(Controlled_elbow_angle);
    force_outbound_flag=force_tolerance(pos_actual,loadcell_value,T_theor);

    if(force_outbound_flag==0)
    {
      elbow_angle_required=Controlled_elbow_angle*180/PI;
      Serial.println("Obstacle over the trajectory detected, the movement stops");
    }
    print_Movementinfo();
/*change    Serial.print(pos_actual);
    Serial.print(",");
    Serial.print(PWM_value);
    Serial.print(",");
    Serial.print(loadcell_value);
    Serial.print(",");
    Serial.print(angle_filter.output()*180/PI);
    Serial.print(",");
//    Serial.print(T_theor);
//    Serial.print(",");
    Serial.print(loadcell_value-T_theor);
    Serial.print(",");
    Serial.print(force_outbound_flag);
    Serial.print(",");
    Serial.print(elbow_angle_required);
    Serial.print(",");
    Serial.println(Controlled_elbow_angle*180/PI);
    */
  }while(1);
}


//---- Function Print Movement Information to monitoring the behavior or the algorithm
void print_Movementinfo()
{
  Serial.print("Ang. Set: ");
  Serial.print(pos_actual);
//*change  Serial.print(PWM_value);
//*change  Serial.print(",");
  Serial.print(", Ang. Msr: ");
  Serial.print(angle_filter.output()*180/PI);
  Serial.print(", L.C. Msr: ");
  Serial.print(loadcell_value);
  Serial.print(", Tens. Calc: ");
  Serial.print(T_theor);
  Serial.print(", Tens. Error: ");
  Serial.print(loadcell_value-T_theor);
  Serial.print(", Force Outbound Flag: ");
  Serial.print(force_outbound_flag);
  Serial.print(", Ang. Controlled: ");
  Serial.println(Controlled_elbow_angle*180/PI);
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
