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
    register_vpot[i] = values_int[ar_vpot_mean];
    delay(20);
  }

  do
  {
    servooldg.write(angle);
    delay(200);
    equal_mean=0;
    ascending=0;
    descending=0;
    readSensors(values_int);
    int aux=values_int[ar_vpot_mean];
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
    register_vpot[comparador-1]=values_int[ar_vpot_mean];
  }while(values_int[ar_vpot_mean]!=equal_mean);
  //}while(ascending==comparador || descending==comparador);
  Serial.print(values_int[ar_vpot_mean]);
  Serial.print(',');
  Serial.println(equal_mean);

  for(int i=0;i<comparador;i++)
  {
    readSensors(values_int);
    register_vpot[i] = values_int[ar_vpot_mean];
    delayMicroseconds(10);
  }
  int measure=0;
  for(int i=0;i<comparador;i++)
  {
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
  int aux_lcbit_min=LC_BIT_MIN;
  int aux_lcbit_max=LC_BIT_MAX;
  float aux_lcnewton_min=LC_NEWTON_MIN;
  float aux_lcnewton_max=LC_NEWTON_MAX;
  int values_int[ar_last];
  float aux_lc;

  Serial.println("For calibration of the Load Cell enter: Max Measure (bits), Min Measure (bits), Max Weight (N), Min Weight (N)");
  Serial.println("Enter -1 to exit to the calibration menu");
  while(Serial.available()==0){};

  do{
    if (Serial.available()){
      aux_lcbit_max= Serial.parseInt();
      if(aux_lcbit_max<0){
        break;
      }
      aux_lcbit_min= Serial.parseInt();
      aux_lcnewton_max= Serial.parseFloat();
      aux_lcnewton_min= Serial.parseFloat();
      Serial.print(aux_lcbit_max);
      Serial.print(", ");
      Serial.print(aux_lcbit_min);
      Serial.print(", ");
      Serial.print(aux_lcnewton_max);
      Serial.print(", ");
      Serial.println(aux_lcnewton_min);
      if (Serial.read()=='\n'){}
      }
    readSensors(values_int);
    aux_lc=((values_int[ar_vloadcell_mean]-aux_lcbit_min)*(aux_lcnewton_max-aux_lcnewton_min))/(aux_lcbit_max-aux_lcbit_min)+aux_lcnewton_min;
    Serial.print("Load Cell bit: ");
    Serial.print(values_int[ar_vloadcell_mean]);
    Serial.print(", Load Cell Calculated:");
    Serial.println(aux_lc);
    delay(1000);

    }while(1);
}


void calibrate_pot(int num_measures)
{
  servooldg.write(0);
  delay(200);
  float fract_angle=180/(num_measures-1);
  for(float i=0;i<181;i+=fract_angle)
  {
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

//------FUNCTION for elbow calibration values and constants
void calibrate_elbow_angle()
{
  PWM_value=FULL_OPEN_ELBOW;
  Serial.println(PWM_value);
  servooldg.write(PWM_value);
  delay(20);
  do
  {
    //menu of elbow calibration
    Serial.println("Choose an action for calibration:");
    Serial.println("  1) Free movement of elbow PWM bits, and measures");
    Serial.println("  2) Confirmation to test choosen values for elbow angle movement calibration");
    Serial.println("  3) Send Calibration values to EEPROM");
    Serial.println(" -1) To exit calibration tests");
    int menu_value=0;
    do
    {
      if (Serial.available())
      {
        menu_value= Serial.parseInt();
        Serial.println(menu_value);
        if (Serial.read()=='\n'){}
      }
    }while(menu_value==0);
    if(menu_value<0)
    {
      break;
    }
    switch (menu_value)
    {
      case 1:
        elbow_calibration_menu1();
        break;
      case 2:
        elbow_calibration_menu2();
        break;
      case 3:
        elbow_calibration_set_to_eeprom();
        break;
    }
  }while(1);
}

//intern subfuction of elbow menu
void elbow_calibration_menu1()
{
  int arm_pos;
  int values_int[ar_last];
  Serial.println("Enter the value that is required to move from 0 to 180 (bits). Always take count the position of the motor's piston:");
  Serial.println("Enter -1 to exit to the calibration menu");
  do
  {
    if (Serial.available())
    {
      arm_pos= Serial.parseInt();
      if(arm_pos<0)
      {
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
    readSensors_filteronly();
//    readSensors(values_int);
    Serial.print("Vpot= ");
    Serial.println(vpot_filter.output());
//    Serial.println(values_int[ar_vpot_mean]);
  }while(1);
}

//intern subfuction of elbow menu
void elbow_calibration_menu2()
{
  int aux_angle_max;
  int aux_angle_min;
  int aux_full_open;
  int auxbit_downward_compen;
  int aux_vpot_max=ANGLE_VPOT_MAX;
  int aux_vpot_min=ANGLE_VPOT_MIN;
  int auxvpot_downward_compen;
  int auxbit_compen;
  int auxvpot_compen;
  float aux_angle;
  int values_int[ar_last];
  float angle_read;
  int aux_Traj_angle;
  float x_tensor_read;
  float Traj_x_min;
  float Traj_x_max;
  float vpot_max_function;


  Serial.println("For test enter, full open elbow value(bits), max open elbow (degrees), min open elbow (degrees) and desired angle");
  Serial.println("Enter -1 to exit to the calibration menu");
  do
  {
    if (Serial.available())
    {

      aux_full_open= Serial.parseInt();
      if(aux_full_open<0){
        break;
      }
      auxbit_compen= Serial.parseInt();
      aux_angle_max= Serial.parseInt();
      aux_angle_min=Serial.parseInt();
      aux_vpot_max=Serial.parseInt();
      auxvpot_compen=Serial.parseInt();
      aux_vpot_min=Serial.parseInt();
      aux_angle= Serial.parseInt();

      if (Serial.read()=='\n'){}
      Serial.print(aux_full_open);
      Serial.print(',');
      Serial.print(auxbit_compen);
      Serial.print(',');
      Serial.print(aux_angle_max);
      Serial.print(',');
      Serial.print(aux_angle_min);
      Serial.print(',');
      Serial.print(aux_vpot_max);
      Serial.print(',');
      Serial.print(auxvpot_compen);
      Serial.print(',');
      Serial.print(aux_vpot_min);
      Serial.print(',');
      Serial.println(aux_angle);

      //Calculation of the x tensor values and the PWM value given the data entered by the user
      auxbit_downward_compen=auxbit_compen-aux_full_open;
      auxvpot_downward_compen=auxvpot_compen-aux_vpot_max;
      Traj_x_min=sqrt(pow(DCA,2)+pow(DCF,2)-2*DCA*DCF*cos(aux_angle_min*PI/180));
      Traj_x_max=sqrt(pow(DCA,2)+pow(DCF,2)-2*DCA*DCF*cos(aux_angle_max*PI/180))-Traj_x_min;
      float x_tensor=sqrt(pow(DCA,2)+pow(DCF,2)-2*DCA*DCF*cos(aux_angle*PI/180))-Traj_x_min;

      int full_open_function=aux_full_open;
      vpot_max_function=aux_vpot_max;
      
      readSensors(values_int);
      float xx_tensor_read=((values_int[ar_vpot_mean]-aux_vpot_min)*Traj_x_max)/(aux_vpot_max-aux_vpot_min)+Traj_x_min;
      float actual_angle=acos((pow(DCA,2)+pow(DCF,2)-pow(xx_tensor_read,2))/(2*DCA*DCF))*180/PI;
      if (aux_angle>actual_angle)
      {
        Serial.println("compensated hysteresis");
        full_open_function+=auxbit_downward_compen;
        vpot_max_function+=auxvpot_downward_compen;
        Serial.println(vpot_max_function);
      }
      PWM_value=(x_tensor*full_open_function)/Traj_x_max;
      Serial.print("PWM setted: ");
      Serial.println(PWM_value);

      servooldg.write(PWM_value);
      delay(20);

      Serial.println("Set the maximum opening of the elbow angle in degres for calibration, followed by angle opening desired");
      Serial.println("Enter -1 to exit to the calibration menu");
    }
    delay(100);
//    readSensors(values_int);
    readSensors_filteronly();
    //Calculation for measure the elbow angle

    x_tensor_read=((vpot_filter.output()-aux_vpot_min)*Traj_x_max)/(vpot_max_function-aux_vpot_min)+Traj_x_min;
//    x_tensor_read=((values_int[ar_vpot_mean]-aux_vpot_min)*Traj_x_max)/(vpot_max_function-aux_vpot_min)+Traj_x_min;
    angle_read=acos((pow(DCA,2)+pow(DCF,2)-pow(x_tensor_read,2))/(2*DCA*DCF));
    Serial.print(Traj_angle);
    Serial.print(", ");
    Serial.print(vpot_filter.output());
    Serial.print(", ");
    Serial.print(x_tensor_read);
    Serial.print(" Angle measured: ");
    Serial.println(angle_read*180/PI);
  }while(1);
}

void LS_parameters_finder ()
{
  int values_int[ar_last];
  int equal_mean;
  int register_vpot[comparador];
  
  float angle_rawfilt;
  float loadcell_rawfilt;
  float angle_ant=0;
  cont_high = cont_low=0;

  //initializing the position of the test

//  do{
//    PWM_value=0;
//    servooldg.write(PWM_value);
//    delay(100);    
//    angle_ant=angle_rawfilt;
//    readSensors_filteronly();
//    angle_rawfilt=read_elbow_angle(vpot_filter.output());
//    hysteresis_function(PWM_value);
//  }while(angle_rawfilt!=angle_ant);

  do
  {
    PWM_value=0;
    servooldg.write(PWM_value);
    delay(100);
    hysteresis_function(PWM_value);
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
    register_vpot[comparador-1]=values_int[ar_vpot_mean];
  }while(values_int[ar_vpot_mean]!=equal_mean);

  //initializing the speed and aceleration arrays
  pos_actual=40;
  readSensors_filteronly();
  angular_measures(read_elbow_angle(vpot_filter.output()));
  readSensors_filteronly();
  angular_measures(read_elbow_angle(vpot_filter.output()));


  cont_cycle=0;
  cont_high=cont_low=20;
  do{
    openclosearmsquarewave();
    cont_cycle++;
    cont_frvar++;

    readSensors_filteronly();
    angle_filter.input(read_elbow_angle(vpot_filter.output()));
    angular_measures(angle_filter.output());
    hysteresis_function(PWM_value);
    loadcell_rawfilt=((loadcell_filter.output()-LC_BIT_MIN)*(LC_NEWTON_MAX-LC_NEWTON_MIN))/(LC_BIT_MAX-LC_BIT_MIN)+LC_NEWTON_MIN;
    
    do{
      delayMicroseconds(500);
      t1_time=millis();
      t_time=t1_time-t0_time;
    }while(t_time<const_time);
    
    t0_time=t1_time;
  }while(cont_cycle<41);

  //repose cycle
  cont_cycle=0;
  cont_high=cont_low=20;
  do{
    cont_cycle++;

    readSensors_filteronly();
    angle_filter.input(read_elbow_angle(vpot_filter.output()));
    angular_measures(angle_filter.output());
    hysteresis_function(PWM_value);
    loadcell_rawfilt=((loadcell_filter.output()-LC_BIT_MIN)*(LC_NEWTON_MAX-LC_NEWTON_MIN))/(LC_BIT_MAX-LC_BIT_MIN)+LC_NEWTON_MIN;
    
    do{
      delayMicroseconds(500);
      t1_time=millis();
      t_time=t1_time-t0_time;
    }while(t_time<const_time);
    
    t0_time=t1_time;
  }while(cont_cycle<51);



  //begin the measures to send for the LS procediment
  Serial.println("Ready");
  cont_cycle=0;
  cont_frvar=0;
  t0_time=millis();
  cont_high=0;

  //Runing cycles of measures
  do{
    if (Serial.available())
    {
      cont_high = cont_low = Serial.parseInt();
      if(cont_high<0)
      {
        menu_var=-1;
        break;
      }
      if (Serial.read()=='\n'){}
    }
    if(cont_high>0)
    {
      openclosearmsquarewave();
      cont_cycle++;
      cont_frvar++;

      tfilter=millis();
      readSensors_filteronly();
      angle_filter.input(read_elbow_angle(vpot_filter.output()));
      angular_measures(angle_filter.output());
      hysteresis_function(PWM_value);
      loadcell_rawfilt=((loadcell_filter.output()-LC_BIT_MIN)*(LC_NEWTON_MAX-LC_NEWTON_MIN))/(LC_BIT_MAX-LC_BIT_MIN)+LC_NEWTON_MIN;
      tfilter=millis()-tfilter;

      //Sending information over serial
      Serial.print(PWM_value);
      Serial.print(',');
      Serial.print(loadcell_rawfilt);
      Serial.print(',');
      Serial.print(angle_filter.output()*180/PI);
      Serial.print(',');
      Serial.print(speed_filter.output());
      Serial.print(',');
      Serial.print(accel_filter.output());
      Serial.print(',');
      Serial.print(h1_filter.output());
      Serial.print(',');
      Serial.print(h2_filter.output());
      Serial.print(',');
      Serial.print(t_time);
      Serial.println(',');
      do{
        delayMicroseconds(500);
        t1_time=millis();
        t_time=t1_time-t0_time;
      }while(t_time<const_time);
      t0_time=t1_time;
  }
  else{
      delayMicroseconds(500);
  }
  }while(cont_high>=0);

}

void LS_parameters_saver()
{
  LS_param_array[0]=0;
    Serial.println("Enter the parameters of the theorical model Equation");
    Serial.println("Order: I, F. Friction, F. Weight, H1, H2, Neg. collision limit, Posit. collision limit.");
  do{
    if (Serial.available())
    {
      LS_param_array[0] = Serial.parseFloat();
      if(LS_param_array[0]<0)
      {
        menu_var=-1;
        break;
      }
      LS_param_array[1] = Serial.parseFloat();
      LS_param_array[2] = Serial.parseFloat();
      LS_param_array[3] = Serial.parseFloat();
      LS_param_array[4] = Serial.parseFloat();
      LS_param_array[5] = Serial.parseFloat();
      LS_param_array[6] = Serial.parseFloat();
      if (Serial.read()=='\n'){}
    }
  }while(LS_param_array[0]==0);
  menu_var=-1;
  Serial.print(LS_param_array[0]);
  Serial.print(',');
  Serial.print(LS_param_array[1]);
  Serial.print(',');
  Serial.print(LS_param_array[2]);
  Serial.print(',');
  Serial.print(LS_param_array[3]);
  Serial.print(',');
  Serial.print(LS_param_array[4]);
  Serial.print(',');
  Serial.print(LS_param_array[5]);
  Serial.print(',');
  Serial.println(LS_param_array[6]);

}


void elbow_calibration_set_to_eeprom()
{
  int full_open_elbow   ;
  int full_open_compen  ;
  int max_elbow_angle   ;
  int min_elbow_angle   ;
  int angle_vpot_max    ;
  int angle_vpot_compen ;
  int angle_vpot_min    ;

  //Ask user input to save in EEPROM
  Serial.println("Insert the values you want to save to EEPROM in the following order Or '-1' to exit:");
  Serial.println("full_open_elbow,full_open_compen,max_elbow_angle(in degrees),min_elbow_angle(in degrees),angle_vpot_max,angle_vpot_compen,angle_vpot_min");
  do
  {
    // Treat input
    if (Serial.available())
    {
      full_open_elbow   = Serial.parseInt();
      if(full_open_elbow<0)
      {
        break;
      }
      full_open_compen  = Serial.parseInt();
      max_elbow_angle   = Serial.parseInt();
      min_elbow_angle   = Serial.parseInt();
      angle_vpot_max    = Serial.parseInt();
      angle_vpot_compen = Serial.parseInt();
      angle_vpot_min    = Serial.parseInt();


      if (Serial.read()=='\n'){}
      Serial.println("Are you certain these are the values you want to save in EEPROM? (y/n)");
      Serial.print(full_open_elbow);
      Serial.print(',');
      Serial.print(full_open_compen);
      Serial.print(',');
      Serial.print(max_elbow_angle);
      Serial.print(',');
      Serial.print(min_elbow_angle);
      Serial.print(',');
      Serial.print(angle_vpot_max);
      Serial.print(',');
      Serial.print(angle_vpot_compen);
      Serial.print(',');
      Serial.println(angle_vpot_min);

      while (menu_var!=89 && menu_var!=121 && menu_var!=78 && menu_var!=110)
      {
        menu_var=Serial.read();
      }
      if(menu_var==89 || menu_var==121)
      {
        // Save in EEPROM
        EEPROM.updateInt(0, full_open_elbow);
        EEPROM.updateInt(2, full_open_compen);
        EEPROM.updateInt(4, max_elbow_angle);
        EEPROM.updateInt(6, min_elbow_angle);
        EEPROM.updateInt(8, angle_vpot_max);
        EEPROM.updateInt(10, angle_vpot_compen);
        EEPROM.updateInt(12, angle_vpot_min);

        Serial.println("New Data Saved");
        // Read values saved in EEPROM
        FULL_OPEN_ELBOW   = EEPROM.readInt(0);
        FULL_OPEN_COMPEN  = EEPROM.readInt(2);
        MAX_ELBOW_ANGLE   = EEPROM.readInt(4)*PI/180;
        MIN_ELBOW_ANGLE   = EEPROM.readInt(6)*PI/180;
        ANGLE_VPOT_MAX    = EEPROM.readInt(8);
        ANGLE_VPOT_COMPEN = EEPROM.readInt(10);
        ANGLE_VPOT_MIN    = EEPROM.readInt(12);
        Serial.println("New Data Loaded");
        Serial.println("");
        break;
      }
      if(menu_var==78 || menu_var==110)
      {
        Serial.println("Insert the values you want to save to EEPROM in the following order:");
        Serial.println("full_open_elbow,full_open_compen,max_elbow_angle(in degrees),min_elbow_angle(in degrees),angle_vpot_max,angle_vpot_compen,angle_vpot_min");
        menu_var=0;
        continue;
      }
    }
  }while(1);
  menu_var=0;
}
