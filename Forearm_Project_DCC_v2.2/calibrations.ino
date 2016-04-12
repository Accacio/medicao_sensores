//----------- Functions Related to Calibration of Sensors and Actuators---------------------

//-------- Calibrate LoadCell-----------
void calibrate_loadcell()
{
  int aux_lcbit_min=LC_BIT_MIN;
  int aux_lcbit_max=LC_BIT_MAX;
  float aux_lcnewton_min=LC_NEWTON_MIN;
  float aux_lcnewton_max=LC_NEWTON_MAX;
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

    readSensors_byfilters();
    aux_lc=((loadcell_filter.output()-aux_lcbit_min)*(aux_lcnewton_max-aux_lcnewton_min))/(aux_lcbit_max-aux_lcbit_min)+aux_lcnewton_min;
    Serial.print("Load Cell bit: ");
    Serial.print(loadcell_filter.output());

    Serial.print(", Load Cell Calculated:");
    Serial.println(aux_lc);
    delay(1000);

    }while(1);
}

//--------- calibrate Potentiometer----------
void calibrate_pot()
{
  int num_measures=5;
  float fraction=180/(num_measures-1);
  float pospot=0;
  int adder_array[COMPARADOR];
  int adder;
  Serial.println("****Notice!!!*****");
  Serial.println("Disconect the cable of the Motor which links to the Exoesqueleton to begin tests and any obstacle in the Motor Piston trajectory.");
  do{
      Serial.println("Is the cable-link disconected? (Y/N):");
      while (menu_var!=89 && menu_var!=121 && menu_var!=78 && menu_var!=110)
      {
        menu_var=Serial.read();
      };
  }while(menu_var!=89 && menu_var!=121);

  Serial.println("Begining monitoring between full contracted and full extended");
  servooldg.write(0);
  delay(200);

  for(int i=0;i<num_measures;i++)
  {
    servooldg.write(pospot);
    readSensors_byfilters();

    for(int j=0;j<COMPARADOR;j++)
    {
      adder_array[j]=int(vpot_filter.output()+0.5);
    }

    do
    {
    delay(500);
    adder=adder_array[0];
    for(int j=1;j<COMPARADOR;j++)
    {
      adder_array[j-1]=adder_array[j];
      adder+=adder_array[j];
    }
    adder=int(adder/COMPARADOR+0.5);
    readSensors_byfilters();
    adder_array[COMPARADOR-1]=int(vpot_filter.output()+0.5);
    Serial.println(adder);
    Serial.println(adder_array[COMPARADOR-1]);
    }while( adder_array[COMPARADOR-1]!=adder);

    Serial.print("Measure of position ");
    Serial.print(100*pospot/180);
    Serial.print(" is: ");
    Serial.println(vpot_filter.output());
    pospot=pospot+fraction;
  }
  Serial.println("End of monitoring Motor potentiometer position");
}

//---- Bank of functions to calibrate elbow definitions
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
    Serial.println("  4) Show values saved on EEPROM");
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
      case 4:
        show_calibration_eeprom_values();
        break;
    }
  }while(1);
}

//intern subfuction of elbow menu
void elbow_calibration_menu1()
{
  int arm_pos;
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
    readSensors_byfilters();
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
//**changes  int values_int[ar_last];
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
      if(aux_full_open<0)
      {
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

//**change      readSensors(values_int);
//**change      float xx_tensor_read=((values_int[ar_vpot_mean]-aux_vpot_min)*Traj_x_max)/(aux_vpot_max-aux_vpot_min)+Traj_x_min;
//**change      float actual_angle=acos((pow(DCA,2)+pow(DCF,2)-pow(xx_tensor_read,2))/(2*DCA*DCF))*180/PI;

      readSensors_byfilters();
      float xx_tensor_read=((vpot_filter.output()-aux_vpot_min)*Traj_x_max)/(aux_vpot_max-aux_vpot_min)+Traj_x_min;
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
    readSensors_byfilters();
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
      Serial.println("Are you certain these are the values you want to save in EEPROM? (Y/N)");
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
        EEPROM.updateInt(2, full_open_compen-full_open_elbow);
        EEPROM.updateInt(4, max_elbow_angle);
        EEPROM.updateInt(6, min_elbow_angle);
        EEPROM.updateInt(8, angle_vpot_max);
        EEPROM.updateInt(10, angle_vpot_compen-angle_vpot_max);
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

void show_calibration_eeprom_values()
{
  Serial.println("Current Values for elbow calibration saved in EEPROM are:");
  Serial.print(FULL_OPEN_ELBOW);
  Serial.print(',');
  Serial.print(FULL_OPEN_COMPEN);
  Serial.print(',');
  Serial.print(MAX_ELBOW_ANGLE);
  Serial.print(',');
  Serial.print(MIN_ELBOW_ANGLE);
  Serial.print(',');
  Serial.print(ANGLE_VPOT_MAX);
  Serial.print(',');
  Serial.print(ANGLE_VPOT_COMPEN);
  Serial.print(',');
  Serial.println(ANGLE_VPOT_MIN);
}

void LS_parameters_finder ()
{
  int equal_mean;
  int register_vpot[COMPARADOR];
  int aux_pos=0;

  float angle_rawfilt;
  float angle_ant=0;
  cont_high = cont_low=0;

  //initializing the position of the test
  readSensors_byfilters();
  for(int i=0;i<COMPARADOR;i++)
  {
    register_vpot[i]=int(vpot_filter.output()+0.5);
  }

  do
  {
    /*change PWM_value=FULL_CLOSE_ELBOW;
    servooldg.write(PWM_value);
    delay(200);
    */
    pos_actual=MIN_ELBOW_ANGLE*180/PI;
    set_elbow_angle(pos_actual*PI/180);
    hysteresis_function(PWM_value);
    delay(500);
    equal_mean=0;
    for(int i=0;i<COMPARADOR;i++)
    {
      equal_mean+=register_vpot[i];
    }
    equal_mean=int(equal_mean/COMPARADOR+0.5);

    for(int i=1;i<COMPARADOR;i++)
    {
      register_vpot[i-1]=register_vpot[i];
    }
    readSensors_byfilters();
    register_vpot[COMPARADOR-1]=int(vpot_filter.output()+0.5);
  }while(register_vpot[COMPARADOR-1]!=equal_mean);

  //initializing the speed and aceleration arrays

  //*chage pos_actual=40;
  /*change readSensors_byfilters();
  angular_measures(read_elbow_angle(vpot_filter.output()));
  readSensors_byfilters();
  angular_measures(read_elbow_angle(vpot_filter.output()));
  */
  updatePosition();
  updatePosition();


  cont_cycle=0;
  cont_high=cont_low=20;
  do
  {
    opencloseelbowcycle();
    cont_cycle++;
/*change    readSensors_byfilters();
    angle_filter.input(read_elbow_angle(vpot_filter.output()));
    angular_measures(angle_filter.output());
    hysteresis_function(PWM_value);
    loadcell_value=((loadcell_filter.output()-LC_BIT_MIN)*(LC_NEWTON_MAX-LC_NEWTON_MIN))/(LC_BIT_MAX-LC_BIT_MIN)+LC_NEWTON_MIN;
    */
    updatePosition();
    do
    {
      delayMicroseconds(500);
      t1_time=millis();
      t_time=t1_time-t0_time;
    }while(t_time<const_time);

    t0_time=t1_time;
  }while(cont_cycle<41);

  //repose cycle
  cont_cycle=0;
  cont_high=cont_low=20;
  do
  {
    cont_cycle++;
/*change    readSensors_byfilters();
    angle_filter.input(read_elbow_angle(vpot_filter.output()));
    angular_measures(angle_filter.output());
    hysteresis_function(PWM_value);
    loadcell_value=((loadcell_filter.output()-LC_BIT_MIN)*(LC_NEWTON_MAX-LC_NEWTON_MIN))/(LC_BIT_MAX-LC_BIT_MIN)+LC_NEWTON_MIN;
    */
    updatePosition();
    do
    {
      delayMicroseconds(500);
      t1_time=millis();
      t_time=t1_time-t0_time;
    }while(t_time<const_time);

    t0_time=t1_time;
  }while(cont_cycle<41);

  //begin the measures to send for the LS procediment
  Serial.println("Ready");
  cont_cycle=0;
  cont_high=cont_low=0;
  t0_time=millis();


  //Runing cycles of measures
  do
  {
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
      opencloseelbowcycle();
      cont_cycle++;

//*change      tfilter=millis();
/*change      readSensors_byfilters();
      angle_filter.input(read_elbow_angle(vpot_filter.output()));
      angular_measures(angle_filter.output());
      hysteresis_function(PWM_value);
      */

      // functions just to count the time spend in each interaction
      Theorical_Model_fun(angle_filter.output());
      T_theor=T_theorical_filter.output();
      loadcell_value=((loadcell_filter.output()-LC_BIT_MIN)*(LC_NEWTON_MAX-LC_NEWTON_MIN))/(LC_BIT_MAX-LC_BIT_MIN)+LC_NEWTON_MIN;
      DCC_v2(aux_pos,loadcell_value,T_theor);
      aux_pos=force_tolerance(pos_actual,loadcell_value,T_theor);
      t_time=millis()-t0_time;
      t0_time=millis();
      //Sending information over serial
      Serial.print(PWM_value);
      Serial.print(',');
      Serial.print(loadcell_value);
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
/*change      do
      {
        delayMicroseconds(500);
        t1_time=millis();
        t_time=t1_time-t0_time;
      }while(t_time<const_time);
      t0_time=t1_time;
      */
    }
    else
    {
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


void LS_parameters_set_to_eeprom()
{
  LS_param_array[0] =   0;
  float lsparam0;
  float lsparam1;
  float lsparam2;
  float lsparam3;
  float lsparam4;
  float lsparam5;
  float lsparam6;
  //Show Data in EEPROM

  LS_param_array[0]  = EEPROM.readFloat(14);
  LS_param_array[1]  = EEPROM.readFloat(18);
  LS_param_array[2]  = EEPROM.readFloat(22);
  LS_param_array[3]  = EEPROM.readFloat(26);
  LS_param_array[4]  = EEPROM.readFloat(30);
  LS_param_array[5]  = EEPROM.readFloat(34);
  LS_param_array[6]  = EEPROM.readFloat(38);

  Serial.println("Data in EEPROM is:");
  Serial.print(LS_param_array[0],4);
  Serial.print(',');
  Serial.print(LS_param_array[1],4);
  Serial.print(',');
  Serial.print(LS_param_array[2],4);
  Serial.print(',');
  Serial.print(LS_param_array[3],4);
  Serial.print(',');
  Serial.print(LS_param_array[4],4);
  Serial.print(',');
  Serial.print(LS_param_array[5],4);
  Serial.print(',');
  Serial.println(LS_param_array[6],4);
  Serial.println("");


  //Ask user input to save in EEPROM
  Serial.println("Enter the parameters of the theorical model Equation or write -1 to exit");
  Serial.println("Order: I, F. Friction, F. Weight, H1, H2, Neg. collision limit, Posit. collision limit.");
  do
  {
    // Treat input
    if (Serial.available())
    {
      lsparam0 = Serial.parseFloat();
      if(lsparam0==-1)
      {
        break;
      }
      lsparam1 = Serial.parseFloat();
      lsparam2 = Serial.parseFloat();
      lsparam3 = Serial.parseFloat();
      lsparam4 = Serial.parseFloat();
      lsparam5 = Serial.parseFloat();
      lsparam6 = Serial.parseFloat();

      if (Serial.read()=='\n'){}
      Serial.println("Are you certain these are the values you want to save in EEPROM? (Y/N)");
      Serial.print(lsparam0,4);
      Serial.print(',');
      Serial.print(lsparam1,4);
      Serial.print(',');
      Serial.print(lsparam2,4);
      Serial.print(',');
      Serial.print(lsparam3,4);
      Serial.print(',');
      Serial.print(lsparam4,4);
      Serial.print(',');
      Serial.print(lsparam5,4);
      Serial.print(',');
      Serial.println(lsparam6,4);

      while (menu_var!=89 && menu_var!=121 && menu_var!=78 && menu_var!=110)
      {
        menu_var=Serial.read();
      }

      if(menu_var==89 || menu_var==121)
      {
        // Save in EEPROM
        EEPROM.updateFloat(14, lsparam0);
        EEPROM.updateFloat(18, lsparam1);
        EEPROM.updateFloat(22, lsparam2);
        EEPROM.updateFloat(26, lsparam3);
        EEPROM.updateFloat(30, lsparam4);
        EEPROM.updateFloat(34, lsparam5);
        EEPROM.updateFloat(38, lsparam6);

        Serial.println("New Data Saved");
        // Read values saved in EEPROM
        LS_param_array[0]  = EEPROM.readFloat(14);
        LS_param_array[1]  = EEPROM.readFloat(18);
        LS_param_array[2]  = EEPROM.readFloat(22);
        LS_param_array[3]  = EEPROM.readFloat(26);
        LS_param_array[4]  = EEPROM.readFloat(30);
        LS_param_array[5]  = EEPROM.readFloat(34);
        LS_param_array[6]  = EEPROM.readFloat(38);
        Serial.println("New Data Loaded");
        Serial.println("");
        break;
      }

      if(menu_var==78 || menu_var==110)
      {
        Serial.println("Enter the parameters of the theorical model Equation or write -1 to exit");
        Serial.println("Order: I, F. Friction, F. Weight, H1, H2, Neg. collision limit, Posit. collision limit.");
        menu_var=-1;
        continue;
      }
    }
  }while(1);
  menu_var=-1;
  Serial.print(LS_param_array[0],4);
  Serial.print(',');
  Serial.print(LS_param_array[1],4);
  Serial.print(',');
  Serial.print(LS_param_array[2],4);
  Serial.print(',');
  Serial.print(LS_param_array[3],4);
  Serial.print(',');
  Serial.print(LS_param_array[4],4);
  Serial.print(',');
  Serial.print(LS_param_array[5],4);
  Serial.print(',');
  Serial.println(LS_param_array[6],4);

}
