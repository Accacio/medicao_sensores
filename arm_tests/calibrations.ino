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

  do
  {
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
    Serial.println("  3) Under construction");
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
    readSensors(values_int);
    Serial.print("Vpot= ");
    Serial.println(values_int[ar_vpot]);
  }while(1);
}

//intern subfuction of elbow menu
void elbow_calibration_menu2()
{
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
  do
  {
    if (Serial.available())
    {
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
