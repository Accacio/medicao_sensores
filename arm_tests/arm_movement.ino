//----------- Functions related to the elbow---------------------
//----------Elbow movement----------------
void arm_movement()
{
  float elbow_angle=FULL_OPEN_ELBOW;
  int values_int[ar_last];
  float Tension_measured;
  float Controlled_elbow_angle;
  //  PWM_value=FULL_OPEN;
  //Serial.print("Define the aperture of the elbow angle (degres) between ");
  //Serial.print(MAX_ELBOW_ANGLE*180/PI);
  //Serial.print(" and ");
  //Serial.println(MIN_ELBOW_ANGLE*180/PI);

  do
  {
    if (Serial.available())
    {
      elbow_angle= Serial.parseInt();
      elbow_angle=elbow_angle*PI/180;
      //Serial.println(elbow_angle);
      if(elbow_angle<0)
      {
        break;
      }
      if (Serial.read()=='\n'){}
      //Serial.println(elbow_angle);
      
      set_elbow_angle(elbow_angle);
      //Serial.print("Define the aperture of the elbow angle (degres) between ");
      //Serial.print(MAX_ELBOW_ANGLE*180/PI);
      //Serial.print(" and ");
      //Serial.println(MIN_ELBOW_ANGLE*180/PI);
    }

    readSensors(values_int);
    Tension_measured=((values_int[ar_vloadcell_mean]-LC_BIT_MIN)*(LC_NEWTON_MAX-LC_NEWTON_MIN))/(LC_BIT_MAX-LC_BIT_MIN)+LC_NEWTON_MIN;
    Controlled_elbow_angle=collision_control(elbow_angle,Tension_measured);
    set_elbow_angle(Controlled_elbow_angle);
    float angle_calculated=read_elbow_angle(values_int[ar_vpot_mean]);
    Serial.print(Controlled_elbow_angle);
    Serial.print(",");
    Serial.print(angle_calculated*180/PI);
    Serial.print(",");
    Serial.print(Tension_measured);
    Serial.println(",");
  }while(1);
}


//-------------FUNCTION to set the position of the elbow given the angle
void set_elbow_angle(float angle_set)
{
  float aux_angle=angle_set;
  float full_open_function=FULL_OPEN_ELBOW;
  int values_int[ar_last];
  if(aux_angle>=MIN_ELBOW_ANGLE && aux_angle<=MAX_ELBOW_ANGLE)
  {
  Traj_angle=ANGLE_VPOT_MAX-ANGLE_VPOT_MIN;     //without compensation for the hysteresys in the vpot measure
  readSensors(values_int);
  float actual_angle=read_elbow_angle(values_int[ar_vpot_mean]);

  if (aux_angle>actual_angle)
      {
        //Serial.println("compensated hysteresis");
        full_open_function+=FULL_OPEN_COMPEN;
        Traj_angle+=ANGLE_VPOT_COMPEN;          //compensation for the hysteresys in the vpot measure
      }
  //Serial.print(actual_angle);
  //Serial.print(", ");
  //Serial.print(aux_angle);
  //Serial.print(", ");
  //Serial.println(full_open_function);

  float x_tensor=sqrt(pow(DCA,2)+pow(DCF,2)-2*DCA*DCF*cos(aux_angle))-Traj_x_min;
  PWM_value=(x_tensor*full_open_function)/Traj_x_max;
  servooldg.write(PWM_value);
  delay(20);
  }
  else
  {
    //Serial.print("Elbow angle out of limits, please enter again a value between: ");
    //Serial.print(MAX_ELBOW_ANGLE*180/PI);
    //Serial.print(" and ");
    //Serial.println(MIN_ELBOW_ANGLE*180/PI);
  }
}

//-------------FUNCTION to read the elbow angle given the vpot (position of the piston of the motor)
float read_elbow_angle(int Pot_value)
{
  float x_tensor=(Pot_value-ANGLE_VPOT_MIN)*Traj_x_max/Traj_angle+Traj_x_min;
  float angle_elbow=acos((pow(DCA,2)+pow(DCF,2)-pow(x_tensor,2))/(2*DCA*DCF));
  return angle_elbow;
}

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
  
//    Serial.print("Time: ");
//    Serial.print(var_time);
//      Serial.print(", Angle: ");
//      Serial.print(angle_array[0]);
//      Serial.print(", Ang. Speed: ");
//      Serial.print(speed_array[0]);
//      Serial.print(", Ang. Accel.: ");
//      Serial.println(accel_array[0]);
  
}

void hysteresis_function(float Pwm_value)
{
    for (int i=0;i<2;i++)
  {
    Pwm_array[2-i]=Pwm_array[1-i];
    h1_array[2-i]=h1_array[1-i];
    h2_array[2-i]=h2_array[1-i];
  }
  Pwm_array[0]=Pwm_value;
  if (Pwm_array[0]>0)
  {
    Hyst_cont_h1++;
    Hyst_cont_h2=0;
    if(Hyst_cont_h1>Hyst_cont_down)
    {
      h1_array[0]=0;
      h2_array[0]=1;

    }
    else
    {
      h1_array[0]=1;
      h2_array[0]=0;      
    } 
  }
  else
  {
    Hyst_cont_h1=0;
    Hyst_cont_h2++;
    if(Hyst_cont_h2>Hyst_cont_up)
    {
      h1_array[0]=1;
      h2_array[0]=0;
    }
    else
    {
      h1_array[0]=0;
      h2_array[0]=1;      
    } 
  }
}
void deterministic_model()
{
  float elbow_angulars[3];
  float FW=PFW*Subject_weight;
  float FCM=PFCM*(Lf+Lh);
  float I=FW*pow(FCM,2);
  float Tf;
  float Td=0;
  float Taccel;
  float Tcf;
  float x_tensor;
  float teta_ac;
  while(1)
  {
    
    //angular_measures();
    x_tensor=sqrt(pow(DCA,2)+pow(DCF,2)-2*DCA*DCF*cos(elbow_angulars[1]));
    teta_ac=asin((DCF*sin(elbow_angulars[1]))/x_tensor);
    Tf=FW*FCM*G*sin(elbow_angulars[1]);
    Taccel=I*elbow_angulars[3];
    Tcf=(Taccel+Tf+Td)/(DCA*sin(teta_ac));
    Serial.print(" Trq. forearm: ");
    Serial.print(Tf);
    Serial.print(", Trq. mov: ");
    Serial.print(Taccel);
    Serial.print(", Trq. CF: ");
    Serial.println(Tcf);
  }
}


//-----------------------
// Damping control by colision

float collision_control(float teta_ref, float Tension_measure){
float Tension_theoric=28;  //Place where a function returns the value of the theoric tension
float et=Tension_measure-Tension_theoric;
float sgm_low=1-1/(1+exp(-(et+3-Sgm_left_lim)));
float sgm_up=1/(1+exp(-(et-3-Sgm_right_lim)));
float u=teta_ref+(MIN_ELBOW_ANGLE-teta_ref)*sgm_low+(MAX_ELBOW_ANGLE-teta_ref)*sgm_up;
//    Serial.print(exp(et+3-Sgm_left_lim));
//    Serial.print("|");
//    Serial.print(teta_ref);
//    Serial.print("|");
//    
//    Serial.print(et);
//    Serial.print("|");
//    Serial.print((MIN_ELBOW_ANGLE-teta_ref)*sgm_low);
//    Serial.print("|");
//    Serial.print((MAX_ELBOW_ANGLE-teta_ref)*sgm_up);
//    Serial.print("|");
//    Serial.print(u);
//    Serial.print("|");

return u;  
}

