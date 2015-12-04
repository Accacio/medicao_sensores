//----------- Functions related to the elbow---------------------
//----------Elbow movement----------------
void arm_movement()
{
  float elbow_angle_required=FULL_OPEN_ELBOW/2;
  float angle_rawfilt;
  float loadcell_rawfilt;
  //int values_int[ar_last];
  float Tension_measured;
  float Controlled_elbow_angle;
  float T_theor;
  //  PWM_value=FULL_OPEN;
  //Serial.print("Define the aperture of the elbow angle (degres) between ");
  //Serial.print(MAX_ELBOW_ANGLE*180/PI);
  //Serial.print(" and ");
  //Serial.println(MIN_ELBOW_ANGLE*180/PI);

  do
  {
    if (Serial.available())
    {

      elbow_angle_required= Serial.parseInt()*PI/180;
      Serial.println(elbow_angle_required*180/PI);
      if(elbow_angle_required<0)
      {
        menu_var=-1;
        break;
      }
      if (Serial.read()=='\n'){}
      //Serial.println(elbow_angle);
      set_elbow_angle(elbow_angle_required);
    }

    //readSensors(values_int);
//    Tension_measured=((values_int[ar_vloadcell_mean]-LC_BIT_MIN)*(LC_NEWTON_MAX-LC_NEWTON_MIN))/(LC_BIT_MAX-LC_BIT_MIN)+LC_NEWTON_MIN;
//    Controlled_elbow_angle=collision_control(elbow_angle,Tension_measured);
//    set_elbow_angle(Controlled_elbow_angle);

    readSensors_filteronly();   
    angle_rawfilt=read_elbow_angle(vpot_filter.output());
    angular_measures(angle_rawfilt);
    hysteresis_function(PWM_value);
    T_theor=Theorical_model(angle_rawfilt);
    loadcell_rawfilt=((loadcell_filter.output()-LC_BIT_MIN)*(LC_NEWTON_MAX-LC_NEWTON_MIN))/(LC_BIT_MAX-LC_BIT_MIN)+LC_NEWTON_MIN;
    Controlled_elbow_angle=collision_control(elbow_angle_required,loadcell_rawfilt,T_theor);
    set_elbow_angle(Controlled_elbow_angle);

//    Serial.print(loadcell_rawfilt);
//    Serial.print(",");
//    Serial.print(T_theor);
//    Serial.print(",");
//    Serial.print(loadcell_rawfilt-T_theor);  
//    Serial.print(",");
//    Serial.println(Controlled_elbow_angle);    
//    float angle_calculated=read_elbow_angle(values_int[ar_vpot_mean]);
  }while(1);

}


//-------------FUNCTION to set the position of the elbow given the angle
void set_elbow_angle(float angle_set)
{
  float aux_angle=angle_set;
  float full_open_function=FULL_OPEN_ELBOW;
  int values_int[ar_last];
  float actual_angle;
  float x_tensor;
  
  if(aux_angle>=MIN_ELBOW_ANGLE && aux_angle<=MAX_ELBOW_ANGLE)
  {
  Traj_angle=ANGLE_VPOT_MAX-ANGLE_VPOT_MIN;     //without compensation for the hysteresys in the vpot measure
  //readSensors(values_int);
  //actual_angle=read_elbow_angle(values_int[ar_vpot_mean]);
//    readSensors_filteronly();
//    actual_angle=read_elbow_angle(vpot_filter.output());
    
  if (h1_filter.output()>0)
      {
        full_open_function+=FULL_OPEN_COMPEN;
        Traj_angle+=ANGLE_VPOT_COMPEN;          //compensation for the hysteresys in the vpot measure
      }

//    full_open_function=FULL_OPEN_ELBOW+FULL_OPEN_COMPEN*h1_filter.output();
//    Traj_angle=Traj_angle+ANGLE_VPOT_COMPEN*h1_filter.output();
//  if (aux_angle>actual_angle)
//      {
//        //Serial.println("compensated hysteresis");
//        full_open_function+=FULL_OPEN_COMPEN;
//        Traj_angle+=ANGLE_VPOT_COMPEN;          //compensation for the hysteresys in the vpot measure
//      }
  //Serial.print(actual_angle);
  //Serial.print(", ");
  //Serial.print(aux_angle);
  //Serial.print(", ");
  //Serial.println(full_open_function);

  x_tensor=sqrt(pow(DCA,2)+pow(DCF,2)-2*DCA*DCF*cos(aux_angle))-Traj_x_min;
  PWM_value=(x_tensor*full_open_function)/Traj_x_max;
  servooldg.write(PWM_value);
  //delay(20);
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


//----------Hysteresis Function construction
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


//------------Deterministic model construction
float Theorical_model(float angle)
{
    //Definitions
    float I_fact=LS_param_array[0];
    float Frict_fact=LS_param_array[1];
    float W_fact=LS_param_array[2];
    float h1_fact=LS_param_array[3];
    float h2_fact=LS_param_array[4]; 
    
    //angular calculations
    float x_tensor=sqrt(pow(DCA,2)+pow(DCF,2)-2*DCA*DCF*cos(angle));
    float Beta=asin((DCF*sin(angle))/x_tensor);
   
    //forces calculation
    float Ti=I_fact*accel_filter.output();
    float Tfric=Frict_fact*G*sin(angle)*(Lf+Lh)*DCMF*speed_filter.output();
    float Tf=W_fact*G*sin(angle)*(Lf+Lh)*DCMF;
    float h1=h1_fact*h1_filter.output();
    float h2=h2_fact*h2_filter.output();

    
//    Serial.print(sin(angle));
//    Serial.print(',');
//    Serial.print(Tf/(DCF*sin(Beta)));
//    Serial.print(',');
//    Serial.print(angle);
//    Serial.print(',');
//    Serial.print(Beta);
//    Serial.print(',');
//    
//    Serial.print(Ti);
//    Serial.print(',');
//    Serial.print(Tfric);
//    Serial.print(',');
//    Serial.print(Tf);
//    Serial.print(',');
//    Serial.print(h1);
//    Serial.print(',');
//    Serial.println(h2);
//

    
    // Theorical tension calculation
    float result=(Ti+Tfric+Tf+h1+h2)/(DCF*sin(Beta));
    T_theorical_filter.input(result);
    return result;
}


//---- Damping control by colision function

float collision_control(float teta_ref, float Tension_measure, float T_theorical)
{
//  float Tension_theoric=0;  //Place where a function returns the value of the theoric tension

  //definitions
  float et=Tension_measure-T_theorical;
  Sgm_left_lim=LS_param_array[5];
  Sgm_right_lim=LS_param_array[6];
  
  //function calculations
  float sgm_low=1-1/(1+exp(-((et+4)/Sgm_slope-Sgm_left_lim)));
  float sgm_up=1/(1+exp(-((et-4)/Sgm_slope-Sgm_right_lim)));
  float u=teta_ref+(MIN_ELBOW_ANGLE-teta_ref)*sgm_low+(MAX_ELBOW_ANGLE-teta_ref)*sgm_up;
//    Serial.print(exp(et+3-Sgm_left_lim));
//    Serial.print("|");
//    Serial.print(teta_ref);
//    Serial.print("|");
//    
//    Serial.print(et);
//    Serial.print("|");
//    Serial.print(sgm_low*1000);
//    Serial.print("|");
//    Serial.print(sgm_up*1000);
//    Serial.print("|");
//    Serial.print((MIN_ELBOW_ANGLE-teta_ref)*sgm_low);
//    Serial.print("|");
//    Serial.print((MAX_ELBOW_ANGLE-teta_ref)*sgm_up);
//    Serial.print("|");
//    Serial.println(u);
//    Serial.print("|");

return u;  
}

