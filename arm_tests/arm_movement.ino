//----------- Functions related to the elbow---------------------
//----------Elbow movement----------------
void arm_movement()
{
  int elbow_angle;
  int values_int[ar_last];
  //  PWM_value=FULL_OPEN;
  Serial.print("Define the aperture of the elbow angle (degres) between ");
  Serial.print(MAX_ELBOW_ANGLE*180/PI);
  Serial.print(" and ");
  Serial.println(MIN_ELBOW_ANGLE*180/PI);

  do
  {
    if (Serial.available())
    {
      elbow_angle= Serial.parseInt();
      Serial.println(elbow_angle);
      if(elbow_angle<0)
      {
        break;
      }
      if (Serial.read()=='\n')
      {
        flag=1;
      }
      Serial.println(elbow_angle);
      set_elbow_angle(elbow_angle);
      Serial.print("Define the aperture of the elbow angle (degres) between ");
      Serial.print(MAX_ELBOW_ANGLE*180/PI);
      Serial.print(" and ");
      Serial.println(MIN_ELBOW_ANGLE*180/PI);
    }
    readSensors(values_int);
    float angle_calculated=read_elbow_angle(values_int[ar_vpot]);
    Serial.print(angle_calculated*180/PI);
    Serial.print(",");
    Serial.print(values_int[ar_vpot]);
    Serial.print(",");
    Serial.println(((values_int[ar_vloadcell_mean]-LC_BIT_MIN)*(LC_NEWTON_MAX-LC_NEWTON_MIN))/(LC_BIT_MAX-LC_BIT_MIN)+LC_NEWTON_MIN);
  }while(1);
}


//-------------FUNCTION to set the position of the elbow given the angle
void set_elbow_angle(float angle_set)
{
  float aux_angle=angle_set*PI/180;
  if(aux_angle>=MIN_ELBOW_ANGLE && aux_angle<=MAX_ELBOW_ANGLE)
  {
  float x_tensor=sqrt(pow(DCA,2)+pow(DCF,2)-2*DCA*DCF*cos(aux_angle))-Traj_x_min;
  PWM_value=(x_tensor*FULL_OPEN_ELBOW)/Traj_x_max;
  servooldg.write(PWM_value);
  delay(20);
  }
  else
  {
    Serial.print("Elbow angle out of limits, please enter again a value between: ");
    Serial.print(MAX_ELBOW_ANGLE*180/PI);
    Serial.print(" and ");
    Serial.println(MIN_ELBOW_ANGLE*180/PI);
  }
}

//-------------FUNCTION to read the elbow angle given the vpot (position of the piston of the motor)
float read_elbow_angle(int Pot_value)
{
  float x_tensor=(Pot_value-ANGLE_VPOT_MIN)*Traj_x_max/Traj_angle+Traj_x_min;
  float angle_elbow=acos((pow(DCA,2)+pow(DCF,2)-pow(x_tensor,2))/(2*DCA*DCF));
  return angle_elbow;
}

void angular_measures (float results[3])
{
  unsigned long old_time=0;
  unsigned long new_time=0;
  unsigned long var_time;
  float old_angle=0;
  float new_angle=0;
  float old_aspeed=0;
  float new_aspeed=0;
  float old_aaccel=0;
  float new_aaccel=0;
  int values_int[ar_last];

  int num_measures=3;

  for(int i=0;i<num_measures;i++)
  {
    readSensors(values_int);
    old_time=new_time;
    new_time=millis();
    var_time=new_time-old_time;
    old_angle=new_angle;
    new_angle=read_elbow_angle(values_int[ar_vpot_mean]); //in rads
    old_aspeed=new_aspeed;
    new_aspeed=(new_angle-old_angle)/var_time;            // in rad/??
    old_aaccel=new_aaccel;
    new_aaccel=(new_aspeed-old_aspeed)/var_time;

    Serial.print("Time: ");
    Serial.print(var_time);
    Serial.print(", Angle: ");
    Serial.print(new_angle);
    Serial.print(", Ang. Speed: ");
    Serial.print(new_aspeed);
    Serial.print(", Ang. Accel.: ");
    Serial.println(new_aaccel);
  }
  results[1]=new_angle;
  results[2]=new_aspeed;
  results[3]=new_aaccel;
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
    angular_measures(elbow_angulars);
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
