void readSensors_filteronly()
{
  // Acquire sensor values
  for(int i=0;i<NUM_READS_FILTER;i++)
  {
    delayMicroseconds(10);
    vpot_filter.input(analogRead(VPOT_IN));
    loadcell_filter.input(analogRead(LOADCELL_IN));
  }
}

void readSensors(int returnval_int[5])
{
   // read multiple values of three sensors at same time and sort them to take the mode


   int vecvalue_vpot[NUM_READS];
   int vecvalue_vloadcell[NUM_READS];

   // Acquire sensor values
   for(int i=0;i<NUM_READS;i++)
   {
     delayMicroseconds(10);
     vecvalue_vpot[i] = analogRead(VPOT_IN);
     vecvalue_vloadcell[i] = analogRead(LOADCELL_IN);
     vpot_filter.input(vecvalue_vpot[i]);
     loadcell_filter.input(vecvalue_vloadcell[i]);
   }

   // Filter acquired values

   returnval_int[ar_vpot] = vecvalue_vpot[99];//int(filter(vecvalue_vpot)+0.5);
   returnval_int[ar_vloadcell] = vecvalue_vloadcell[99];//int(filter(vecvalue_vloadcell)+0.5);


   returnval_int[ar_vpot_mean] = 0;//value for mean of vpot
   returnval_int[ar_vloadcell_mean] = 0;//value for mean of load cell

   float aux_val[ar_last];
   aux_val[ar_vpot_mean] =0;
   aux_val[ar_vloadcell_mean] =0;

   for(int i=0;i<NUM_READS;i++)
   {
     aux_val[ar_vpot_mean] +=vecvalue_vpot[i];
     aux_val[ar_vloadcell_mean] +=vecvalue_vloadcell[i];
   }


   returnval_int[ar_vpot_mean] =int(aux_val[ar_vpot_mean]/NUM_READS+0.5);
   returnval_int[ar_vloadcell_mean] =int(aux_val[ar_vloadcell_mean]/NUM_READS+0.5);

   //return returnval;
}

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
