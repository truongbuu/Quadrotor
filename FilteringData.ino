const int num_readings = 10;
int ptr_f = 0;
int ptr_v = 0;
float fx[10] = {0,0,0,0,0,0,0,0,0,0};
float fy[10] = {0,0,0,0,0,0,0,0,0,0};
float gyx[10] = {0,0,0,0,0,0,0,0,0,0};
float gyy[10] = {0,0,0,0,0,0,0,0,0,0};

float ve_x[10] = {0,0,0,0,0,0,0,0,0,0};
float ve_y[10] = {0,0,0,0,0,0,0,0,0,0};
float mx[10]   = {0,0,0,0,0,0,0,0,0,0};
float my[10]   = {0,0,0,0,0,0,0,0,0,0};
void filter_velo(){
  ve_x[ptr_v] = velo_x;
  ve_y[ptr_v] = velo_y;

  mx[ptr_v] = move_x;
  my[ptr_v] = move_y;
  
  ptr_v++;

  if(ptr_v >= 10){
    ptr_v = 0 ;
  }
  int k =0;
  for(k = 0; k < 10; k++){
    velo_x += ve_x[k];
    velo_y += ve_y[k];
   // move_x += mx[k];
   // move_y += my[k];
  }

  velo_x = velo_x/10;
  velo_y = velo_y/10;
  
 // move_x = move_x/10;
 // move_y = move_y/10;
  }

void filter_flow(){
    fx[ptr_f] = flow_x;
    fy[ptr_f] = flow_y;
    gyx[ptr_f] = x_rate;
    gyy[ptr_f] = y_rate;
    
    ptr_f++;
    
    if(ptr_f >= num_readings){
      ptr_f = 0;   
    }
    
    int c = 0;
    
    for(c = 0; c< num_readings; c++){
      flow_x += fx[c];
      flow_y += fy[c];
      x_rate += gyx[c];
      y_rate += gyy[c]; 
    }
    flow_x = flow_x/10;
    flow_y = flow_y/10;
    x_rate = x_rate/10;
    y_rate = y_rate/10;
  }
