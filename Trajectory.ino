
/***************************************************
 * Find the trajectory points between two steps,
 * The trajectory can be linear (ground phase) 
 * or elliptical (flight phase)
 *****************************************************/
void leg_trajectory_for_step(struct leg *L, int s)
{
      //find the current step 
      int c_step = WALKS[CURRENT_WALK][s][L->index];
      
      //find the next step
      int n_step = WALKS[CURRENT_WALK][(s == (STEPS_IN_WALK - 1))? 0:s + 1][L->index];
      
     
      if(CHAT)
      {
        Serial.print("LEG:");
        Serial.print(LegNames[L->index]);
        Serial.print("current step:");
        Serial.print(c_step);
        Serial.print(" next step:");
        Serial.println(n_step);
      }
      
      boolean flight = false;
      if(c_step == 4 && n_step == 1) //we are walking forwards
      {
         //create an elliptical trajectory
         elliptical_trajectory(L->trajectory, GRANULARITY, &L->flight,false);
         flight = true;
      }
      
      if(c_step == 1 && n_step == 4)  //we are walking backwards
      {
         elliptical_trajectory(L->trajectory, GRANULARITY, &L->flight,false);
         reverse_trajectory(L->trajectory);
         flight = true;
      }
      
      if(!flight)
      {
          struct pose P1;
          struct point p1;
          struct point p2;
          struct point w_h = { POSE_WIDTH, POSE_HEIGHT };
          struct point off = { POSE_ORIGIN_X, POSE_ORIGIN_Y };

          P1.width_height = w_h;
          P1.origin_offset = off;
          
          P1.index = c_step;    
          p1 = point_for_pose(&P1);
          P1.index = n_step;
          p2 = point_for_pose(&P1);
        
          struct line l = { p1,p2 };

          linear_trajectory(L->trajectory, GRANULARITY, &l, true);
      }	

}

/******************************************
 * Given a pose structure return a cartesian point
 * A pose can have an index from 0 to 4, where 0 is
 * the swing phase and 1 to four are the support phase
 * The pose contains the relevant geometry to generate the point
 *********************************************/
struct point point_for_pose(struct pose *P)
{
	struct point p;
        p.x = P->origin_offset.x;
        p.y = P->origin_offset.y;

	if(P->index == 0)
	{
		p.y = p.y - P->width_height.y;
	}
        if(P->index == 1)
	{
		p.x = p.x + P->width_height.x;
	}
        if(P->index == 2)
	{
		p.x = p.x + (P->width_height.x / 3);
	}
        if(P->index == 3)
	{
		p.x = p.x - (P->width_height.x / 3);
	}
        if(P->index == 4)
	{
		p.x = p.x - P->width_height.x;
	}

	return p;
}


/*********************************************************
 * generate a linear trajectory plan (steps) between two points
 * with the required granularity (number of points)
 * The last parameter tells if you want the first point in the plan or not
 *********************************************************/
void linear_trajectory(struct point steps[], int granularity, struct line *L, boolean skip_start_point)
{
       // find the slopes/delta
       float delta_x = L->P2.x - L->P1.x;
       float delta_y = L->P2.y - L->P1.y;
       
	      //calculate the distance between the two points
        float distance = sqrt( ((delta_x) * (delta_x)) + ((delta_y) * (delta_y)) );
  
        //divide the line int the required number of points
        //decrease the granularity one step to be able to include the end point
        int skip = (skip_start_point)? 0:1;
        float step_size = distance / (granularity - skip);

        float c_step = (skip_start_point)? step_size:0;
        
        for(int i=0;i < granularity;i++)
        {
              float inc = c_step / distance;
              float x = L->P1.x + (inc * delta_x);
              float y = L->P1.y + (inc * delta_y);
                 
              steps[i].x = (int)x;
              steps[i].y = (int)y;
              c_step+= step_size;
        }
        
}

/*********************************************************
 * Generate an elliptical trajectory plan 
 * with the required granularity for the provided arc struct. 
 * The last parameter tells if you want the first point in the plan or not
 *********************************************************/
void elliptical_trajectory(struct point steps[], int granularity, struct arc *a, boolean skip_start_point)
{
      //divide the angles int the required number of points
      //decrease the granularity one step to be able to include the end point
      int skip = (skip_start_point)? 0:1;
      
      float step_size = (a->end_angle - a->start_angle) / (granularity - skip);
      float c_angle = a->start_angle;
      
      if(skip_start_point) c_angle+= step_size;
      
      for(int i=0;i < granularity;i++)
      {
        float x = a->origin.x + a->radius.x * cos(radians(c_angle));
        float y = a->origin.y + a->radius.y * sin(radians(c_angle));
       
        steps[(granularity - 1) - i].x = (int)x  + POSE_ORIGIN_X;
        steps[(granularity - 1) - i].y = POSE_ORIGIN_Y - (int)y ;
        c_angle+= step_size;
      }
}

void reverse_trajectory(struct point steps[])
{
    struct point t[10];
    for(int i=0;i < GRANULARITY;i++)
    {
      t[GRANULARITY - i -1] = steps[i];
    }
    for(int i=0;i < GRANULARITY;i++)
    {
      steps[i] = t[i];
    }
    
}


