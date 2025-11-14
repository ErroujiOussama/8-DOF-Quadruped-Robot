
/***************************************************************
 *  Credits:
 *  Oussama Errouji
 *  Developed and adapted this code during the NRC Competition,
 *  2025 edition.
 *
 *  File Purpose:
 *  Generates and manages leg movement trajectories (linear and elliptical)
 *  for coordinated walking and gait cycles. Critical for synchronized motion.
 *
 *  Please cite this contribution if reused or reworked.
 ***************************************************************/

 /***************************************************
 * Find the trajectory points between two steps,
 * The trajectory can be linear (ground phase) 
 * or elliptical (flight phase)
 *****************************************************/
// --------------------------------------------------------------
// leg_trajectory_for_step:
// Computes the sequence of trajectory points for a single robot leg between two walk steps.
// Based on the current and next step, it chooses between ground phase (linear) and flight phase (elliptical) paths.
//  - Linear path is used for legs on the ground (stance phase).
//  - Elliptical path (with optional reversal) models the leg 'flying' through the air (swing phase), allowing smooth lifting and placement.
// This trajectory is crucial for coordinated quadruped walking in robotics, allowing alternating support and swing phases.
// --------------------------------------------------------------
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
 * point_for_pose:
 * Maps a discrete pose structure (with width, height, origin, and phase index) to coordinates in space.
 * The pose index represents specific gait phases:
 *   0: swing; 1-4: stance/support.
 * This mapping underpins trajectory generation, translating gait logic to motion.
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
 * linear_trajectory:
 * Generates a set of evenly spaced points between two coordinates, forming the leg's path during the stance (ground) phase.
 * Uses basic linear interpolation in Cartesian space.
 * This method is fundamental in robotics for achieving predictable, smooth, ground-phase movement.
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
 * elliptical_trajectory:
 * Generates a set of points along an elliptical arc, used for the flight phase of leg movement (leg off the ground),
 * providing smooth vertical lift and forward progression.
 * This models natural animal gaits and avoids abrupt touchdown/launch, enabling stable and realistic walking.
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

/*********************************************************
 * reverse_trajectory:
 * Simple utility function that reverses the order of waypoints in a trajectory array.
 * Useful for generating backward motion without recalculating a new trajectory.
 *********************************************************/
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


