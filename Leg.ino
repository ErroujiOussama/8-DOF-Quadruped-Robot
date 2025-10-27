/**
create a default leg structure
**/
struct leg make_leg(int idx)
{
  struct arc a;
  struct point o = { 0,0 };
  //struct point o = { POSE_ORIGIN_X, POSE_ORIGIN_Y };
  a.origin = o;
  
  struct point r = {POSE_WIDTH, POSE_HEIGHT};
  a.radius = r;
  
  a.start_angle = 0;
  a.end_angle = 180;
  
  struct leg lg;
  lg.flight = a;
  lg.index = idx;
  lg.forward = true;

  return lg;
}



void leg_position_for_tick(struct leg *L)
{
   if(CHAT)
   {
     Serial.print("tick position leg:");
     Serial.print(L->index);
     Serial.print(" x:");
     Serial.print(L->trajectory[CURRENT_TICK].x);
     Serial.print(" y:");
     Serial.println(L->trajectory[CURRENT_TICK].y);
   }
   
   int tk = CURRENT_TICK;
   
   L->hipknee = IK(FEMUR_SIZE,TIBIA_SIZE,L->trajectory[tk]);
   
   //mirow the angles for right side
   if(L->index > 1)
   {
      L->hipknee.hip = 180 - L->hipknee.hip;
      L->hipknee.knee = 180 - L->hipknee.knee;
   }
   
   if(CHAT)
   {
      Serial.print("leg:");
      Serial.print(L->index);
      Serial.print(" hip:");
      Serial.print(L->hipknee.hip);
      Serial.print(" knee:");
      Serial.println(L->hipknee.knee);
    }
    pwm.setPWM(servos[L->index][0],0,map(L->hipknee.hip, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(servos[L->index][1],0,map(L->hipknee.knee, 0, 180, SERVOMIN, SERVOMAX));  
}

void set_leg(int idx, int pose_idx)
{
    //legs[idx]
    struct pose P;
    struct point p;
    struct point w_h = { POSE_WIDTH, POSE_HEIGHT };
    struct point off = { POSE_ORIGIN_X, POSE_ORIGIN_Y };
    P.index = pose_idx;
    P.width_height = w_h;
    P.origin_offset = off;
    p = point_for_pose(&P);
    
    legs[idx].hipknee = IK(FEMUR_SIZE,TIBIA_SIZE,p);  
    
    boolean mirrow = false;
    //check if right - mirrow the angles
    if(idx > 1)
    {
      mirrow = true;
      legs[idx].hipknee.hip = 180 - legs[idx].hipknee.hip;
      legs[idx].hipknee.knee = 180 - legs[idx].hipknee.knee;
    }
    
    if(CHAT)
    {
      Serial.print("leg:");
      Serial.print(idx);
      Serial.print(" mirrow:");
      Serial.print(mirrow);
      Serial.print(" step:");
      Serial.print(pose_idx);
      Serial.print(" hip:");
      Serial.print(legs[idx].hipknee.hip);
      Serial.print(" knee:");
      Serial.println(legs[idx].hipknee.knee);
    }
    
    pwm.setPWM(servos[idx][0],0,map(legs[idx].hipknee.hip, 0, 180, SERVOMIN, SERVOMAX));
    pwm.setPWM(servos[idx][1],0,map(legs[idx].hipknee.knee, 0, 180, SERVOMIN, SERVOMAX));
}

/**********************************************************
 * Inverse Kinematic function for a two link planar system.
 * Given the size of the two links an a desired position, 
 * it returns the angles for both links
 **********************************************************/
struct angles IK(int L1, int L2, struct point p)
{
    struct angles a;

    int L1P2 = L1 * L1;
    int L2P2 = L2 * L2;
    int x2 = p.x * p.x;
    int y2 = p.y * p.y;

    //calculates the distance beetween the first link and the endpoint
    float d = sqrt( x2 + y2 );
    float c = min( d, L1 + L2 );
    float c2 = c * c;

    //Find wich cartesian quadrant the solution should be in
    float D = atan2(p.y,p.x);

    //calculates the angle between the distance segment and the first link
    float B = acos( (L1P2 - L2P2 + c2) / ( -2 * L1 * c) );

    //Add the quadrant information and convert from radias to angles 
    a.hip = (int)((D + B) * RAD_TO_DEG );
    
    if(a.hip > 180) a.hip -= 180;

    //calculate the angle between the first and second link
    float C = acos( (c2 - L1P2 - L2P2) / (-2 * L1 * L2) );
    
    //the knee angle is relative to the first link;
    a.knee = (int)( C * RAD_TO_DEG);
    
    //relate the angle to the x axis (horizon) and flip it to match the servo position
    a.knee = 180 - ( a.knee - a.hip );

    return a;
}



