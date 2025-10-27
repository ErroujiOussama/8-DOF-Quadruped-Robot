void test_point_for_pose()
{
  struct pose P;
  struct point p;
  struct point w_h = { POSE_WIDTH, POSE_HEIGHT };
  struct point off = { POSE_ORIGIN_X, POSE_ORIGIN_Y };
  P.width_height = w_h;
  P.origin_offset = off;
  
  for(int i=0;i < 5;i++)
  {
    P.index = i;    
    p = point_for_pose(&P);
    
    struct angles a = IK(FEMUR_SIZE,TIBIA_SIZE,p);  
    
    Serial.print("index:");
    Serial.print(i);
    Serial.print(" x:");
    Serial.print(p.x);
    Serial.print(" y:");
    Serial.print(p.y);
    Serial.print(" hip:");
    Serial.print(a.hip);
    Serial.print(" knee:");
    Serial.println(a.knee);
  }
}

void test_ik()
{
    struct point p1;
    p1.x = 0; 
    p1.y = -100;
  
    struct angles a = IK(80,65,p1);  
    
    p1.x = 0;
    p1.y = -70;
    a = IK(80,65,p1);  
  
    p1.x = 40;
    p1.y = -100;
    a = IK(80,65,p1);  
  
    p1.x = -40;
    p1.y = -100;
    a = IK(80,65,p1); 
}

void test_linear_trajectory()
{
   struct point S[10] = {};
   struct point P1 = {0,0};
   struct point P2 = {100,100};
   struct line l = { P1,P2 };
  
   int granularity = 10;
   linear_trajectory(S, granularity, &l, true);
   for(int i=0;i < granularity;i++)
   {
      Serial.print("step:");
      Serial.print(i);
      Serial.print(" x:");
      Serial.print(S[i].x);
      Serial.print(" y:");
      Serial.println(S[i].y);
   }
}

void test_elliptical_trajectory()
{
    struct point S[10] = {};
    struct point P1 = {0,0};
    struct point P2 = {100,100};
    struct line l = { P1,P2 };
    
    int granularity = 10;
    struct arc a;
    a.origin.x = 0;
    a.origin.y = 0; 
    a.radius.x = 40;
    a.radius.y = 40; 
    a.start_angle = 0;
    a.end_angle = 180;
    
    elliptical_trajectory(S,granularity, &a, false);
   
    for(int i=0;i < granularity;i++)
    {
      Serial.print("step:");
      Serial.print(i);
      Serial.print(" x:");
      Serial.print(S[i].x);
      Serial.print(" y:");
      Serial.println(S[i].y);
    }
}

void test_reverse_trajectory()
{
  
    struct point S[10] = {};
    struct point P1 = {0,0};
    struct point P2 = {100,100};
    struct line l = { P1,P2 };
    
    int granularity = 4;
    struct arc a;
    a.origin.x = 0;
    a.origin.y = 0; 
    a.radius.x = 40;
    a.radius.y = 40; 
    a.start_angle = 0;
    a.end_angle = 180;
    
    elliptical_trajectory(S,granularity, &a, false);
   
    Serial.println("********************");
    for(int i=0;i < granularity;i++)
    {
      Serial.print("step:");
      Serial.print(i);
      Serial.print(" x:");
      Serial.print(S[i].x);
      Serial.print(" y:");
      Serial.println(S[i].y);
    }
    
    reverse_trajectory(S);
    
    Serial.println("********************");
    for(int i=0;i < granularity;i++)
    {
      Serial.print("step:");
      Serial.print(i);
      Serial.print(" x:");
      Serial.print(S[i].x);
      Serial.print(" y:");
      Serial.println(S[i].y);
    }
}

