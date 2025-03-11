/* Includes ------------------------------------------------------------------*/
#include <Servo.h>    //to define and control servos
#include "FlexiTimer2.h"//to set a timer to manage all servos
/* Servos --------------------------------------------------------------------*/
//define 12 servos for 4 legs
Servo servo[4][3];
//define servos' ports
const int servo_pin[4][3] = { {2, 3, 4}, {5, 6, 7}, {8, 9, 10}, {11, 12, 13} };
int count=0;
/* Size of the robot ---------------------------------------------------------*/
const float length_a = 55;
const float length_b = 77.5;
const float length_c = 27.5;
const float length_side = 71;
const float z_absolute = -28;
/* Constants for movement ----------------------------------------------------*/
const float z_default = -50, z_up = -30, z_boot = z_absolute;
const float x_default = 62, x_offset = 0;
const float y_start = 0, y_step = 40;
const float y_default = x_default;
/* variables for movement ----------------------------------------------------*/
volatile float site_now[4][3];    //real-time coordinates of the end of each leg
volatile float site_expect[4][3]; //expected coordinates of the end of each leg
float temp_speed[4][3];   //each axis' speed, needs to be recalculated before each movement
float move_speed;     //movement speed
float speed_multiple = 1; //movement speed multiple
const float spot_turn_speed = 8;
const float leg_move_speed = 16;
const float body_move_speed = 4;
const float stand_seat_speed = 1;
volatile int rest_counter;      //+1/0.02s, for automatic rest
//functions' parameter
const float KEEP = 255;
//define PI for calculation
const float pi = 3.1415926;
/* Constants for turn --------------------------------------------------------*/
//temp length
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
//site for turn
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;
const int STEPS = 8;
char data = 0;
char p;
/* ---------------------------------------------------------------------------*/

/*
  - setup function
   ---------------------------------------------------------------------------*/
void setup()
{
  //start serial for debug
  Serial.begin(9600);

  Serial.println("Robot starts initialization");
  //initialize default parameter
  default_position();
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      site_now[i][j] = site_expect[i][j];
    }
  }
  //start servo service
  FlexiTimer2::set(20, servo_service);
  FlexiTimer2::start();

  //initialize servos
  servo_attach();
 
  
  stand();
  delay(1000);
}


void servo_attach(void)
{

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      servo[i][j].attach(servo_pin[i][j]);
      delay(100);
    }
  }
}

void servo_detach(void)
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      servo[i][j].detach();
      delay(100);
    }
  }
}
/*
  - loop function
   ---------------------------------------------------------------------------*/
void loop()
{
   
   if(Serial.available())      
   {
      data = Serial.read();           
      if(data == 'F') 
        {
         
         
         while(data!='0')
         {
          step_forward();
          data = Serial.read(); 
         }
         to_stand(2);
         delay(1000);
        }
      else if(data == 'B')        
         { 
         
          while(data!='0')
         {
          step_back();
          data = Serial.read(); 
         }
         to_stand(3);
         delay(1000);
        }    
      else if(data == 'L')        
         { 
          while(data!='0')
         {
          turn_left();
          data = Serial.read(); 
         }
         to_stand(3);
         delay(1000);
        }
      else if(data == 'R')        
        { 
          while(data!='0')
         {
          turn_right();
          data = Serial.read(); 
         }
         to_stand(2);
         delay(1000);
        } 
        else if(data == 'S')
        {
            stand();
            delay(1000);
        }
         else if(data == 'X')
        {
          sit();  
          delay(2000); 
        }
        else if(data == 'W')        
        { 
          hand_wave(3);
          delay(2000);
        } 
          else if(data == 'H')        
        { 
         hand_shake(3);
         delay(2000);
        }
        else if(data == 'O')        
        { 
         body_dance(10);
         delay(2000);
        }
        while(Serial.available()) {Serial.read();}
        
   }
}

/*
  - default position
  - blocking function
   ---------------------------------------------------------------------------*/

void default_position(void){
  set_site(0, x_default, y_start + y_step, z_boot);
  set_site(1, x_default + 11, y_start + y_step, z_boot);
  set_site(2, x_default, y_start + y_step, z_boot);
  set_site(3, x_default - 1, y_start + y_step, z_boot);
}

/*
  - sit
  - blocking function
   ---------------------------------------------------------------------------*/
void sit(void)
{
  move_speed = stand_seat_speed;
  // Define the desired x-coordinate for sitting (adjust this value as needed)
  float sittingZ = 0;  // Replace with the desired x-coordinate

  set_site(0, KEEP, KEEP , sittingZ);
  set_site(1, KEEP, KEEP, sittingZ);
  set_site(2, KEEP, KEEP, sittingZ);
  set_site(3, x_default + 17, KEEP, sittingZ - 13 );
  wait_all_reach();
}
/*
  - lower position
  - blocking function
   ---------------------------------------------------------------------------*/
void low(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 3; leg++)
  {
    set_site(leg, KEEP, KEEP, z_boot);
  }
  set_site(3, KEEP, KEEP, z_boot - 17);
  wait_all_reach();
}

/*
  - stand
  - blocking function
   ---------------------------------------------------------------------------*/
void stand(void)
{
  default_position();
  move_speed=stand_seat_speed;
  set_site(0,  KEEP , KEEP, z_default);
  set_site(1, KEEP, KEEP, z_default);
  set_site(2, KEEP, KEEP, z_default);
  set_site(3, KEEP, KEEP, z_default - 15);
  wait_all_reach();
}

/*
  - to stand position
  - blocking function
   ---------------------------------------------------------------------------*/
void to_stand(int ind){
  if(site_now[ind][1] == y_start){
    move_leg_smoothly(2, x_default + x_offset, y_start, z_up, STEPS);
    move_leg_smoothly(2, x_default, y_start, z_up, STEPS);
    move_leg_smoothly(2, x_default, y_start + y_step, z_up, STEPS);
    move_leg_smoothly(2, x_default, y_start + y_step, z_default, STEPS);
    
    move_leg_smoothly(3, x_default + x_offset, y_start, z_up, STEPS);
    move_leg_smoothly(3, x_default - 1, y_start, z_up, STEPS);
    move_leg_smoothly(3, x_default - 1, y_start + y_step, z_up, STEPS);
    move_leg_smoothly(3, x_default - 1, y_start + y_step, z_default - 15, STEPS);

    move_leg_smoothly(0, x_default - x_offset, y_start + y_step, z_up, STEPS);
    move_leg_smoothly(0, x_default, y_start + y_step, z_up, STEPS);
    move_leg_smoothly(0, x_default, y_start + y_step, z_default, STEPS);

    move_leg_smoothly(1, x_default - x_offset, y_start + y_step, z_up, STEPS);
    move_leg_smoothly(1, x_default + 11, y_start + y_step, z_up, STEPS);
    move_leg_smoothly(1, x_default + 11, y_start + y_step, z_default, STEPS);
  }
  else{
    move_leg_smoothly(0, x_default + x_offset, y_start, z_up, STEPS);
    move_leg_smoothly(0, x_default, y_start, z_up, STEPS);
    move_leg_smoothly(0, x_default, y_start + y_step, z_up, STEPS);
    move_leg_smoothly(0, x_default, y_start + y_step, z_default, STEPS);
    
    move_leg_smoothly(1, x_default + x_offset, y_start, z_up, STEPS);
    move_leg_smoothly(1, x_default + 11, y_start, z_up, STEPS);
    move_leg_smoothly(1, x_default + 11, y_start + y_step, z_up, STEPS);
    move_leg_smoothly(1, x_default + 11, y_start + y_step, z_default, STEPS);

    move_leg_smoothly(2, x_default - x_offset, y_start + y_step, z_up, STEPS);
    move_leg_smoothly(2, x_default, y_start + y_step, z_up, STEPS);
    move_leg_smoothly(2, x_default, y_start + y_step, z_default, STEPS);

    move_leg_smoothly(3, x_default - x_offset, y_start + y_step, z_up, STEPS);
    move_leg_smoothly(3, x_default - 1, y_start + y_step, z_up, STEPS);
    move_leg_smoothly(3, x_default - 1, y_start + y_step, z_default - 15, STEPS);
  }
}


/*
  - spot turn to left
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void turn_left()
{
    move_speed = spot_turn_speed;
    if (site_now[3][1] == y_start)
    {
      //leg 3&1 move
      move_leg_smoothly(3, x_default + x_offset, y_start, z_up, STEPS);

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      move_leg_smoothly(3, turn_x0 + x_offset, turn_y0, z_default, STEPS);

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      move_leg_smoothly(1, turn_x0 + x_offset, turn_y0, z_up, STEPS);

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_leg_smoothly(1, x_default + x_offset, y_start, z_default, STEPS);
      
    }
    else
    {
      //leg 0&2 move
      move_leg_smoothly(0, x_default + x_offset, y_start, z_up, STEPS);

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      move_leg_smoothly(0, turn_x0 + x_offset, turn_y0, z_default, STEPS);

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      move_leg_smoothly(2, turn_x0 + x_offset, turn_y0, z_up, STEPS);

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_leg_smoothly(2, x_default + x_offset, y_start, z_default, STEPS);
    }
    
}

/*
  - spot turn to right
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void turn_right()
{
    move_speed = spot_turn_speed;
    if (site_now[2][1] == y_start)
    {
      //leg 2&0 move
      move_leg_smoothly(2, x_default + x_offset, y_start, z_up, STEPS);

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      move_leg_smoothly(2, turn_x0 + x_offset, turn_y0, z_default, STEPS);

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      move_leg_smoothly(0, turn_x0 + x_offset, turn_y0, z_up, STEPS);

      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_leg_smoothly(0, x_default + x_offset, y_start, z_default, STEPS);
    }
    else
    {
      //leg 1&3 move
      move_leg_smoothly(1, x_default + x_offset, y_start, z_up, STEPS);

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      move_leg_smoothly(1, turn_x0 + x_offset, turn_y0, z_default, STEPS);

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);

      move_leg_smoothly(3, turn_x0 + x_offset, turn_y0, z_up, STEPS);

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      move_leg_smoothly(3, x_default + x_offset, y_start, z_default, STEPS);
    }
    
}

/*
  - go forward
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void step_forward()
{
  move_speed = leg_move_speed;
  
    if (site_now[2][1] == y_start)
    {
      //leg 2&1 move
      move_leg_smoothly(2, x_default + x_offset, y_start, z_up, STEPS);
      move_leg_smoothly(2, x_default + x_offset, y_start + 2 * y_step, z_up, STEPS);
      move_leg_smoothly(2, x_default + x_offset, y_start + 2 * y_step, z_default, STEPS);

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      move_leg_smoothly(1, x_default + x_offset, y_start + 2 * y_step, z_up, STEPS);
      move_leg_smoothly(1, x_default + x_offset, y_start, z_up, STEPS);
      move_leg_smoothly(1, x_default + x_offset, y_start, z_default, STEPS);
    }
    else
    {
      //leg 0&3 move
      move_leg_smoothly(0, x_default + x_offset, y_start, z_up, STEPS);
      move_leg_smoothly(0, x_default + x_offset, y_start + 2 * y_step, z_up, STEPS);
      move_leg_smoothly(0, x_default + x_offset, y_start + 2 * y_step, z_default, STEPS);

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      move_leg_smoothly(3, x_default + x_offset, y_start + 2 * y_step, z_up, STEPS);
      move_leg_smoothly(3, x_default + x_offset, y_start, z_up, STEPS);
      move_leg_smoothly(3, x_default + x_offset, y_start, z_default, STEPS);
    }
  

}


/*
  - go back
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void step_back()
{
    move_speed = leg_move_speed;
    if (site_now[3][1] == y_start)
    {
      //leg 3&0 move
      move_leg_smoothly(3, x_default + x_offset, y_start, z_up, STEPS);
      move_leg_smoothly(3, x_default + x_offset, y_start + 2 * y_step, z_up, STEPS);
      move_leg_smoothly(3, x_default + x_offset, y_start + 2 * y_step, z_default, STEPS);

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      move_leg_smoothly(0, x_default + x_offset, y_start + 2 * y_step, z_up, STEPS);
      move_leg_smoothly(0, x_default + x_offset, y_start, z_up, STEPS);
      move_leg_smoothly(0, x_default + x_offset, y_start, z_default, STEPS);
    }
    else
    {
      //leg 1&2 move
      move_leg_smoothly(1, x_default + x_offset, y_start, z_up, STEPS);
      move_leg_smoothly(1, x_default + x_offset, y_start + 2 * y_step, z_up, STEPS);
      move_leg_smoothly(1, x_default + x_offset, y_start + 2 * y_step, z_default, STEPS);

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      move_leg_smoothly(2, x_default + x_offset, y_start + 2 * y_step, z_up, STEPS);
      move_leg_smoothly(2, x_default + x_offset, y_start, z_up, STEPS);
      move_leg_smoothly(2, x_default + x_offset, y_start, z_default, STEPS);
    }
    
}

void body_left(int i)
{
  set_site(0, site_now[0][0] + i, KEEP, KEEP);
  set_site(1, site_now[1][0] + i, KEEP, KEEP);
  set_site(2, site_now[2][0] - i, KEEP, KEEP);
  set_site(3, site_now[3][0] - i, KEEP, KEEP);
  wait_all_reach();
}

void body_right(int i)
{
  set_site(0, site_now[0][0] - i, KEEP, KEEP);
  set_site(1, site_now[1][0] - i, KEEP, KEEP);
  set_site(2, site_now[2][0] + i, KEEP, KEEP);
  set_site(3, site_now[3][0] + i, KEEP, KEEP);
  wait_all_reach();
}

void hand_wave(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  body_left(15);
  x_tmp = site_now[0][0];
  y_tmp = site_now[0][1];
  z_tmp = site_now[0][2];
  move_speed = body_move_speed;
  for (int j = 0; j < i; j++)
  {
    set_site(0, turn_x1, turn_y1, 50);
    wait_all_reach();
    set_site(0, turn_x0, turn_y0, 50);
    wait_all_reach();
  }
  set_site(0, x_tmp, y_tmp, z_tmp);
  wait_all_reach();
  move_speed = 1;
  body_right(15);
}

void hand_shake(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  body_left(15);
  x_tmp = site_now[0][0];
  y_tmp = site_now[0][1];
  z_tmp = site_now[0][2];
  move_speed = body_move_speed;
  for (int j = 0; j < i; j++)
  {
    set_site(0, x_default - 30, y_start + 2 * y_step, 55);
    wait_all_reach();
    set_site(0, x_default - 30, y_start + 2 * y_step, 10);
    wait_all_reach();
  }
  set_site(0, x_tmp, y_tmp, z_tmp);
  wait_all_reach();
  move_speed = 1;
  body_right(15);
}

void head_up(int i)
{
  set_site(0, KEEP, KEEP, site_now[0][2] - i);
  set_site(1, KEEP, KEEP, site_now[1][2] + i);
  set_site(2, KEEP, KEEP, site_now[2][2] - i);
  set_site(3, KEEP, KEEP, site_now[3][2] + i - 7);
  wait_all_reach();
}

void head_down(int i)
{
  set_site(0, KEEP, KEEP, site_now[0][2] + i);
  set_site(1, KEEP, KEEP, site_now[1][2] - i);
  set_site(2, KEEP, KEEP, site_now[2][2] + i);
  set_site(3, KEEP, KEEP, site_now[3][2] - i - 7);
  wait_all_reach();
}

void body_dance(int i)
{
  float tmp[4][3];

  for (int m = 0; m < 4; m++)
  {
    for (int n = 0; n < 3; n++)
    {
      tmp[m][n] = site_now[m][n];
    }
  }
  
  float body_dance_speed = 2;
  sit();
  move_speed = 1;
  set_site(0, KEEP, y_default, KEEP);
  set_site(1, KEEP, y_default, KEEP);
  set_site(2, KEEP, y_default, KEEP);
  set_site(3, KEEP, y_default, KEEP);
  wait_all_reach();

  set_site(0, KEEP, KEEP, z_default - 20);
  set_site(1, KEEP, KEEP, z_default - 20);
  set_site(2, KEEP, KEEP, z_default - 20);
  set_site(3, KEEP, KEEP, z_default - 23);
  wait_all_reach();
  wait_all_reach();
  move_speed = body_dance_speed;
  head_up(30);
  for (int j = 0; j < i; j++)
  {
    if (j > i / 4)
      move_speed = body_dance_speed * 2;
    if (j > i / 2)
      move_speed = body_dance_speed * 3;
    set_site(0, KEEP, y_default - 20, KEEP);
    set_site(1, KEEP, y_default + 20, KEEP);
    set_site(2, KEEP, y_default - 20, KEEP);
    set_site(3, KEEP, y_default + 20, KEEP);
    wait_all_reach();
    set_site(0, KEEP, y_default + 20, KEEP);
    set_site(1, KEEP, y_default - 20, KEEP);
    set_site(2, KEEP, y_default + 20, KEEP);
    set_site(3, KEEP, y_default - 20, KEEP);
    wait_all_reach();
  }
  move_speed = body_dance_speed;
  head_down(30);
  move_speed = body_move_speed;
  set_site(0, tmp[0][0], tmp[0][1], tmp[0][2]);
  set_site(1, tmp[1][0], tmp[1][1], tmp[1][2]);
  set_site(2, tmp[2][0], tmp[2][1], tmp[2][2]);
  set_site(3, tmp[3][0], tmp[3][1], tmp[3][2]);
  wait_all_reach();
}


/*
  - microservos service /timer interrupt function/50Hz
  - when set site expected,this function move the end point to it in a straight line
  - temp_speed[4][3] should be set before set expect site,it make sure the end point
   move in a straight line,and decide move speed.
   ---------------------------------------------------------------------------*/
void servo_service(void)
{
  sei();
  static float alpha, beta, gamma;

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];
      else
        site_now[i][j] = site_expect[i][j];
    }

    cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
    polar_to_servo(i, alpha, beta, gamma);
  }

  rest_counter++;
}

/*
  - set one of end points' expect site
  - this founction will set temp_speed[4][3] at same time
  - non - blocking function
   ---------------------------------------------------------------------------*/
void set_site(int leg, float x, float y, float z)
{
  float length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP)
    length_x = x - site_now[leg][0];
  if (y != KEEP)
    length_y = y - site_now[leg][1];
  if (z != KEEP)
    length_z = z - site_now[leg][2];

  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
  temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
  temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;

  if (x != KEEP)
    site_expect[leg][0] = x;
  if (y != KEEP)
    site_expect[leg][1] = y;
  if (z != KEEP)
    site_expect[leg][2] = z;
}

/*
  - wait one of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_reach(int leg)
{
  while (1)
    if (site_now[leg][0] == site_expect[leg][0])
      if (site_now[leg][1] == site_expect[leg][1])
        if (site_now[leg][2] == site_expect[leg][2])
          break;
}

/*
  - wait all of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_all_reach(void)
{
  for (int i = 0; i < 4; i++)
    wait_reach(i);
}

/*
  - smoother movement
  - blocking function
   ---------------------------------------------------------------------------*/

void move_leg_smoothly(int leg_index, float target_x, float target_y, float target_z, int steps)
{
  float x_diff = (target_x - site_now[leg_index][0]) / steps;
  float y_diff = (target_y - site_now[leg_index][1]) / steps;
  float z_diff = (target_z - site_now[leg_index][2]) / steps;
  
  for (int i = 0; i < steps; i++)
  {
    set_site(leg_index,
             site_now[leg_index][0] + x_diff,
             site_now[leg_index][1] + y_diff,
             site_now[leg_index][2] + z_diff);
    wait_all_reach();
  }
}


/*
  - trans site from cartesian to polar
  - mathematical model 2/2
   ---------------------------------------------------------------------------*/
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z)
{
  //calculate w-z degree
  float v, w;
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
  v = w - length_c;
  alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
  beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
  //calculate x-y-z degree
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  //trans degree pi->180
  alpha = alpha / pi * 180;
  beta = beta / pi * 180;
  gamma = gamma / pi * 180;
}

/*
  - trans site from polar to microservos
  - mathematical model map to fact
  - the errors saved in eeprom will be add
   ---------------------------------------------------------------------------*/
void polar_to_servo(int leg, float alpha, float beta, float gamma)
{
  if (leg == 0)
  {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  }
  else if (leg == 1)
  {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  }
  else if (leg == 2)
  {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  }
  else if (leg == 3)
  {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  }

  servo[leg][0].write(alpha);
  servo[leg][1].write(beta);
  servo[leg][2].write(gamma);
}
