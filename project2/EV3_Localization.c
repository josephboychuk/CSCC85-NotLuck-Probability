/*

  CSC C85 - Embedded Systems - Project # 1 - EV3 Robot Localization
  
 This file provides the implementation of all the functionality required for the EV3
 robot localization project. Please read through this file carefully, and note the
 sections where you must implement functionality for your bot. 
 
 You are allowed to change *any part of this file*, not only the sections marked
 ** TO DO **. You are also allowed to add functions as needed (which must also
 be added to the header file). However, *you must clearly document* where you 
 made changes so your work can be properly evaluated by the TA.

 NOTES on your implementation:

 * It should be free of unreasonable compiler warnings - if you choose to ignore
   a compiler warning, you must have a good reason for doing so and be ready to
   defend your rationale with your TA.
 * It must be free of memory management errors and memory leaks - you are expected
   to develop high wuality, clean code. Test your code extensively with valgrind,
   and make sure its memory management is clean.
 
 In a nutshell, the starter code provides:
 
 * Reading a map from an input image (in .ppm format). The map is bordered with red, 
   must have black streets with yellow intersections, and buildings must be either
   blue, green, or be left white (no building).
   
 * Setting up an array with map information which contains, for each intersection,
   the colours of the buildings around it in ** CLOCKWISE ** order from the top-left.
   
 * Initialization of the EV3 robot (opening a socket and setting up the communication
   between your laptop and your bot)
   
 What you must implement:
 
 * All aspects of robot control:
   - Finding and then following a street
   - Recognizing intersections
   - Scanning building colours around intersections
   - Detecting the map boundary and turning around or going back - the robot must not
     wander outside the map (though of course it's possible parts of the robot will
     leave the map while turning at the boundary)

 * The histogram-based localization algorithm that the robot will use to determine its
   location in the map - this is as discussed in lecture.

 * Basic robot exploration strategy so the robot can scan different intersections in
   a sequence that allows it to achieve reliable localization
   
 * Basic path planning - once the robot has found its location, it must drive toward a 
   user-specified position somewhere in the map.

 --- OPTIONALLY but strongly recommended ---
 
  The starter code provides a skeleton for implementing a sensor calibration routine,
 it is called when the code receives -1  -1 as target coordinates. The goal of this
 function should be to gather informatin about what the sensor reads for different
 colours under the particular map/room illumination/battery level conditions you are
 working on - it's entirely up to you how you want to do this, but note that careful
 calibration would make your work much easier, by allowing your robot to more
 robustly (and with fewer mistakes) interpret the sensor data into colours. 
 
   --> The code will exit after calibration without running localization (no target!)
       SO - your calibration code must *save* the calibration information into a
            file, and you have to add code to main() to read and use this
            calibration data yourselves.
   
 What you need to understand thoroughly in order to complete this project:
 
 * The histogram localization method as discussed in lecture. The general steps of
   probabilistic robot localization.

 * Sensors and signal management - your colour readings will be noisy and unreliable,
   you have to handle this smartly
   
 * Robot control with feedback - your robot does not perform exact motions, you can
   assume there will be error and drift, your code has to handle this.
   
 * The robot control API you will use to get your robot to move, and to acquire 
   sensor data. Please see the API directory and read through the header files and
   attached documentation
   
 Starter code:
 F. Estrada, 2018 - for CSC C85 
 
*/

#include "EV3_Localization.h"
#define FORWARD_MOTION 90
#define RIGHT_MOTION 5
#define LEFT_MOTION 5 
#define LOCALIZED_THRESHOLD 0.999
#define BATTERY 80
#define BATTERY_ADJUST (1 + (0.01 *(100 - BATTERY)))

int map[400][4];            // This holds the representation of the map, up to 20x20
                            // intersections, raster ordered, 4 building colours per
                            // intersection.
int sx, sy;                 // Size of the map (number of intersections along x and y)
double beliefs[400][4];     // Beliefs for each location and motion direction

int main(int argc, char *argv[])
{
 char mapname[1024];
 int dest_x, dest_y, rx, ry;
 unsigned char *map_image;
 
 memset(&map[0][0],0,400*4*sizeof(int));
 sx=0;
 sy=0;
 
 if (argc<4)
 {
  fprintf(stderr,"Usage: EV3_Localization map_name dest_x dest_y\n");
  fprintf(stderr,"    map_name - should correspond to a properly formatted .ppm map image\n");
  fprintf(stderr,"    dest_x, dest_y - target location for the bot within the map, -1 -1 calls calibration routine\n");
  exit(1);
 }
 strcpy(&mapname[0],argv[1]);
 dest_x=atoi(argv[2]);
 dest_y=atoi(argv[3]);

 if (dest_x==-1&&dest_y==-1)
 {
  calibrate_sensor();
  exit(1);
 }

 /******************************************************************************************************************
  * OPTIONAL TO DO: If you added code for sensor calibration, add just below this comment block any code needed to
  *   read your calibration data for use in your localization code. Skip this if you are not using calibration
  * ****************************************************************************************************************/
 
 // Your code for reading any calibration information should not go below this line //
 
 map_image=readPPMimage(&mapname[0],&rx,&ry);
 if (map_image==NULL)
 {
  fprintf(stderr,"Unable to open specified map image\n");
  exit(1);
 }
 
 if (parse_map(map_image, rx, ry)==0)
 { 
  fprintf(stderr,"Unable to parse input image map. Make sure the image is properly formatted\n");
  free(map_image);
  exit(1);
 }

 if (dest_x<0||dest_x>=sx||dest_y<0||dest_y>=sy)
 {
  fprintf(stderr,"Destination location is outside of the map\n");
  free(map_image);
  exit(1);
 }

 // Initialize beliefs - uniform probability for each location and direction
 for (int j=0; j<sy; j++)
  for (int i=0; i<sx; i++)
  {
   beliefs[i+(j*sx)][0]=1.0/(double)(sx*sy*4);
   beliefs[i+(j*sx)][1]=1.0/(double)(sx*sy*4);
   beliefs[i+(j*sx)][2]=1.0/(double)(sx*sy*4);
   beliefs[i+(j*sx)][3]=1.0/(double)(sx*sy*4);
  }

 // Open a socket to the EV3 for remote controlling the bot.
 if (BT_open(HEXKEY)!=0)
 {
  fprintf(stderr,"Unable to open comm socket to the EV3, make sure the EV3 kit is powered on, and that the\n");
  fprintf(stderr," hex key for the EV3 matches the one in EV3_Localization.h\n");
  free(map_image);
  exit(1);
 }
 fprintf(stderr,"All set, ready to go!\n");
 
/*******************************************************************************************************************************
 *
 *  TO DO - Implement the main localization loop, this loop will have the robot explore the map, scanning intersections and
 *          updating beliefs in the beliefs array until a single location/direction is determined to be the correct one.
 * 
 *          The beliefs array contains one row per intersection (recall that the number of intersections in the map_image
 *          is given by sx, sy, and that the map[][] array contains the colour indices of buildings around each intersection.
 *          Indexing into the map[][] and beliefs[][] arrays is by raster order, so for an intersection at i,j (with 0<=i<=sx-1
 *          and 0<=j<=sy-1), index=i+(j*sx)
 *  
 *          In the beliefs[][] array, you need to keep track of 4 values per intersection, these correspond to the belief the
 *          robot is at that specific intersection, moving in one of the 4 possible directions as follows:
 * 
 *          beliefs[i][0] <---- belief the robot is at intersection with index i, facing UP
 *          beliefs[i][1] <---- belief the robot is at intersection with index i, facing RIGHT
 *          beliefs[i][2] <---- belief the robot is at intersection with index i, facing DOWN
 *          beliefs[i][3] <---- belief the robot is at intersection with index i, facing LEFT
 * 
 *          Initially, all of these beliefs have uniform, equal probability. Your robot must scan intersections and update
 *          belief values based on agreement between what the robot sensed, and the colours in the map. 
 * 
 *          You have two main tasks these are organized into two major functions:
 * 
 *          robot_localization()    <---- Runs the localization loop until the robot's location is found
 *          go_to_target()          <---- After localization is achieved, takes the bot to the specified map location
 * 
 *          The target location, read from the command line, is left in dest_x, dest_y
 * 
 *          Here in main(), you have to call these two functions as appropriate. But keep in mind that it is always possible
 *          that even if your bot managed to find its location, it can become lost again while driving to the target
 *          location, or it may be the initial localization was wrong and the robot ends up in an unexpected place - 
 *          a very solid implementation should give your robot the ability to determine it's lost and needs to 
 *          run localization again.
 *
 *******************************************************************************************************************************/  

 // HERE - write code to call robot_localization() and go_to_target() as needed, any additional logic required to get the
 //        robot to complete its task should be here.

 int scanned_colours[4];
 int localized = 0;
 int robot_x = -1;
 int robot_y = -1;
 int direction = -1;
 
 while (localized != 1)
 {
   localized = robot_localization(&robot_x, &robot_y, &direction);
 }

 fprintf(stderr, "The robot has cleverly deduced it is at (%d, %d) facing (%d)\n", robot_x, robot_y, direction);

 if (go_to_target(robot_x, robot_y, direction, dest_x, dest_y) == 1)
 {
  fprintf(stderr, "Made it to destination!\n");
 }
 else
 {
  fprintf(stderr, "Damn, got lost and couldn't make it to destination\n");
 }

 BT_all_stop(0);
 
 // Cleanup and exit - DO NOT WRITE ANY CODE BELOW THIS LINE
 BT_close();
 free(map_image);
 exit(0);
}

// Adjusts the wheels forward for the intersection
void intersection_adjust(void) {

  while (BT_read_colour_sensor(PORT_1) == 4)
  {
    BT_turn(MOTOR_B, -6* BATTERY_ADJUST, MOTOR_A, -6 * BATTERY_ADJUST);
  }
  BT_all_stop(0);

  BT_timed_motor_port_start_v2(MOTOR_B, -8, 450 * BATTERY_ADJUST);
  BT_timed_motor_port_start_v2(MOTOR_A, -8, 450 * BATTERY_ADJUST);
  sleep(1);
  find_street();
}

int find_street(void)   
{
 /*
  * This function gets your robot onto a street, wherever it is placed on the map. You can do this in many ways, but think
  * about what is the most effective and reliable way to detect a street and stop your robot once it's on it.
  * 
  * You can use the return value to indicate success or failure, or to inform the rest of your code of the state of your
  * bot after calling this function
  */   
  while (BT_read_colour_sensor(PORT_1) != 1)
  {
    BT_turn(MOTOR_B, -7 * BATTERY_ADJUST, MOTOR_A, 7 * BATTERY_ADJUST);
  }
  if (BT_read_colour_sensor(PORT_1) == 1) return 1;
  return 0;
}

int drive_along_street(void)
{
 /*
  * This function drives your bot along a street, making sure it stays on the street without straying to other pars of
  * the map. It stops at an intersection.
  * 
  * You can implement this in many ways, including a controlled (PID for example), a neural network trained to track and
  * follow streets, or a carefully coded process of scanning and moving. It's up to you, feel free to consult your TA
  * or the course instructor for help carrying out your plan.
  * 
  * You can use the return value to indicate success or failure, or to inform the rest of your code of the state of your
  * bot after calling this function.
  */   
  while (BT_read_colour_sensor(PORT_1) == 1)
  {
    BT_turn(MOTOR_B, -8 * BATTERY_ADJUST, MOTOR_A, -8 * BATTERY_ADJUST);
  }
  return 1;
}

int drive_away_from_boundary(void)
{
 /*
  * This function makes the robot reverse away from a boundary until the previous intersection, at which
  * point the robot will turn right.
  */   

  BT_timed_motor_port_start_v2(MOTOR_B, 8, 450 * BATTERY_ADJUST);
  BT_timed_motor_port_start_v2(MOTOR_A, 8, 450 * BATTERY_ADJUST);

  while (BT_read_colour_sensor(PORT_1) == 1)
  {
    BT_turn(MOTOR_B, 8 * BATTERY_ADJUST, MOTOR_A, 8 * BATTERY_ADJUST);
  }
  if (BT_read_colour_sensor(PORT_1) == 4){
    return 1;
  }
  return 0;
}

int scan_intersection(int *tl, int *tr, int *br, int *bl)
{
 /*
  * This function carries out the intersection scan - the bot should (obviously) be placed at an intersection for this,
  * and the specific set of actions will depend on how you designed your bot and its sensor. Whatever the process, you
  * should make sure the intersection scan is reliable - i.e. the positioning of the sensor is reliably over the buildings
  * it needs to read, repeatably, and as the robot moves over the map.
  * 
  * Use the APIs sensor reading calls to poll the sensors. You need to remember that sensor readings are noisy and 
  * unreliable so * YOU HAVE TO IMPLEMENT SOME KIND OF SENSOR / SIGNAL MANAGEMENT * to obtain reliable measurements.
  * 
  * Recall your lectures on sensor and noise management, and implement a strategy that makes sense. Document your process
  * in the code below so your TA can quickly understand how it works.
  * 
  * Once your bot has read the colours at the intersection, it must return them using the provided pointers to 4 integer
  * variables:
  * 
  * tl - top left building colour
  * tr - top right building colour
  * br - bottom right building colour
  * bl - bottom left building colour
  * 
  * The function's return value can be used to indicate success or failure, or to notify your code of the bot's state
  * after this call.
  */
 
  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/
  intersection_adjust();
  // BT_timed_motor_port_start(MOTOR_B, -20, 0, 675 * BATTERY_ADJUST, 0);
  // BT_timed_motor_port_start(MOTOR_A, 20, 0, 675 * BATTERY_ADJUST, 0);
  // sleep(2);

  int pos = BT_read_gyro_sensor(PORT_2);

  while(abs(pos - BT_read_gyro_sensor(PORT_2)) < 45)
  {
    BT_turn(MOTOR_B, -15, MOTOR_A, 15);
  }

  *(tr) = BT_read_colour_sensor(PORT_1);

  // BT_timed_motor_port_start(MOTOR_B, -20, 0, 1350 * BATTERY_ADJUST, 0);
  // BT_timed_motor_port_start(MOTOR_A, 20, 0, 1350 * BATTERY_ADJUST, 0);
  // sleep(3);
  pos = BT_read_gyro_sensor(PORT_2);

  while(abs(pos - BT_read_gyro_sensor(PORT_2)) < 90)
  {
    BT_turn(MOTOR_B, -15, MOTOR_A, 15);
  }

  *(br) = BT_read_colour_sensor(PORT_1);

  // BT_timed_motor_port_start(MOTOR_B, -20, 0, 1350 * BATTERY_ADJUST, 0);
  // BT_timed_motor_port_start(MOTOR_A, 20, 0, 1350 * BATTERY_ADJUST, 0);
  // sleep(3);

  pos = BT_read_gyro_sensor(PORT_2);

  while(abs(pos - BT_read_gyro_sensor(PORT_2)) < 90)
  {
    BT_turn(MOTOR_B, -15, MOTOR_A, 15);
  }
  *(bl) = BT_read_colour_sensor(PORT_1);


  // BT_timed_motor_port_start(MOTOR_B, -20, 0, 1350 * BATTERY_ADJUST, 0);
  // BT_timed_motor_port_start(MOTOR_A, 20, 0, 1350 * BATTERY_ADJUST, 0);
  // sleep(3);
  pos = BT_read_gyro_sensor(PORT_2);

  while(abs(pos - BT_read_gyro_sensor(PORT_2)) < 90)
  {
    BT_turn(MOTOR_B, -15, MOTOR_A, 15);
  }
  *(tl) = BT_read_colour_sensor(PORT_1);

  // BT_timed_motor_port_start(MOTOR_B, -20, 0, 675 * BATTERY_ADJUST, 0);
  // BT_timed_motor_port_start(MOTOR_A, 20, 0, 675 * BATTERY_ADJUST, 0);
  // sleep(2);

  find_street();

 // Return invalid colour values, and a zero to indicate failure (you will replace this with your code)
 return 1;
 
}

int turn_at_intersection(int turn_direction)
{
 /*
  * This function is used to have the robot turn either left or right at an intersection (obviously your bot can not just
  * drive forward!). 
  * 
  * If turn_direction=0, turn right, else if turn_direction=1, turn left.
  * 
  * You're free to implement this in any way you like, but it should reliably leave your bot facing the correct direction
  * and on a street it can follow. 
  * 
  * You can use the return value to indicate success or failure, or to inform your code of the state of the bot
  */

  int pos = BT_read_gyro_sensor(PORT_2);

  if (turn_direction == 0){
    while(abs(pos - BT_read_gyro_sensor(PORT_2)) < 85)
    {
      BT_turn(MOTOR_B, -15, MOTOR_A, 15);
    }
  }
  else if (turn_direction == 1){
    while(abs(pos - BT_read_gyro_sensor(PORT_2)) < 95)
    {
      BT_turn(MOTOR_B, 15, MOTOR_A, -15);
    }
  }

  find_street();

  return 1;
}

// Reads the contents of a 4 index array as a 4 digit base 10 integer for comparing colours
int read_array_num(int orientation, int *array_to_parse){
  if (orientation == 1) return (array_to_parse[1] * 1000) + (array_to_parse[2] * 100) + (array_to_parse[3] * 10) + array_to_parse[0];
  if (orientation == 2) return (array_to_parse[2] * 1000) + (array_to_parse[3] * 100) + (array_to_parse[0] * 10) + array_to_parse[1];
  if (orientation == 3) return (array_to_parse[3] * 1000) + (array_to_parse[0] * 100) + (array_to_parse[1] * 10) + array_to_parse[2];
  return (array_to_parse[0] * 1000) + (array_to_parse[1] * 100) + (array_to_parse[2] * 10) + array_to_parse[3]; 
}

void print_beliefs(double bel[400][4])
{
  for (int i = 0; i < 15; i++)
  {
    fprintf(stderr, "%d(", i);
    for (int j = 0; j < 4; j++)
    {
      fprintf(stderr, "%f, ", bel[i][j]);
    }
    fprintf(stderr, ")  ");
    if (i % 3 == 2)
    {
      fprintf(stderr, "\n");
    }
  }
}


int do_motion_step(void)
{
  /*  This function controls the entire motion step of the localization loop. It should drive the robot
      to the next intersection, or in the case of running into a boundary return to the same intersection
      and rotate 90 degrees CW.

      The function returns an indication of the motion that it just completed

        1  -  The robot drove forward to the next intersection
        2  -  The robot rotated right because of the boundary
       (-1) - The motion step failed (Only returned in catastrophic error)

      This function is separated from the localization because this motion is closely measured. 
      The localization function should not keep track of its own movement.
  */
  int motion_complete = 0;
  while (motion_complete == 0){
    // In general, the loop should attempt to drive to an important point (intersection or boundary)
    find_street();
    drive_along_street();
    BT_all_stop(0);
    // Stopped driving because an intersection was detected
    if (BT_read_colour_sensor(PORT_1) == 4)
    {
      //BT_all_stop(1);
      motion_complete = 1;
    }

    // Stopped driving because a boundary was detected
    else if (BT_read_colour_sensor(PORT_1) == 5)
    {
      drive_away_from_boundary();
      BT_all_stop(0);
      intersection_adjust();
      turn_at_intersection(0);
      BT_all_stop(1);
      motion_complete = 2;
    }
    
    // Something horrible has happened :O
    else if (BT_read_colour_sensor(PORT_1) == 0)
    {
      BT_all_stop(1);
      motion_complete = -1;
    }

  }
  return motion_complete;
}

int robot_localization(int *robot_x, int *robot_y, int *direction)
{
 /*  This function implements the main robot localization process. You have to write all code that will control the robot
  *  and get it to carry out the actions required to achieve localization.
  *
  *  Localization process:
  *
  *  - Find the street, and drive along the street toward an intersection
  *  - Scan the colours of buildings around the intersection
  *  - Update the beliefs in the beliefs[][] array according to the sensor measurements and the map data
  *  - Repeat the process until a single intersection/facing direction is distintly more likely than all the rest
  * 
  *  * We have provided headers for the following functions:
  * 
  *  find_street()
  *  drive_along_street()
  *  scan_intersection()
  *  turn_at_intersection()
  * 
  *  You *do not* have to use them, and can write your own to organize your robot's work as you like, they are
  *  provided as a suggestion.
  * 
  *  Note that *your bot must explore* the map to achieve reliable localization, this means your intersection
  *  scanning strategy should not rely exclusively on moving forward, but should include turning and exploring
  *  other streets than the one your bot was initially placed on.
  * 
  *  For each of the control functions, however, you will need to use the EV3 API, so be sure to become familiar with
  *  it.
  * 
  *  In terms of sensor management - the API allows you to read colours either as indexed values or RGB, it's up to
  *  you which one to use, and how to interpret the noisy, unreliable data you're likely to get from the sensor
  *  in order to update beliefs.
  * 
  *  HOWEVER: *** YOU must document clearly both in comments within this function, and in your report, how the
  *               sensor is used to read colour data, and how the beliefs are updated based on the sensor readings.
  * 
  *  DO NOT FORGET - Beliefs should always remain normalized to be a probability distribution, that means the
  *                  sum of beliefs over all intersections and facing directions must be 1 at all times.
  * 
  *  The function receives as input pointers to three integer values, these will be used to store the estimated
  *   robot's location and facing direction. The direction is specified as:
  *   0 - UP
  *   1 - RIGHT
  *   2 - BOTTOM
  *   3 - LEFT
  * 
  *  The function's return value is 1 if localization was successful, and 0 otherwise.
  */
 
  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/

  double temp_beliefs[15][4];
  int scanned_colours[4];
  double sensor_accuracy = 95;
  int localizing = 1;

  while (localizing == 1){
    // Initial step when starting at random point
    int found_intersection = 0;
    while (found_intersection == 0 && BT_read_colour_sensor(PORT_1) != 4)
    {
      find_street();
      drive_along_street();
      if (BT_read_colour_sensor(PORT_1) == 5)
      {
        drive_away_from_boundary();
      }
      else
      {
        found_intersection = 1;
      }
    }

    // Think about 2 steps: sense and motion

    // **SENSE**
    //    Should be at intersection
    scan_intersection(&scanned_colours[0], &scanned_colours[1], &scanned_colours[2], &scanned_colours[3]);
    // Compare these values to each map point iteration
    fprintf(stderr, "scanned colours = %d, %d, %d, %d\n", scanned_colours[0], scanned_colours[1], scanned_colours[2], scanned_colours[3]);
    for (int intersection = 0; intersection < 15; intersection++)
    {
      for (int orientation = 0; orientation < 4; orientation++)
      {
        if (read_array_num(orientation, map[intersection]) == read_array_num(0, scanned_colours)) // Fixed sensor reading at 0
        {
          
          beliefs[intersection][orientation] = beliefs[intersection][orientation] * sensor_accuracy;
        }
        else {
          beliefs[intersection][orientation] = beliefs[intersection][orientation] * (100 - sensor_accuracy);
        }
      }
    }

    // At this point, we should have completed the SENSE step. All beliefs should now be held in temp_beliefs

    // **MOTION**
    // Call motion function to actually tell the robot to do things. The motion function should return
    // exactly what the robot did
    int motion = do_motion_step();

    // Reset normalizer for probability calculation
    double normalizer = 0;
    // At intersection
    if (motion == 1)
    {
      fprintf(stderr, "Moved to next intersection\n");
      for (int intersection = 0; intersection < 15; intersection++)
      {
        for (int orientation = 0; orientation < 4; orientation++)
        {
          /* Each calculation requires 3 different factors from the motion model
                1. The motion in the given direction from the previous intersection   (ex. int.2[down]  --down-->   int.5[down])
                2. The motion from the CW perpendicular movement                      (ex. int.2[right] --right-->  int.5[down])
                3. The motion from the CCW perpendicular movement                     (ex. int.2[left]  --left-->   int.5[down])
          */
          if (orientation == 0)
          {
            // All intersections except bottom 3 (12, 13, 14)
            if (intersection < 12){
              double forward = beliefs[intersection + 3][0] * FORWARD_MOTION;
              double left = beliefs[intersection + 3][1] * LEFT_MOTION;
              double right = beliefs[intersection + 3][3] * RIGHT_MOTION;
              temp_beliefs[intersection][orientation] = forward + left + right;
            } 
            else 
            {
              temp_beliefs[intersection][orientation] = beliefs[intersection][orientation] * 1;
            }
          }
          else if (orientation == 1)
          {
            // All intersections not on left side (0, 3, 6, 9, 12)
            if (intersection % 3 != 0){
              double forward = beliefs[intersection - 1][1] * FORWARD_MOTION;
              double left = beliefs[intersection - 1][2] * LEFT_MOTION;
              double right = beliefs[intersection - 1][0] * RIGHT_MOTION;
              temp_beliefs[intersection][orientation] = forward + left + right;
            } 
            else 
            {
              temp_beliefs[intersection][orientation] = beliefs[intersection][orientation] * 1;
            }
          }
          else if (orientation == 2)
          {
            // All intersections after the first 3 (0, 1, 2)
            if (intersection > 2){
              double forward = beliefs[intersection - 3][2] * FORWARD_MOTION;
              double left = beliefs[intersection - 3][3] * LEFT_MOTION;
              double right = beliefs[intersection - 3][1] * RIGHT_MOTION;
              temp_beliefs[intersection][orientation] = forward + left + right;
            } 
            else 
            {
              temp_beliefs[intersection][orientation] = beliefs[intersection][orientation] * 1;
            }
          }
          else if (orientation == 3)
          {
            // All intersections not on the right side (2, 5, 8, 11, 14)
            if (intersection % 3 != 2){
              double forward = beliefs[intersection + 1][3] * FORWARD_MOTION;
              double left = beliefs[intersection + 1][0] * LEFT_MOTION;
              double right = beliefs[intersection + 1][2] * RIGHT_MOTION;
              temp_beliefs[intersection][orientation] = forward + left + right;
            } 
            else 
            {
              temp_beliefs[intersection][orientation] = beliefs[intersection][orientation] * 1;
            }
          }

          // Create a running sum of all belief values to use for normalizing
          normalizer += temp_beliefs[intersection][orientation];
        }
      }
    }

    // At Boundary
    else if (motion == 2)
    {
      fprintf(stderr, "Moved away from intersection and turning\n");
      for (int intersection = 0; intersection < 15; intersection++)
      {
        for (int orientation = 0; orientation < 4; orientation++)
        {
          if (orientation == 0 && intersection % 3 == 0)
          {
            temp_beliefs[intersection][orientation] = beliefs[intersection][3] * sensor_accuracy; //Should be motion model for moving back from a boundary and rotating which is different from driving to intersection
          }
          else if (orientation == 1 && intersection < 3)
          {
            temp_beliefs[intersection][orientation] = beliefs[intersection][0] * sensor_accuracy;
          }
          else if (orientation == 2 && intersection % 3 == 2)
          {
            temp_beliefs[intersection][orientation] = beliefs[intersection][1] * sensor_accuracy;
          }
          else if (orientation == 3 && intersection > 11)
          {
            temp_beliefs[intersection][orientation] = beliefs[intersection][2] * sensor_accuracy;
          }
          
          else 
          {
            temp_beliefs[intersection][orientation] = beliefs[intersection][orientation] * (100 - sensor_accuracy);
          }
          // Create a running sum of all belief values to use for normalizing
          normalizer += temp_beliefs[intersection][orientation];
        }
      }
    }

    // Motion step messed up, don't use it to inform probability
    else
    {
      for (int intersection = 0; intersection < 15; intersection++)
      {
        for (int orientation = 0; orientation < 4; orientation++)
        {
          temp_beliefs[intersection][orientation] = beliefs[intersection][orientation];
          normalizer += temp_beliefs[intersection][orientation];
        }
      }
    }
    

    /* At this point, the grid should be updated with the information regarding the last motion.
        This info is in the temp_beliefs. It needs to be copied onto the real beliefs array and 
        then normalized.

        We should also be able to determine if the bot has been localized.
    */
    for (int i =0; i < 15; i++)
    {
      for (int o = 0; o < 4; o++)
      {
        beliefs[i][o] = (temp_beliefs[i][o] / normalizer);
        if (beliefs[i][o] >= LOCALIZED_THRESHOLD) 
        {
          *(robot_x)= i % 3;
          *(robot_y)= i / 3;
          *(direction)= o;
          print_beliefs(beliefs);
          return 1;
        }
      }
    }
  }
  print_beliefs(beliefs);

  // The robot has not been localized
  *(robot_x)=-1;
  *(robot_y)=-1;
  *(direction)=-1;
  return(0);
}

int go_to_target(int robot_x, int robot_y, int direction, int target_x, int target_y)
{
 /*
  * This function is called once localization has been successful, it performs the actions required to take the robot
  * from its current location to the specified target location. 
  *
  * You have to write the code required to carry out this task - once again, you can use the function headers provided, or
  * write your own code to control the bot, but document your process carefully in the comments below so your TA can easily
  * understand how everything works.
  *
  * Your code should be able to determine if the robot has gotten lost (or if localization was incorrect), and your bot
  * should be able to recover.
  * 
  * Inputs - The robot's current location x,y (the intersection coordinates, not image pixel coordinates)
  *          The target's intersection location
  * 
  * Return values: 1 if successful (the bot reached its target destination), 0 otherwise
  */   

  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/
  
  intersection_adjust();

  while (robot_x > target_x)
  {
    while (direction != 3)
    {
      turn_at_intersection(0);
      direction += 1;
    }
    intersection_adjust();
    while (BT_read_colour_sensor(PORT_1) != 4)
    {
      find_street();
      drive_along_street();
    }
    robot_x += -1;
  }

  while (robot_x < target_x)
  {
    while (direction != 1)
    {
      if (direction > 1){
        turn_at_intersection(1);
        direction += -1;
      }
      else 
      {
        turn_at_intersection(0);
        direction += 1;
      }
    }
    intersection_adjust();
    while (BT_read_colour_sensor(PORT_1) != 4)
    {
      find_street();
      drive_along_street();
    }
    robot_x += 1;
  }

  intersection_adjust();

  while (robot_y > target_y)
  {
    while (direction != 0)
    {
      turn_at_intersection(1);
      direction += -1;
    }
    intersection_adjust();
    while (BT_read_colour_sensor(PORT_1) != 4)
    {
      find_street();
      drive_along_street();
    }
    robot_y += -1;
  }

  while (robot_y < target_y)
  {
    while (direction != 2)
    {
      if (direction < 2)
      {
        turn_at_intersection(0);
        direction += 1;
      }
      else 
      {
        turn_at_intersection(1);
        direction += -1;
      }
    }
    intersection_adjust();
    while (BT_read_colour_sensor(PORT_1) != 4)
    {
      find_street();
      drive_along_street();
    }
    robot_y += 1;
  }

  if (robot_x == target_x && robot_y == target_y) return 1;

  return(0);

}

void calibrate_sensor(void)
{
 /*
  * This function is called when the program is started with -1  -1 for the target location. 
  *
  * You DO NOT NEED TO IMPLEMENT ANYTHING HERE - but it is strongly recommended as good calibration will make sensor
  * readings more reliable and will make your code more resistent to changes in illumination, map quality, or battery
  * level.
  * 
  * The principle is - Your code should allow you to sample the different colours in the map, and store representative
  * values that will help you figure out what colours the sensor is reading given the current conditions.
  * 
  * Inputs - None
  * Return values - None - your code has to save the calibration information to a file, for later use (see in main())
  * 
  * How to do this part is up to you, but feel free to talk with your TA and instructor about it!
  */   

  /************************************************************************************************************************
   *   OIPTIONAL TO DO  -   Complete this function
   ***********************************************************************************************************************/
  fprintf(stderr,"Calibration function called!\n");  
}

int parse_map(unsigned char *map_img, int rx, int ry)
{
 /*
   This function takes an input image map array, and two integers that specify the image size.
   It attempts to parse this image into a representation of the map in the image. The size
   and resolution of the map image should not affect the parsing (i.e. you can make your own
   maps without worrying about the exact position of intersections, roads, buildings, etc.).

   However, this function requires:
   
   * White background for the image  [255 255 255]
   * Red borders around the map  [255 0 0]
   * Black roads  [0 0 0]
   * Yellow intersections  [255 255 0]
   * Buildings that are pure green [0 255 0], pure blue [0 0 255], or white [255 255 255]
   (any other colour values are ignored - so you can add markings if you like, those 
    will not affect parsing)

   The image must be a properly formated .ppm image, see readPPMimage below for details of
   the format. The GIMP image editor saves properly formatted .ppm images, as does the
   imagemagick image processing suite.
   
   The map representation is read into the map array, with each row in the array corrsponding
   to one intersection, in raster order, that is, for a map with k intersections along its width:
   
    (row index for the intersection)
    
    0     1     2    3 ......   k-1
    
    k    k+1   k+2  ........    
    
    Each row will then contain the colour values for buildings around the intersection 
    clockwise from top-left, that is
    
    
    top-left               top-right
            
            intersection
    
    bottom-left           bottom-right
    
    So, for the first intersection (at row 0 in the map array)
    map[0][0] <---- colour for the top-left building
    map[0][1] <---- colour for the top-right building
    map[0][2] <---- colour for the bottom-right building
    map[0][3] <---- colour for the bottom-left building
    
    Color values for map locations are defined as follows (this agrees with what the
    EV3 sensor returns in indexed-colour-reading mode):
    
    1 -  Black
    2 -  Blue
    3 -  Green
    4 -  Yellow
    5 -  Red
    6 -  White
    
    If you find a 0, that means you're trying to access an intersection that is not on the
    map! Also note that in practice, because of how the map is defined, you should find
    only Green, Blue, or White around a given intersection.
    
    The map size (the number of intersections along the horizontal and vertical directions) is
    updated and left in the global variables sx and sy.

    Feel free to create your own maps for testing (you'll have to print them to a reasonable
    size to use with your bot).
    
 */    
 
 int last3[3];
 int x,y;
 unsigned char R,G,B;
 int ix,iy;
 int bx,by,dx,dy,wx,wy;         // Intersection geometry parameters
 int tgl;
 int idx;
 
 ix=iy=0;       // Index to identify the current intersection
 
 // Determine the spacing and size of intersections in the map
 tgl=0;
 for (int i=0; i<rx; i++)
 {
  for (int j=0; j<ry; j++)
  {
   R=*(map_img+((i+(j*rx))*3));
   G=*(map_img+((i+(j*rx))*3)+1);
   B=*(map_img+((i+(j*rx))*3)+2);
   if (R==255&&G==255&&B==0)
   {
    // First intersection, top-left pixel. Scan right to find width and spacing
    bx=i;           // Anchor for intersection locations
    by=j;
    for (int k=i; k<rx; k++)        // Find width and horizontal distance to next intersection
    {
     R=*(map_img+((k+(by*rx))*3));
     G=*(map_img+((k+(by*rx))*3)+1);
     B=*(map_img+((k+(by*rx))*3)+2);
     if (tgl==0&&(R!=255||G!=255||B!=0))
     {
      tgl=1;
      wx=k-i;
     }
     if (tgl==1&&R==255&&G==255&&B==0)
     {
      tgl=2;
      dx=k-i;
     }
    }
    for (int k=j; k<ry; k++)        // Find height and vertical distance to next intersection
    {
     R=*(map_img+((bx+(k*rx))*3));
     G=*(map_img+((bx+(k*rx))*3)+1);
     B=*(map_img+((bx+(k*rx))*3)+2);
     if (tgl==2&&(R!=255||G!=255||B!=0))
     {
      tgl=3;
      wy=k-j;
     }
     if (tgl==3&&R==255&&G==255&&B==0)
     {
      tgl=4;
      dy=k-j;
     }
    }
    
    if (tgl!=4)
    {
     fprintf(stderr,"Unable to determine intersection geometry!\n");
     return(0);
    }
    else break;
   }
  }
  if (tgl==4) break;
 }
  fprintf(stderr,"Intersection parameters: base_x=%d, base_y=%d, width=%d, height=%d, horiz_distance=%d, vertical_distance=%d\n",bx,by,wx,wy,dx,dy);

  sx=0;
  for (int i=bx+(wx/2);i<rx;i+=dx)
  {
   R=*(map_img+((i+(by*rx))*3));
   G=*(map_img+((i+(by*rx))*3)+1);
   B=*(map_img+((i+(by*rx))*3)+2);
   if (R==255&&G==255&&B==0) sx++;
  }

  sy=0;
  for (int j=by+(wy/2);j<ry;j+=dy)
  {
   R=*(map_img+((bx+(j*rx))*3));
   G=*(map_img+((bx+(j*rx))*3)+1);
   B=*(map_img+((bx+(j*rx))*3)+2);
   if (R==255&&G==255&&B==0) sy++;
  }
  
  fprintf(stderr,"Map size: Number of horizontal intersections=%d, number of vertical intersections=%d\n",sx,sy);

  // Scan for building colours around each intersection
  idx=0;
  for (int j=0; j<sy; j++)
   for (int i=0; i<sx; i++)
   {
    x=bx+(i*dx)+(wx/2);
    y=by+(j*dy)+(wy/2);
    
    fprintf(stderr,"Intersection location: %d, %d\n",x,y);
    // Top-left
    x-=wx;
    y-=wy;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][0]=3;
    else if (R==0&&G==0&&B==255) map[idx][0]=2;
    else if (R==255&&G==255&&B==255) map[idx][0]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Top-Left RGB=%d,%d,%d\n",i,j,R,G,B);

    // Top-right
    x+=2*wx;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][1]=3;
    else if (R==0&&G==0&&B==255) map[idx][1]=2;
    else if (R==255&&G==255&&B==255) map[idx][1]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Top-Right RGB=%d,%d,%d\n",i,j,R,G,B);

    // Bottom-right
    y+=2*wy;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][2]=3;
    else if (R==0&&G==0&&B==255) map[idx][2]=2;
    else if (R==255&&G==255&&B==255) map[idx][2]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Bottom-Right RGB=%d,%d,%d\n",i,j,R,G,B);
    
    // Bottom-left
    x-=2*wx;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][3]=3;
    else if (R==0&&G==0&&B==255) map[idx][3]=2;
    else if (R==255&&G==255&&B==255) map[idx][3]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Bottom-Left RGB=%d,%d,%d\n",i,j,R,G,B);
    
    fprintf(stderr,"Colours for this intersection: %d, %d, %d, %d\n",map[idx][0],map[idx][1],map[idx][2],map[idx][3]);
    
    idx++;
   }

 return(1);  
}

unsigned char *readPPMimage(const char *filename, int *rx, int *ry)
{
 // Reads an image from a .ppm file. A .ppm file is a very simple image representation
 // format with a text header followed by the binary RGB data at 24bits per pixel.
 // The header has the following form:
 //
 // P6
 // # One or more comment lines preceded by '#'
 // 340 200
 // 255
 //
 // The first line 'P6' is the .ppm format identifier, this is followed by one or more
 // lines with comments, typically used to inidicate which program generated the
 // .ppm file.
 // After the comments, a line with two integer values specifies the image resolution
 // as number of pixels in x and number of pixels in y.
 // The final line of the header stores the maximum value for pixels in the image,
 // usually 255.
 // After this last header line, binary data stores the RGB values for each pixel
 // in row-major order. Each pixel requires 3 bytes ordered R, G, and B.
 //
 // NOTE: Windows file handling is rather crotchetty. You may have to change the
 //       way this file is accessed if the images are being corrupted on read
 //       on Windows.
 //

 FILE *f;
 unsigned char *im;
 char line[1024];
 int i;
 unsigned char *tmp;
 double *fRGB;

 im=NULL;
 f=fopen(filename,"rb+");
 if (f==NULL)
 {
  fprintf(stderr,"Unable to open file %s for reading, please check name and path\n",filename);
  return(NULL);
 }
 fgets(&line[0],1000,f);
 if (strcmp(&line[0],"P6\n")!=0)
 {
  fprintf(stderr,"Wrong file format, not a .ppm file or header end-of-line characters missing\n");
  fclose(f);
  return(NULL);
 }
 fprintf(stderr,"%s\n",line);
 // Skip over comments
 fgets(&line[0],511,f);
 while (line[0]=='#')
 {
  fprintf(stderr,"%s",line);
  fgets(&line[0],511,f);
 }
 sscanf(&line[0],"%d %d\n",rx,ry);                  // Read image size
 fprintf(stderr,"nx=%d, ny=%d\n\n",*rx,*ry);

 fgets(&line[0],9,f);  	                // Read the remaining header line
 fprintf(stderr,"%s\n",line);
 im=(unsigned char *)calloc((*rx)*(*ry)*3,sizeof(unsigned char));
 if (im==NULL)
 {
  fprintf(stderr,"Out of memory allocating space for image\n");
  fclose(f);
  return(NULL);
 }
 fread(im,(*rx)*(*ry)*3*sizeof(unsigned char),1,f);
 fclose(f);

 return(im);    
}