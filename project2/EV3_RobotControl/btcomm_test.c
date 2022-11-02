/* EV3 API
 *  Copyright (C) 2018-2019 Francisco Estrada and Lioudmila Tishkina
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

// Initial testing of a bare-bones BlueTooth communication
// library for the EV3 - Thank you Lego for changing everything
// from the NXT to the EV3!

#include "btcomm.h"
#include "stdbool.h"

const char* colour (int index) {
  const char* col;
  switch (index)
  {
    case 1:
      col = "Black";
      break;
    case 2:
      col = "Blue";
      break;
    case 3:
      col = "Green";
      break;
    case 4:
      col = "Yellow";
      break;
    case 5:
      col = "Red";
      break;
    case 6:
      col = "White";
      break;
    case 7:
      col = "Brown";
      break;
    default:
      col = "NO COLOUR";
  }
  return col;
}

void colour_tone(int index, int tone[1][3]) {
  // switch (index)
  // {
  //   case 1:
  //     // col = "Black"; 
  //     break;
  //   case 2:
  //     // col = "Blue";
  //     break;
  //   case 3:
  //     // col = "Green";
  //     break;
  //   case 4:
  //     // col = "Yellow";
  //     break;
  //   case 5:
  //     // col = "Red";
  //     break;
  //   case 6:
  //     // col = "White";
  //     break;
  //   case 7:
  //     // col = "Brown";
  //     break;
  //   default:
  //     // col = "NO COLOUR";
  // }
  tone[0][0] = 50 + index*250;
  tone[0][1] = 750;
  tone[0][2] = 30;
}


// Adjusts the wheels forward for the intersection
void intersection_adjust() {
  BT_timed_motor_port_start_v2(MOTOR_B, -8, 650);
  BT_timed_motor_port_start_v2(MOTOR_A, -8, 650);
  BT_timed_motor_port_start_v2(MOTOR_B, -8, 650);
  BT_timed_motor_port_start_v2(MOTOR_A, -8, 650);
  sleep(1);
}

//Rotates rotation number of degrees and stores a reading in the given array
int rotate_and_read(int rotation, int RGB[3]) {

  BT_timed_motor_port_start(MOTOR_B, -20, 0, rotation * 52 / 3, 0);
  BT_timed_motor_port_start(MOTOR_A, 20, 0, rotation * 52 / 3, 0);
  sleep(rotation/30);
  BT_read_colour_sensor_RGB(PORT_1, RGB);
  printf("The colour being read is (R, G, B) (%d, %d, %d) \n", RGB[0], RGB[1], RGB[2]);
  int coloridx = BT_read_colour_sensor(PORT_1);
  printf("The Bot considers this to be %s \n\n", colour(coloridx));
  return coloridx;
}

void offroad_adjust(int zag) {
  BT_all_stop(1);
  bool offroad = true;
  while (offroad)
  {
    BT_turn(MOTOR_B, -10 * zag, MOTOR_A, 10 * zag);
    if (BT_read_colour_sensor(PORT_1) == 1) offroad = false;
  }
}

int main(int argc, char *argv[]) {
  char test_msg[8] = {0x06, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x01};
  char reply[1024];
  int tone_data[50][3];

  // Reset tone data information
  for (int i = 0; i < 50; i++) {
    tone_data[i][0] = -1;
    tone_data[i][1] = -1;
    tone_data[i][2] = -1;
  }

  memset(&reply[0], 0, 1024);

// just uncomment your bot's hex key to compile for your bot, and comment the
// other ones out.
#ifndef HEXKEY
#define HEXKEY "00:16:53:55:DB:D3"  // <--- SET UP YOUR EV3's HEX ID here
#define FULL_POWER 100
#endif

  if (BT_open(HEXKEY) == -1)
  {
  	fprintf(stderr, "failed to connect to ev3\n");
  	BT_close();
  	return -1;
  }
  
  // Stores the colour value being read
  int RGB[3];
  int colour_read; // ************** TO BE REPLACED, CURRENTLY FOR INDEXED FUNCTION ************
  // Variable to indicate state / movement option
  int option = 0;
  // Variable to change adjustment direction when offroad;
  int zag = 1;
  // Indicates that the bot can stop navigating
  bool end_run = false;
  
  bool did_stop = false;
  
  int starting_angle = BT_read_gyro_sensor(PORT_2);
  
  while (abs(starting_angle - BT_read_gyro_sensor(PORT_2)) < 90)
  {
    BT_turn(MOTOR_B, -8, MOTOR_A, 8);
  }

  BT_all_stop(1);

  int second_angle = BT_read_gyro_sensor(PORT_2);

  fprintf(stderr, "The first angle is %d and the second angle is %d\n", starting_angle, second_angle);

;

  

  // while (!end_run){

  //   // Do some movement based on the given option
    
  //   // Normal Drive
  //   if (option == 0) {
  //     BT_turn(MOTOR_B, -12, MOTOR_A, -12);
  //   } 

  //   // At Red Boundary
  //   else if (option == 1)
  //   {
  //     printf("Found Boundary!\n");
  //     BT_all_stop(1);
  //     bool reversing = true;
  //     while (reversing)
  //     {
  //       BT_turn(MOTOR_B, 10, MOTOR_A, 10);
  //       if (BT_read_colour_sensor(PORT_1) == 4) {
  //         reversing = false;
  //         BT_all_stop(0);
  //       }
  //     }
  //     intersection_adjust();
  //     rotate_and_read(90, RGB);

  //     did_stop = true;
  //   }
    

  //   //At Intersection
  //   else if (option == 2)
  //   {
  //     if (did_stop)
  //     {
  //       end_run = true;
  //       break;
  //     }
  //     BT_all_stop(1);
  //     intersection_adjust();
  //     offroad_adjust(zag);
  //     // zag = zag * -1;
  //     int coloridx = 0;
  //     int tone[1][3] = {0, 0, 0};
  //     coloridx = rotate_and_read(45, RGB);
  //     colour_tone(coloridx, tone);
  //     BT_play_tone_sequence(tone);
  //     coloridx = rotate_and_read(90, RGB);
  //     colour_tone(coloridx, tone);
  //     BT_play_tone_sequence(tone);
  //     coloridx = rotate_and_read(90, RGB);
  //     colour_tone(coloridx, tone);
  //     BT_play_tone_sequence(tone);
  //     coloridx = rotate_and_read(90, RGB);
  //     colour_tone(coloridx, tone);
  //     BT_play_tone_sequence(tone);
  //     rotate_and_read(45, RGB);
  //     //rotate_and_read(90, RGB);
  //   }
    
    
  //   //Read colour, decide next option
  //   colour_read = BT_read_colour_sensor(PORT_1);

  //   // Detected Intersection
  //   if (colour_read == 4) {
  //     option = 2;
  //   }
  //   // Detected Barrier
  //   else if (colour_read == 5)
  //   {
  //     option = 1;
  //   }
  //   else if (colour_read == 1)
  //   {
  //     option = 0;
  //   }

  //   //Off road, readjust
  //   else if (colour_read == 2 || colour_read == 3 || colour_read == 6) 
  //   {
  //     printf("Offroad, readjust! \n");
  //     offroad_adjust(zag);
  //     option = 0;
  //   }

  // }

  BT_close();
  fprintf(stderr, "Done!\n");
}
