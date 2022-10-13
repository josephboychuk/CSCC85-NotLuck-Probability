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

  tone_data[0][0] = 262;
  tone_data[0][1] = 250;
  tone_data[0][2] = 1;
  tone_data[1][0] = 330;
  tone_data[1][1] = 250;
  tone_data[1][2] = 25;
  tone_data[2][0] = 392;
  tone_data[2][1] = 250;
  tone_data[2][2] = 50;
  tone_data[3][0] = 523;
  tone_data[3][1] = 250;
  tone_data[3][2] = 63;

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

  // name must not contain spaces or special characters
  // max name length is 12 characters
  //BT_setEV3name("CoolGuy");

  //BT_play_tone_sequence(tone_data);
  
  int colour_sensor_values[3];

  BT_read_colour_sensor_RGB(PORT_1, colour_sensor_values);

  bool barrier = false;
  while (!barrier){
    BT_drive(MOTOR_B, MOTOR_A, -15);
    if (BT_read_colour_sensor(PORT_1) == 4) barrier = true;

  }
  BT_all_stop(1);


  BT_timed_motor_port_start_v2(MOTOR_B, -15, 500);
  BT_timed_motor_port_start_v2(MOTOR_A, -15, 500);
  
  sleep(2);

  BT_timed_motor_port_start(MOTOR_B, -20, 0, 750, 0);
  BT_timed_motor_port_start(MOTOR_A, 20, 0, 750, 0);
  sleep(2);
  printf("The colour being read is (R, G, B) (%d, %d, %d) \n", colour_sensor_values[0], colour_sensor_values[1], colour_sensor_values[2]);
  printf("The Bot considers this to be %s \n\n", colour(BT_read_colour_sensor(PORT_1)));
  
  BT_timed_motor_port_start(MOTOR_B, -20, 0, 1500, 0);
  BT_timed_motor_port_start(MOTOR_A, 20, 0, 1500, 0);
  sleep(2);
  printf("The colour being read is (R, G, B) (%d, %d, %d) \n", colour_sensor_values[0], colour_sensor_values[1], colour_sensor_values[2]);
  printf("The Bot considers this to be %s \n\n", colour(BT_read_colour_sensor(PORT_1)));

  BT_timed_motor_port_start(MOTOR_B, -20, 0, 1500, 0);
  BT_timed_motor_port_start(MOTOR_A, 20, 0, 1500, 0);
  sleep(2);
  printf("The colour being read is (R, G, B) (%d, %d, %d) \n", colour_sensor_values[0], colour_sensor_values[1], colour_sensor_values[2]);
  printf("The Bot considers this to be %s \n\n", colour(BT_read_colour_sensor(PORT_1)));

  BT_timed_motor_port_start(MOTOR_B, -20, 0, 1500, 0);
  BT_timed_motor_port_start(MOTOR_A, 20, 0, 1500, 0);
  sleep(2);
  printf("The colour being read is (R, G, B) (%d, %d, %d) \n", colour_sensor_values[0], colour_sensor_values[1], colour_sensor_values[2]);
  printf("The Bot considers this to be %s \n\n", colour(BT_read_colour_sensor(PORT_1)));

  BT_timed_motor_port_start(MOTOR_B, -20, 0, 750, 0);
  BT_timed_motor_port_start(MOTOR_A, 20, 0, 750, 0);
  sleep(2);



  BT_all_stop(0);

  BT_close();
  fprintf(stderr, "Done!\n");
}
