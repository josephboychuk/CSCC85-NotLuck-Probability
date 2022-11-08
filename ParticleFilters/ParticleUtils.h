/*
 
 CSC C85 - Fundamentals of Robotics and Automated Systems

 Particle filter utilities and image handling.

 You *DO NOT* need to modify this file, but you
 should be familiar with the functions here and
 their descriptions. There will be used in
 your particle filter implementation.

 Coded by F.J.E. May 2012
 Updated by F.J.E., Oct 2021
*/

#ifndef __ParticleUtils_header

#define __PArticleUtils_header

// General use libs.
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<time.h>
#include<string.h>

// Structure definitions:
struct particle{
 // A particle is characterized by its (x,y) position
 // in image coordinates with (0,0) the top-left corner
 // and (size_x-1, size_y-1) the bottom-right corner
 // as well as an angle of motion.
 // The angle of motion is measured CCW from
 // vertical so that
 // 0 degrees is 12
 // 90 degrees is 9
 // 180 degrees is 6
 // 270 degrees in 3
 double x;
 double y;
 double theta;
 double measureD[16];		// Array to hold sonar
				// distance measurements
				// or 'ground truth'
				// measurements.
 double prob;			// Belief Bel(p_i) for this particle
 struct particle *next;		// So we can make a list
};

// Particle filter utilities

// The hit() function returns true if a particle has
// hit a wall or obstacle in the map. 
// *p is a pointer to a particle
// *map is a pointer to the map image
// (sx, sy) is the size of the image in pixels.
int hit(struct particle *p, unsigned char *map, int sx, int sy);

// The move() function takes a particle p and simulates
// its motion in the direction 'theta' by 'dist' units.
// However note that motion is noisy, so the actual
// distance traveled is not exactly 'dist'
// Also, the move function simulates noise in motion
// direction, you will see the robot's direction change
// slowly and randomly. This is a more accurate
// simulation of how robot motion works.
// *p is a pointer to a particle
// dist is the distance to move in pixel units. 
//   * DO NOT MOVE by large amounts, or your particles
//     will walk through walls, which robots don't do.
void move(struct particle *p, double dist);

// The sonar_measurement() function uses the sonar
// to determine the location of walls if p represents
// a robot. The distance measurements obtained are
// noisy of course, since we know sonars are noisy.
// Requires a pointer to the map image as well as the
// image size in pixels.
// *p is a pointer to a particle
// *map is a pointer to the map image
// (sx, sy) is the size of the image in pixels
// Measurements are stored in the particle's 'measureD[]' array
void sonar_measurement(struct particle *p, unsigned char *map, int sx, int sy);

// The ground_truth() function determines from the
// map the *actual* exact distance to walls and
// obstacles. This is used to determine for each
// particle, what a robot with the particle's
// position would 'sense' given the map and a
// perfect sensor. ground_truth() measurements
// are *not* noisy.
// Requires a pointer to the map image as well as the
// image size in pixels.
// *p is a pointer to a particle
// *map is a pointer to the map image
// (sx,sy) is the size of the image in pixels
// Ground truth measurements are stored in the 'measureD[]' array
void ground_truth(struct particle *p, unsigned char *map, int sx, int sy);

// This function gives you a randomly initialized robot
// Since particles are hypothetical robots, this is also used
// to create particles!
// The bot is at a random location in the map, with an initially
// random motion direction. The location is chosen so
// as not to be too close to walls. 
// *map is a pointer to the map image
// (sx,sy) is the size of the map in pixels
struct particle *initRobot(unsigned char *map, int sx, int sy);		

// Helpful function to evaliate a Gaussian PDF with zero mean
// and specified sigma. 
double GaussEval(double x, double sigma);

// Helpful function to generate Gaussian distributed random
// numbers with a given mu and sigma
double GaussianNoise(double mu, double sigma);

// Handy function for deleting a linked list of particles
void deleteList(struct particle *l);

// Image display and OpenGL initialization functions
// You probably won't need to use any of these
unsigned char *readPPMimage(const char *filename, int *sx, int *sy);
void renderFrame(unsigned char *map, unsigned char *map_b, int sx, int sy, struct particle *robot, struct particle *list);

#endif

