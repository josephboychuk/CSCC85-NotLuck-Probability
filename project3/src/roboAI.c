/**************************************************************************
  CSC C85 - UTSC RoboSoccer AI core

  This file is where the actual planning is done and commands are sent
  to the robot.

  Please read all comments in this file, and add code where needed to
  implement your game playing logic. 

  Things to consider:

  - Plan - don't just react
  - Use the heading vectors!
  - Mind the noise (it's everywhere)
  - Try to predict what your oponent will do
  - Use feedback from the camera

  What your code should not do: 

  - Attack the opponent, or otherwise behave aggressively toward the
    oponent
  - Hog the ball (you can kick it, push it, or leave it alone)
  - Sit at the goal-line or inside the goal
  - Run completely out of bounds

  AI scaffold: Parker-Lee-Estrada, Summer 2013

  EV3 Version 2.0 - Updated Jul. 2022 - F. Estrada
***************************************************************************/

#include "roboAI.h"			// <--- Look at this header file!
// Goal angle threshold
#define ANG_ROTATE_TOWARDS 0.14
#define ANG_PINCERS 0.07
#define ANG_GOAL 0.14 // 8 degrees
// PID controller constants
#define K1 10
#define K2 1
#define K3 1
// Controls how fast to drive forward during move_to_target
#define C 4
extern int sx;              // Get access to the image size from the imageCapture module
extern int sy;
int laggy=0;

/**************************************************************
 * Display List Management
 * 
 * The display list head is kept as a pointer inside the A.I. 
 * data structure. Initially NULL (of course). It works like
 * any other linked list - anytime you add a graphical marker
 * it's added to the list, the imageCapture code loops over
 * the list and draws any items in there.
 * 
 * The list IS NOT CLEARED between frames (so you can display
 * things like motion paths that go over mutiple frames).
 * Your code will need to call clearDP() when you want this
 * list cleared.
 * 
 * ***********************************************************/
struct displayList *addPoint(struct displayList *head, int x, int y, double R, double G, double B)
{
  struct displayList *newNode;
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addPoint(): Out of memory!\n");
    return head;
  }
  newNode->type=0;
  newNode->x1=x;
  newNode->y1=y;
  newNode->x2=-1;
  newNode->y2=-1;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  
  newNode->next=head;
  return(newNode);
}

struct displayList *addLine(struct displayList *head, int x1, int y1, int x2, int y2, double R, double G, double B)
{
  struct displayList *newNode;
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addLine(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x1;
  newNode->y1=y1;
  newNode->x2=x2;
  newNode->y2=y2;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  return(newNode);  
}

struct displayList *addVector(struct displayList *head, int x1, int y1, double dx, double dy, int length, double R, double G, double B)
{
  struct displayList *newNode;
  double l;
  
  l=sqrt((dx*dx)+(dy*dy));
  dx=dx/l;
  dy=dy/l;
  
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addVector(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x1;
  newNode->y1=y1;
  newNode->x2=x1+(length*dx);
  newNode->y2=y1+(length*dy);
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  return(newNode);
}

struct displayList *addCross(struct displayList *head, int x, int y, int length, double R, double G, double B)
{
  struct displayList *newNode;
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addLine(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x-length;
  newNode->y1=y;
  newNode->x2=x+length;
  newNode->y2=y;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  head=newNode;

  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addLine(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x;
  newNode->y1=y-length;
  newNode->x2=x;
  newNode->y2=y+length;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  return(newNode);
}

struct displayList *clearDP(struct displayList *head)
{
  struct displayList *q;
  while(head)
  {
      q=head->next;
      free(head);
      head=q;
  }
  return(NULL);
}

/**************************************************************
 * End of Display List Management
 * ***********************************************************/

/*************************************************************
 * Blob identification and tracking
 * ***********************************************************/

struct blob *id_coloured_blob2(struct RoboAI *ai, struct blob *blobs, int col)
{
 /////////////////////////////////////////////////////////////////////////////
 // This function looks for and identifies a blob with the specified colour.
 // It uses the hue and saturation values computed for each blob and tries to
 // select the blob that is most like the expected colour (red, green, or blue)
 //
 // If you find that tracking of blobs is not working as well as you'd like,
 // you can try to improve the matching criteria used in this function.
 // Remember you also have access to shape data and orientation axes for blobs.
 //
 // Inputs: The robot's AI data structure, a list of blobs, and a colour target:
 // Colour parameter: 0 -> Blue bot
 //                   1 -> Red bot
 //                   2 -> Yellow ball
 // Returns: Pointer to the blob with the desired colour, or NULL if no such
 // 	     blob can be found.
 /////////////////////////////////////////////////////////////////////////////

 struct blob *p, *fnd;
 double vr_x,vr_y,maxfit,mincos,dp;
 double vb_x,vb_y,fit;
 double maxsize=0;
 double maxgray;
 int grayness;
 int i;
 static double Mh[4]={-1,-1,-1,-1};
 static double mx0,my0,mx1,my1,mx2,my2;
 FILE *f;
 
 // Import calibration data from file - this will contain the colour values selected by
 // the user in the U.I.
 if (Mh[0]==-1)
 {
  f=fopen("colours.dat","r");
  if (f!=NULL)
  {
   fread(&Mh[0],4*sizeof(double),1,f);
   fclose(f);
   mx0=cos(Mh[0]);
   my0=sin(Mh[0]);
   mx1=cos(Mh[1]);
   my1=sin(Mh[1]);
   mx2=cos(Mh[2]);
   my2=sin(Mh[2]);
  }
 }

 if (Mh[0]==-1)
 {
     fprintf(stderr,"roboAI.c :: id_coloured_blob2(): No colour calibration data, can not ID blobs. Please capture colour calibration data on the U.I. first\n");
     return NULL;
 }
 
 maxfit=.025;                                             // Minimum fitness threshold
 mincos=.90;                                              // Threshold on colour angle similarity
 maxgray=.25;                                             // Maximum allowed difference in colour
                                                          // to be considered gray-ish (as a percentage
                                                          // of intensity)

 // The reference colours here are in the HSV colourspace, we look at the hue component, which is a
 // defined within a colour-wheel that contains all possible colours. Hence, the hue component
 // is a value in [0 360] degrees, or [0 2*pi] radians, indicating the colour's location on the
 // colour wheel. If we want to detect a different colour, all we need to do is figure out its
 // location in the colour wheel and then set the angles below (in radians) to that colour's
 // angle within the wheel.
 // For reference: Red is at 0 degrees, Yellow is at 60 degrees, Green is at 120, and Blue at 240.

  // Agent IDs are as follows: 0 : blue bot,  1 : red bot, 2 : yellow ball
  if (col==0) {vr_x=mx0; vr_y=my0;}                                                    
  else if (col==1) {vr_x=mx1; vr_y=my1;}
  else if (col==2) {vr_x=mx2; vr_y=my2;}

 // In what follows, colours are represented by a unit-length vector in the direction of the
 // hue for that colour. Similarity between two colours (e.g. a reference above, and a pixel's
 // or blob's colour) is measured as the dot-product between the corresponding colour vectors.
 // If the dot product is 1 the colours are identical (their vectors perfectly aligned), 
 // from there, the dot product decreases as the colour vectors start to point in different
 // directions. Two colours that are opposite will result in a dot product of -1.
 
 p=blobs;
 while (p!=NULL)
 { 
  if (p->size>maxsize) maxsize=p->size;
  p=p->next;
 }

 p=blobs;
 fnd=NULL;
 while (p!=NULL)
 {
  // Normalization and range extension
  vb_x=cos(p->H);
  vb_y=sin(p->H);

  dp=(vb_x*vr_x)+(vb_y*vr_y);                                       // Dot product between the reference color vector, and the
                                                                    // blob's color vector.

  fit=dp*p->S*p->S*(p->size/maxsize);                               // <<< --- This is the critical matching criterion.
                                                                    // * THe dot product with the reference direction,
                                                                    // * Saturation squared
                                                                    // * And blob size (in pixels, not from bounding box)
                                                                    // You can try to fine tune this if you feel you can
                                                                    // improve tracking stability by changing this fitness
                                                                    // computation

  // Check for a gray-ish blob - they tend to give trouble
  grayness=0;
  if (fabs(p->R-p->G)/p->R<maxgray&&fabs(p->R-p->G)/p->G<maxgray&&fabs(p->R-p->B)/p->R<maxgray&&fabs(p->R-p->B)/p->B<maxgray&&\
      fabs(p->G-p->B)/p->G<maxgray&&fabs(p->G-p->B)/p->B<maxgray) grayness=1;
  
  if (fit>maxfit&&dp>mincos&&grayness==0)
  {
   fnd=p;
   maxfit=fit;
  }
  
  p=p->next;
 }

 return(fnd);
}

void track_agents(struct RoboAI *ai, struct blob *blobs)
{
 ////////////////////////////////////////////////////////////////////////
 // This function does the tracking of each agent in the field. It looks
 // for blobs that represent the bot, the ball, and our opponent (which
 // colour is assigned to each bot is determined by a command line
 // parameter).
 // It keeps track within the robot's AI data structure of multiple 
 // parameters related to each agent:
 // - Position
 // - Velocity vector. Not valid while rotating, but possibly valid
 //   while turning.
 // - Motion direction vector. Not valid
 //   while rotating - possibly valid while turning
 // - Heading direction - vector obtained from the blob shape, it is
 //   correct up to a factor of (-1) (i.e. it may point backward w.r.t.
 //   the direction your bot is facing). This vector remains valid
 //   under rotation.
 // - Pointers to the blob data structure for each agent
 //
 // This function will update the blob data structure with the velocity
 // and heading information from tracking. 
 //
 // NOTE: If a particular agent is not found, the corresponding field in
 //       the AI data structure (ai->st.ball, ai->st.self, ai->st.opp)
 //       will remain NULL. Make sure you check for this before you 
 //       try to access an agent's blob data! 
 //
 // In addition to this, if calibration data is available then this
 // function adjusts the Y location of the bot and the opponent to 
 // adjust for perspective projection error. See the handout on how
 // to perform the calibration process.
 //
 // This function receives a pointer to the robot's AI data structure,
 // and a list of blobs.
 //
 // You can change this function if you feel the tracking is not stable.
 // First, though, be sure to completely understand what it's doing.
 /////////////////////////////////////////////////////////////////////////

 struct blob *p;
 double mg,vx,vy,pink,doff,dmin,dmax,adj;
 
 // Reset ID flags and agent blob pointers
 ai->st.ballID=0;
 ai->st.selfID=0;
 ai->st.oppID=0;
 ai->st.ball=NULL;			// Be sure you check these are not NULL before
 ai->st.self=NULL;			// trying to access data for the ball/self/opponent!
 ai->st.opp=NULL;
 
 // Find the ball
 p=id_coloured_blob2(ai,blobs,2);
 if (p)
 {
  ai->st.ball=p;			// New pointer to ball
  ai->st.ballID=1;			// Set ID flag for ball (we found it!)
  ai->st.bvx=p->cx-ai->st.old_bcx;	// Update ball velocity in ai structure and blob structure
  ai->st.bvy=p->cy-ai->st.old_bcy;
  ai->st.ball->vx=ai->st.bvx;
  ai->st.ball->vy=ai->st.bvy;
  ai->st.bdx=p->dx;
  ai->st.bdy=p->dy;

  ai->st.old_bcx=p->cx; 		// Update old position for next frame's computation
  ai->st.old_bcy=p->cy;
  ai->st.ball->idtype=3;

  vx=ai->st.bvx;			// Compute motion direction (normalized motion vector)
  vy=ai->st.bvy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)			// Update heading vector if meaningful motion detected
  {
   vx/=mg;
   vy/=mg;
   ai->st.bmx=vx;
   ai->st.bmy=vy;
  }
  else
  {
    ai->st.bmx=0;
    ai->st.bmy=0;
  }
  ai->st.ball->mx=ai->st.bmx;
  ai->st.ball->my=ai->st.bmy;
 }
 else {
  ai->st.ball=NULL;
 }
 
 // ID our bot - the colour is set from commane line, 0=Blue, 1=Red
 p=id_coloured_blob2(ai,blobs,ai->st.botCol);
 if (p!=NULL&&p!=ai->st.ball)
 {
  ai->st.self=p;			// Update pointer to self-blob
  ai->st.selfID=1;
  ai->st.svx=p->cx-ai->st.old_scx;
  ai->st.svy=p->cy-ai->st.old_scy;
  ai->st.self->vx=ai->st.svx;
  ai->st.self->vy=ai->st.svy;
  ai->st.sdx=p->dx;
  ai->st.sdy=p->dy;

  vx=ai->st.svx;
  vy=ai->st.svy;
  mg=sqrt((vx*vx)+(vy*vy));
//  printf("--->    Track agents(): d=[%lf, %lf], [x,y]=[%3.3lf, %3.3lf], old=[%3.3lf, %3.3lf], v=[%2.3lf, %2.3lf], motion=[%2.3lf, %2.3lf]\n",ai->st.sdx,ai->st.sdy,ai->st.self->cx,ai->st.self->cy,ai->st.old_scx,ai->st.old_scy,vx,vy,vx/mg,vy/mg);
  if (mg>NOISE_VAR)
  {
   vx/=mg;
   vy/=mg;
   ai->st.smx=vx;
   ai->st.smy=vy;
  }
  else
  {
   ai->st.smx=0;
   ai->st.smy=0;
  }
  ai->st.self->mx=ai->st.smx;
  ai->st.self->my=ai->st.smy;
  ai->st.old_scx=p->cx; 
  ai->st.old_scy=p->cy;
  ai->st.self->idtype=1;
 }
 else ai->st.self=NULL;

 // ID our opponent - whatever colour is not botCol
 if (ai->st.botCol==0) p=id_coloured_blob2(ai,blobs,1);
 else p=id_coloured_blob2(ai,blobs,0);
 if (p!=NULL&&p!=ai->st.ball&&p!=ai->st.self)
 {
  ai->st.opp=p;	
  ai->st.oppID=1;
  ai->st.ovx=p->cx-ai->st.old_ocx;
  ai->st.ovy=p->cy-ai->st.old_ocy;
  ai->st.opp->vx=ai->st.ovx;
  ai->st.opp->vy=ai->st.ovy;
  ai->st.odx=p->dx;
  ai->st.ody=p->dy;

  ai->st.old_ocx=p->cx; 
  ai->st.old_ocy=p->cy;
  ai->st.opp->idtype=2;

  vx=ai->st.ovx;
  vy=ai->st.ovy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)
  {
   vx/=mg;
   vy/=mg;
   ai->st.omx=vx;
   ai->st.omy=vy;
  }
  else
  {
   ai->st.omx=0;
   ai->st.omy=0;
  }
  ai->st.opp->mx=ai->st.omx;
  ai->st.opp->my=ai->st.omy;
 }
 else ai->st.opp=NULL;

}

void id_bot(struct RoboAI *ai, struct blob *blobs)
{
 ///////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This routine calls track_agents() to identify the blobs corresponding to the
 // robots and the ball. It commands the bot to move forward slowly so heading
 // can be established from blob-tracking.
 //
 // NOTE 1: All heading estimates, velocity vectors, position, and orientation
 //         are noisy. Remember what you have learned about noise management.
 //
 // NOTE 2: Heading and velocity estimates are not valid while the robot is
 //         rotating in place (and the final heading vector is not valid either).
 //         To re-establish heading, forward/backward motion is needed.
 //
 // NOTE 3: However, you do have a reliable orientation vector within the blob
 //         data structures derived from blob shape. It points along the long
 //         side of the rectangular 'uniform' of your bot. It is valid at all
 //         times (even when rotating), but may be pointing backward and the
 //         pointing direction can change over time.
 //
 // You should *NOT* call this function during the game. This is only for the
 // initialization step. Calling this function during the game will result in
 // unpredictable behaviour since it will update the AI state.
 ///////////////////////////////////////////////////////////////////////////////
 
 struct blob *p;
 static double stepID=0;
 static double oldX,oldY;
 double frame_inc=1.0/5.0;
 double dist;
 
 track_agents(ai,blobs);		// Call the tracking function to find each agent

 BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 30);			// Start forward motion to establish heading
                                                // Will move for a few frames.
  
 if (ai->st.selfID==1&&ai->st.self!=NULL)
  fprintf(stderr,"Successfully identified self blob at (%f,%f)\n",ai->st.self->cx,ai->st.self->cy);
 if (ai->st.oppID==1&&ai->st.opp!=NULL)
  fprintf(stderr,"Successfully identified opponent blob at (%f,%f)\n",ai->st.opp->cx,ai->st.opp->cy);
 if (ai->st.ballID==1&&ai->st.ball!=NULL)
  fprintf(stderr,"Successfully identified ball blob at (%f,%f)\n",ai->st.ball->cx,ai->st.ball->cy);

 stepID+=frame_inc;
 if (stepID>=1&&ai->st.selfID==1)	// Stop after a suitable number of frames.
 {
  ai->st.state+=1;
  stepID=0;
  BT_all_stop(0);
 }
 else if (stepID>=1) stepID=0;

 // At each point, each agent currently in the field should have been identified.
 return;
}
/*********************************************************************************
 * End of blob ID and tracking code
 * ******************************************************************************/

/*********************************************************************************
 * Routine to initialize the AI 
 * *******************************************************************************/
int setupAI(int mode, int own_col, struct RoboAI *ai)
{
 /////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This sets up the initial AI for the robot. There are three different modes:
 //
 // SOCCER -> Complete AI, tries to win a soccer game against an opponent
 // PENALTY -> Score a goal (no goalie!)
 // CHASE -> Kick the ball and chase it around the field
 //
 // Each mode sets a different initial state (0, 100, 200). Hence, 
 // AI states for SOCCER will be 0 through 99
 // AI states for PENALTY will be 100 through 199
 // AI states for CHASE will be 200 through 299
 //
 // You will of course have to add code to the AI_main() routine to handle
 // each mode's states and do the right thing.
 //
 // Your bot should not become confused about what mode it started in!
 //////////////////////////////////////////////////////////////////////////////        

 switch (mode) {
 case AI_SOCCER:
	fprintf(stderr,"Standard Robo-Soccer mode requested\n");
        ai->st.state=0;		// <-- Set AI initial state to 0
        break;
 case AI_PENALTY:
// 	fprintf(stderr,"Penalty mode! let's kick it!\n");
	ai->st.state=100;	// <-- Set AI initial state to 100
        break;
 case AI_CHASE:
	fprintf(stderr,"Chasing the ball...\n");
	ai->st.state=200;	// <-- Set AI initial state to 200
        break;	
 default:
	fprintf(stderr, "AI mode %d is not implemented, setting mode to SOCCER\n", mode);
	ai->st.state=0;
	}

 BT_all_stop(0);			// Stop bot,
 ai->runAI = AI_main;		// and initialize all remaining AI data
 ai->calibrate = AI_calibrate;
 ai->st.ball=NULL;
 ai->st.self=NULL;
 ai->st.opp=NULL;
 ai->st.side=0;
 ai->st.botCol=own_col;
 ai->st.old_bcx=0;
 ai->st.old_bcy=0;
 ai->st.old_scx=0;
 ai->st.old_scy=0;
 ai->st.old_ocx=0;
 ai->st.old_ocy=0;
 ai->st.bvx=0;
 ai->st.bvy=0;
 ai->st.svx=0;
 ai->st.svy=0;
 ai->st.ovx=0;
 ai->st.ovy=0;
 ai->st.sdx=0;
 ai->st.sdy=0;
 ai->st.odx=0;
 ai->st.ody=0;
 ai->st.bdx=0;
 ai->st.bdy=0;
 ai->st.selfID=0;
 ai->st.oppID=0;
 ai->st.ballID=0;
 ai->DPhead=NULL;
 fprintf(stderr,"Initialized!\n");

 return(1);
}

void AI_calibrate(struct RoboAI *ai, struct blob *blobs)
{
 // Basic colour blob tracking loop for calibration of vertical offset
 // See the handout for the sequence of steps needed to achieve calibration.
 // The code here just makes sure the image processing loop is constantly
 // tracking the bots while they're placed in the locations required
 // to do the calibration (i.e. you DON'T need to add anything more
 // in this function).
 track_agents(ai,blobs);
}


/**************************************************************************
 * AI state machine - this is where you will implement your soccer
 * playing logic
 * ************************************************************************/
void AI_main(struct RoboAI *ai, struct blob *blobs, void *state)
{
 /*************************************************************************
  This is your robot's state machine.
  
  It is called by the imageCapture code *once* per frame. And it *must not*
  enter a loop or wait for visual events, since no visual refresh will happen
  until this call returns!
  
  Therefore. Everything you do in here must be based on the states in your
  AI and the actions the robot will perform must be started or stopped 
  depending on *state transitions*. 

  E.g. If your robot is currently standing still, with state = 03, and
   your AI determines it should start moving forward and transition to
   state 4. Then what you must do is 
   - send a command to start forward motion at the desired speed
   - update the robot's state
   - return
  
  I can not emphasize this enough. Unless this call returns, no image
  processing will occur, no new information will be processed, and your
  bot will be stuck on its last action/state.

  You will be working with a state-based AI. You are free to determine
  how many states there will be, what each state will represent, and
  what actions the robot will perform based on the state as well as the
  state transitions.

  You must *FULLY* document your state representation in the report

  The first two states for each more are already defined:
  State 0,100,200 - Before robot ID has taken place (this state is the initial
            	    state, or is the result of pressing 'r' to reset the AI)
  State 1,101,201 - State after robot ID has taken place. At this point the AI
            	    knows where the robot is, as well as where the opponent and
            	    ball are (if visible on the playfield)

  Relevant UI keyboard commands:
  'r' - reset the AI. Will set AI state to zero and re-initialize the AI
	data structure.
  't' - Toggle the AI routine (i.e. start/stop calls to AI_main() ).
  'o' - Robot immediate all-stop! - do not allow your EV3 to get damaged!

   IMPORTANT NOTE: There are TWO sources of information about the 
                   location/parameters of each agent
                   1) The 'blob' data structures from the imageCapture module
                   2) The values in the 'ai' data structure.
                      The 'blob' data is incomplete and changes frame to frame
                      The 'ai' data should be more robust and stable
                      BUT in order for the 'ai' data to be updated, you
                      must call the function 'track_agents()' in your code
                      after eah frame!
                      
    DATA STRUCTURE ORGANIZATION:

    'RoboAI' data structure 'ai'
         \    \    \   \--- calibrate()  (pointer to AI_clibrate() )
          \    \    \--- runAI()  (pointer to the function AI_main() )
           \    \------ Display List head pointer 
            \_________ 'ai_data' data structure 'st'
                         \  \   \------- AI state variable and other flags
                          \  \---------- pointers to 3 'blob' data structures
                           \             (one per agent)
                            \------------ parameters for the 3 agents
                              
  ** Do not change the behaviour of the robot ID routine **
 **************************************************************************/

  static double ux,uy,len,mmx,mmy,tx,ty,x1,y1,x2,y2;
  double angDif;
  char line[1024];
  static int count=0;
  static double old_dx=0, old_dy=0;
      
  /************************************************************
   * Standard initialization routine for starter code,
   * from state **0 performs agent detection and initializes
   * directions, motion vectors, and locations
   * Triggered by toggling the AI on.
   * - Modified now (not in starter code!) to have local
   *   but STATIC data structures to keep track of robot
   *   parameters across frames (blob parameters change
   *   frame to frame, memoryless).
   ************************************************************/
 if (ai->st.state==0||ai->st.state==100||ai->st.state==200)  	// Initial set up - find own, ball, and opponent blobs
 {
  // Carry out self id process.
  fprintf(stderr,"Initial state, self-id in progress...\n");
  
  id_bot(ai,blobs);
  if ((ai->st.state%100)!=0)	  // The id_bot() routine will change the AI state to initial state + 1
  {				                 // if robot identification is successful.
      
   if (ai->st.self->cx>=512) ai->st.side=1; else ai->st.side=0;         // This sets the side the bot thinks as its own side 0->left, 1->right
   BT_all_stop(0);
   
   fprintf(stderr,"Self-ID complete. Current position: (%f,%f), current heading: [%f, %f], blob direction=[%f, %f], AI state=%d\n",ai->st.self->cx,ai->st.self->cy,ai->st.smx,ai->st.smy,ai->st.sdx,ai->st.sdy,ai->st.state);
   
   if (ai->st.self!=NULL)
   {
       // This checks that the motion vector and the blob direction vector
       // are pointing in the same direction. If they are not (the dot product
       // is less than 0) it inverts the blob direction vector so it points
       // in the same direction as the motion vector.
       if (((ai->st.smx*ai->st.sdx)+(ai->st.smy*ai->st.sdy))<0)
       {
           ai->st.self->dx*=-1.0;
           ai->st.self->dy*=-1.0;
           ai->st.sdx*=-1;
           ai->st.sdy*=-1;
       }
       old_dx=ai->st.sdx;
       old_dy=ai->st.sdy;
   }
  
   if (ai->st.opp!=NULL)
   {
       // Checks motion vector and blob direction for opponent. See above.
       if (((ai->st.omx*ai->st.odx)+(ai->st.omy*ai->st.ody))<0)
       {
           ai->st.opp->dx*=-1;
           ai->st.opp->dy*=-1;
           ai->st.odx*=-1;
           ai->st.ody*=-1;
       }       
   }

         
  }
  
  // Initialize BotInfo structures
   
 }
 else
 {
  /****************************************************************************
   TO DO:
   You will need to replace this 'catch-all' code with actual program logic to
   implement your bot's state-based AI.

   After id_bot() has successfully completed its work, the state should be
   1 - if the bot is in SOCCER mode
   101 - if the bot is in PENALTY mode
   201 - if the bot is in CHASE mode

   Your AI code needs to handle these states and their associated state
   transitions which will determine the robot's behaviour for each mode.

   Please note that in this function you should add appropriate functions below
   to handle each state's processing, and the code here should mostly deal with
   state transitions and with calling the appropriate function based on what
   the bot is supposed to be doing.
  *****************************************************************************/
  // TODO cleanup local variables
  double old_scx = ai->st.old_scx;
  double old_scy = ai->st.old_scy;
  // TODO remove when refactoring the state functions out
  double face_ball_dx, face_ball_dy, bx, by;
  // Save the previous heading for correction purposes
  ai->st.old_sdx = ai->st.sdx;
  ai->st.old_sdy = ai->st.sdy;
  track_agents(ai,blobs);
  // Correct the heading vector. Invariant: previous heading vector is correct
  // If the sign of the current heading is opposite of the previous, then
  // it must be flipped. We can check that their dot product is negative.
  // Due to noise, the previous and current may be close to directly up and down
  // but the x components may be the same sign. However, the magnitude of x is
  // close to 0 so the product of the y components which has magnitude close to
  // 1 should overpower the x product and still result in a negative dot product
  // fprintf(stderr, "Correcting heading: prev [%f, %f], curr [%f, %f], ", ai->st.old_sdx, ai->st.old_sdy, ai->st.sdx, ai->st.sdy);
  if (ai->st.old_sdx*ai->st.sdx + ai->st.old_sdy*ai->st.sdy < 0)
  {
    if (ai->st.self != NULL)
    {
      ai->st.self->dx*=-1.0;
      ai->st.self->dy*=-1.0;
    }
    ai->st.sdx*=-1;
    ai->st.sdy*=-1;
  }
  // fprintf(stderr, "correction [%f, %f]\n", ai->st.sdx, ai->st.sdy);

  // fprintf(stderr, "");
  // TODO REMOVE. THIS JUST FORCES US INTO 1 CONDITIONAL FOR TESTING
  // ai->st.state = 401; // impossible state so we can move bot manually and print the heading correction

  if (ai->st.state == 101)
  {
    fprintf(stderr, "[101] rotate towards ball\n");
    // Rotate towards ball
    // TODO recdeclare when moving this funcrtion out of AI_main
    // double face_ball_dx, face_ball_dy;
    get_ball_xy(ai, blobs, &bx, &by);
    // TODO SELF NULL CHECK
    face_ball_dx = bx - ai->st.self->cx;
    face_ball_dy = by - ai->st.self->cy;
    double ang = signed_rotation(ai->st.sdx, ai->st.sdy, face_ball_dx, face_ball_dy);
    ai->DPhead = clearDP(ai->DPhead);
    ai->DPhead = addVector(ai->DPhead, ai->st.self->cx, ai->st.self->cy, ai->st.sdx, ai->st.sdy, 100, 0, 255.0, 0);
    rotate(ai, blobs, ang, 30.0);
    fprintf(stderr, "raw [%f, %f], correct [%f, %f]\n", ai->st.sdx, ai->st.sdy, ai->st.sdx, ai->st.sdy);
    fprintf(stderr, "goal atan2 heading: %f, robot: %f\n", atan2(face_ball_dx, face_ball_dy) * 180.0 / PI, atan2(ai->st.sdx, ai->st.sdy) * 180.0 / PI);
    // Facing the ball, within 8 degrees

    if (fabs(ang) <= ANG_ROTATE_TOWARDS)
    {
      BT_all_stop(1);
      ai->st.state = 102;
    }
    

  }
  else if (ai->st.state == 102)
  {
    ai->DPhead = clearDP(ai->DPhead);
    get_ball_xy(ai, blobs, &bx, &by);
    move_to_target(ai, blobs, bx, by, 100);
    // TODO state transition
    double old_dist = dist(old_scx, old_scy, bx, by);
    // TODO self NULL check
    double curr_dist = dist(ai->st.self->cx, ai->st.self->cy, bx, by);
    fprintf(stderr, "[102] move towards ball    Current dist from ball:%f    Old dist:%f\n", curr_dist, old_dist);
    // OLD boundary logic
    // if (ai->st.self->cx < 100.0 || ai->st.self->cy < 100.0 || ai->st.self->cx > (sx - 100.0) || ai->st.self->cy > (sy - 100.0))
    // {
    //   fprintf(stderr, "Too close to boundary. Stopping!\n");
    //   BT_all_stop(1);
    //   ai->st.state = 101;
    // }
    // TODO if going to drive into boundary, back up?
    // for now, just commented it out since driving into the boundary shouldnt
    // really happen in penalty kick

    // // Not facing ball / moved away from the ball
    // else if (fabs(curr_dist - dist(old_scx, old_scy, bx, by)) > 50.0)
    // {
    //   fprintf(stderr, "[102] Not facing ball. Going back to 101\n");
    //   BT_all_stop(1);
    //   ai->st.state = 101;
    // }
    // when close to ball; note should account for distance from center of
    // uniform to the actual pincers
    // NOTE: since move to target seems to be working well, skip the slower
    // rotation adjustment to see how it works. right now the slower rotation adjustment
    // over rotates
    if (curr_dist <= 250.0)
    {
      ai->st.state = 104;   // for now, skip aim pincers go to position ball in pincers
    }
  }
  // TODO: re-enable transition to this in state 102 if needed. otherwise remove
  // this state from the FSM
  // aim pincers at ball
  else if (ai->st.state == 103)
  {
    fprintf(stderr, "[103] aim pincers at ball\n");
    // Rotate towards ball
    // TODO redeclare when separating this out from AI_main. right now
    // we are redeclaring variables!
    // double correct_dx, correct_dy, face_ball_dx, face_ball_dy;
    get_ball_xy(ai, blobs, &bx, &by);
    // TODO SELF NULL CHECK
    face_ball_dx = bx - ai->st.self->cx;
    face_ball_dy = by - ai->st.self->cy;
    ai->DPhead = clearDP(ai->DPhead);
    double curr_dist = dist(ai->st.self->cx, ai->st.self->cy, bx, by);
    double ang = signed_rotation(ai->st.sdx, ai->st.sdy, face_ball_dx, face_ball_dy);
    // Facing the ball, within some degrees
    if (fabs(ang) <= ANG_PINCERS)
    {
      BT_all_stop(1);
      ai->st.state = 104;
    }
    else if (curr_dist > 300.0)
    {
      BT_all_stop(1);
      ai->st.state = 102;
    }
    else
    {
      rotate(ai, blobs, ang, 10);
      fprintf(stderr, "goal atan2 heading: %f, robot: %f\n", atan2(face_ball_dx, face_ball_dy) * 180.0 / PI, atan2(ai->st.sdx, ai->st.sdy) * 180.0 / PI);
    }
  }
  // position ball in pincers
  else if (ai->st.state == 104)
  {
    ai->DPhead = clearDP(ai->DPhead);
    get_ball_xy(ai, blobs, &bx, &by);
    move_to_target(ai, blobs, bx, by, 30);
    double old_dist = dist(old_scx, old_scy, bx, by);
    // TODO SELF NULL CHECK
    double curr_dist = dist(ai->st.self->cx, ai->st.self->cy, bx, by);
    fprintf(stderr, "[104] positioning ball in pincers    Current dist from ball:%f    Old dist:%f\n", curr_dist, old_dist);
    // Ball is within range
    if (ball_in_pincers(ai, blobs))
    {
      // if moving slowly, robot stops when ball is just outside pincers so
      // let it keep driving forward
      // BT_all_stop(1);
      ai->st.state = 105;
    }
    // TODO factor in distance from center of rectangle to pincers
    // if robot moved far away from the ball, retry
    else if (curr_dist > 300.0)
    {
      ai->st.state = 101;
    }
  }
  // aim ball at goal
  else if (ai->st.state == 105)
  {
    fprintf(stderr, "[105] aiming ball at goal\n");
    double face_goal_dx, face_goal_dy, ang;
    double goal_x, goal_y;
    get_ball_xy(ai, blobs, &bx, &by);
    // TODO SELF NULL CHECK
    goal_x = sx * (1.0 - ai->st.side);
    goal_y = sy / 2.0;
    face_goal_dx = goal_x - ai->st.self->cx;
    face_goal_dy = goal_y - ai->st.self->cy;
    ai->DPhead = clearDP(ai->DPhead);
    // plot the goal position (red cross)
    // plot the path to the goal (magenta line)
    ai->DPhead = addCross(ai->DPhead, goal_x, goal_y, 75, 255, 0, 0);
    ai->DPhead = addLine(ai->DPhead, goal_x, goal_y, ai->st.self->cx, ai->st.self->cy, 255, 0, 255);
    ang = signed_rotation(ai->st.sdx, ai->st.sdy, face_goal_dx, face_goal_dy);
    // if ball fell out of pincers, get it back
    if (ball_in_pincers(ai, blobs) == 0)
    {
      // TODO stop rotation?
      // skipping aim pincers since it seems to over rotate atm
      ai->st.state = 104;
    }
    // if ball is too far
    else if (dist(bx, by, ai->st.self->cx, ai->st.self->cy) > 300.0)
    {
      // TODO stop rotation?
      ai->st.state = 101;
    }
    // if we're facing the goal (and ball in pincers), kick
    else if (fabs(ang) <= ANG_GOAL)
    {
      BT_all_stop(1);
      ai->st.state = 106;
    }
    else
    {
      // TODO try moving forward and turning instead of rotating in place so
      // the ball hopefully stays inside the pincers
      rotate(ai, blobs, ang, 15.0);
      fprintf(stderr, "goal atan2 heading: %f, robot: %f\n", atan2(face_goal_dx, face_goal_dy) * 180.0 / PI, atan2(ai->st.sdx, ai->st.sdy) * 180.0 / PI);
    }
  }
  // kick the ball
  else if (ai->st.state == 106)
  {
    kick(ai, blobs);
    // if the ball moved, we kicked it (probably)
    if (ball_in_pincers(ai, blobs) == 0)
    {
      ai->st.state = 107;
    }
  }
  // done
  else if (ai->st.state == 107)
  {
    BT_all_stop(0);
    fprintf(stderr, "[108] Done\n");
  }

  if (ai->st.state == 201)
  {
    fprintf(stderr, "[201] rotate towards ball\n");
    // Rotate towards ball
    // TODO recdeclare when moving this funcrtion out of AI_main
    // double correct_dx, correct_dy, face_ball_dx, face_ball_dy;
    get_ball_xy(ai, blobs, &bx, &by);
    face_ball_dx = bx - ai->st.self->cx;
    face_ball_dy = by - ai->st.self->cy;
    double ang = signed_rotation(ai->st.sdx, ai->st.sdy, face_ball_dx, face_ball_dy);
    ai->DPhead = clearDP(ai->DPhead);
    ai->DPhead = addVector(ai->DPhead, ai->st.self->cx, ai->st.self->cy, ai->st.sdx, ai->st.sdy, 100, 0, 255.0, 0);
    rotate(ai, blobs, ang, 30.0);
    fprintf(stderr, "raw [%f, %f], correct [%f, %f]\n", ai->st.sdx, ai->st.sdy, ai->st.sdx, ai->st.sdy);
    fprintf(stderr, "goal atan2 heading: %f, robot: %f\n", atan2(face_ball_dx, face_ball_dy) * 180.0 / PI, atan2(ai->st.sdx, ai->st.sdy) * 180.0 / PI);
    // Facing the ball, within 8 degrees

    if (fabs(ang) <= 0.21)
    {
      BT_all_stop(1);
      ai->st.state = 202;
    }
    

  }
  else if (ai->st.state == 202)
  {
    ai->DPhead = clearDP(ai->DPhead);
    get_ball_xy(ai, blobs, &bx, &by);
    move_to_target(ai, blobs, bx, by, 100);
    // TODO state transition
    double old_dist = dist(old_scx, old_scy, bx, by);
    double curr_dist = dist(ai->st.self->cx, ai->st.self->cy, bx, by);
    fprintf(stderr, "[202] move towards ball    Current dist from ball:%f    Old dist:%f\n", curr_dist, old_dist);
    // if too close to boundary
    // TODO also check top and left bounds
    if (ai->st.self->cx < 100.0 || ai->st.self->cy < 100.0 || ai->st.self->cx > (sx - 100.0) || ai->st.self->cy > (sy - 100.0))
    {
      fprintf(stderr, "Too close to boundary. Stopping!\n");
      BT_all_stop(1);
      ai->st.state = 201;
    }
    // Not facing ball / moved away from the ball
    else if (fabs(curr_dist - dist(old_scx, old_scy, bx, by)) > 50.0)
    {
      fprintf(stderr, "[202] Not facing ball. Going back to 201\n");
      BT_all_stop(1);
      ai->st.state = 201;
    }
    // when close to ball
    // TODO need to factor in distance from middle of rectangle to pincers
    // so "close" is actually not close to 0
    else if (curr_dist <= 250.0)
    {
      ai->st.state = 203;
    }
  }
  // aim pincers at ball
  else if (ai->st.state == 203)
  {
    fprintf(stderr, "[203] aim pincers at ball\n");
    // Rotate towards ball
    // TODO redeclare when separating this out from AI_main. right now
    // we are redeclaring variables!
    // double correct_dx, correct_dy, face_ball_dx, face_ball_dy;
    get_ball_xy(ai, blobs, &bx, &by);
    face_ball_dx = bx - ai->st.self->cx;
    face_ball_dy = by - ai->st.self->cy;
    ai->DPhead = clearDP(ai->DPhead);
    double curr_dist = dist(ai->st.self->cx, ai->st.self->cy, bx, by);
    double ang = signed_rotation(ai->st.sdx, ai->st.sdy, face_ball_dx, face_ball_dy);
    rotate(ai, blobs, ang, 10);
    fprintf(stderr, "goal atan2 heading: %f, robot: %f\n", atan2(face_ball_dx, face_ball_dy) * 180.0 / PI, atan2(ai->st.sdx, ai->st.sdy) * 180.0 / PI);
    // Facing the ball, within a 8 degrees
    if (fabs(ang) <= ANG_PINCERS)
    {
      BT_all_stop(1);
      ai->st.state = 204;
    }
    else if (curr_dist > 300.0)
    {
      BT_all_stop(1);
      ai->st.state = 202;
    }
  }
  // position ball in pincers
  else if (ai->st.state == 204)
  {
    ai->DPhead = clearDP(ai->DPhead);
    get_ball_xy(ai, blobs, &bx, &by);
    move_to_target(ai, blobs, bx, by, 30);
    // TODO state transition
    double old_dist = dist(old_scx, old_scy, bx, by);
    double curr_dist = dist(ai->st.self->cx, ai->st.self->cy, bx, by);
    fprintf(stderr, "[204] positioning ball in pincers    Current dist from ball:%f    Old dist:%f\n", curr_dist, old_dist);
    // Ball is within range
    if (ball_in_pincers(ai, blobs))
    {
      ai->st.state = 205;
    }
    // TODO factor in distance from center of rectangle to pincers
    else if (curr_dist > 300.0)
    {
      ai->st.state = 201;
    }
  }
  // kick the ball
  else if (ai->st.state == 205)
  {
    kick(ai, blobs);
    // if the ball moved, we kicked it (probably)
    if (ball_in_pincers(ai, blobs) == 0)
    {
      ai->st.state = 201;
    }
  }
 }

}

/**********************************************************************************
 TO DO:

 Add the rest of your game playing logic below. Create appropriate functions to
 handle different states (be sure to name the states/functions in a meaningful
 way), and do any processing required in the space below.

 AI_main() should *NOT* do any heavy lifting. It should only call appropriate
 functions based on the current AI state.

 You will lose marks if AI_main() is cluttered with code that doesn't belong
 there.
**********************************************************************************/

// TODO REMOVE
void test(struct RoboAI *ai, struct blob *blobs)
{
  double angle = atan2(ai->st.sdx, ai->st.sdy);
  fprintf(stderr, "dx %f\tdy %f\tradians %f\tdegrees %f\n", ai->st.sdx, ai->st.sdy, angle, angle *180.0 / PI);
  // fprintf(stderr, "Press enter to continue\n");
  // getchar();
}

void get_ball_xy(struct RoboAI *ai, struct blob *blobs, double *x, double *y)
{
  if (ai->st.ball == NULL)
  {
    (*x) = ai->st.old_bcx + ai->st.bvx;
    (*y) = ai->st.old_bcy + ai->st.bvy;
    return;
  }
  (*x) = ai->st.ball->cx;
  (*y) = ai->st.ball->cy;
  
}

void get_self_xy(struct RoboAI *ai, struct blob *blobs, double *x, double *y)
{
  if (ai->st.self == NULL)
  {
    (*x) = ai->st.old_scx + ai->st.svx;
    (*y) = ai->st.old_scy + ai->st.svy;
  }
  else
  {
    (*x) = ai->st.self->cx;
    (*y) = ai->st.self->cy;
  }
}

void predict_ball_xy(struct RoboAI *ai, struct blob *blobs, double *x, double *y)
{
  if (ai->st.ball == NULL)
  {
    (*x) = ai->st.old_bcx + 2*ai->st.bvx;
    (*y) = ai->st.old_bcy + 2*ai->st.bvy;
    return;
  }
  (*x) = ai->st.ball->cx + ai->st.bvx;
  (*y) = ai->st.ball->cy + ai->st.bvy;
}

void predict_self_xy(struct RoboAI *ai, struct blob *blobs, double *x, double *y)
{
  if (ai->st.self == NULL)
  {
    (*x) = ai->st.old_scx + 2*ai->st.svx;
    (*y) = ai->st.old_scy + 2*ai->st.svy;
    return;
  }
  (*x) = ai->st.self->cx + ai->st.svx;
  (*y) = ai->st.self->cy + ai->st.svy;
}

double dist(double x1, double y1, double x2, double y2)
{
  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

double signed_rotation(double sx, double sy, double tx, double ty)
{ 
  return atan2(sx*ty-sy*tx, sx*tx+sy*ty);
}


// TODO REMOVE
// int is_left(struct RoboAI *ai, int prev_left, enum Motion motion)
// {
//   int left = prev_left;
//   if (motion == GENERAL)
//   {
//     double tol = 2.44;  // about 140 degrees
//     if (fabs(atan2(ai->st.old_sdx, ai->st.old_sdy) - atan2(ai->st.sdx, ai->st.sdy)) >= tol)
//     {
//       left = prev_left ? 0 : 1;
//     }
//     fprintf(stderr, "old left %d, new left %d\n", prev_left, left);
//   }
//   // if (motion == DIRECTIONAL)
//   // {
//   //   if (ai->st.smx < 0)
//   //   {
//   //     left = 1;
//   //   }
//   //   else
//   //   {
//   //     left = 0;
//   //   }
//   // }
//   if (ai->st.sdx < 0)
//   {
//     ai->st.sdx *= -1.0;
//     ai->st.sdy *= -1.0;
//     left = 1;
//   }
//   return left;
// }

void rotate(struct RoboAI *ai, struct blob *blobs, double direction, double power)
{
  double adj_power = fmin(fmax(power * fabs(direction), 10.0), 100.0);
  if (direction > 0)
  {
    BT_turn(LEFT_MOTOR, -adj_power, RIGHT_MOTOR, adj_power);
  }
  else
  {
    BT_turn(LEFT_MOTOR, adj_power, RIGHT_MOTOR, -adj_power);
  }
}

void temp_rotate(struct RoboAI *ai, struct blob *blobs, char power)
{
  double goal_x = sx * (1.0 - ai->st.side);
  double goal_y = sy / 2.0;
  if (ai->st.self->cx > goal_x)
  {
    if (ai->st.self->cy < goal_y)
    {
      BT_turn(LEFT_MOTOR, power, RIGHT_MOTOR, -power);
    } else
    {
      BT_turn(LEFT_MOTOR, -power, RIGHT_MOTOR, power);
    }
  }
  else
  {
    if (ai->st.self->cy < goal_y)
    {
      BT_turn(LEFT_MOTOR, -power, RIGHT_MOTOR, power);
    } else
    {
      BT_turn(LEFT_MOTOR, power, RIGHT_MOTOR, -power);
    }
  }
}

void move_to_ball(struct RoboAI *ai, struct blob *blobs)
{
  // determine distance from balll
  // further distance is more power
  double bx, by;
  get_ball_xy(ai, blobs, &bx, &by);
  double curr_dist = dist(ai->st.self->cx, ai->st.self->cy, bx, by);
  double power = curr_dist/(sqrt(sx^2 + sy^2));
  BT_turn(LEFT_MOTOR, power, RIGHT_MOTOR, power);
 
  
}

void move_to_target(struct RoboAI *ai, struct blob *blobs, double tx, double ty, double max_power)
{
  double path_x, path_y, e, de, ie;
  path_x = tx - ai->st.self->cx;
  path_y = ty - ai->st.self->cy;
  e = signed_rotation(ai->st.smx, ai->st.smy, path_x, path_y);
  // int iter = 0;
  // ie = 0;
  // while(ai->st.self->e_hist[iter] !=  DELIM && iter < 100)
  // {
  //     ie += ai->st.self->e_hist[iter];
  //     iter++;
  // }
  // ie += e;
  // // At this point the current iter value should also be the index of previous e
  // de = e - ai->st.self->e_hist[iter];
  // double u = K1*e + K2 *de + K3 *ie;
  // Now add this e to e_hist
  // add_to_e_hist(ai, blobs, e);

  double u = K1*e;
  // Now u from our PID controller is found, use that to inform motion
  // C is power constant
  double map_diagonal  = dist(0, 0, sx, sy);
  double power = fmin(fabs(C*100*dist(0, 0, path_x, path_y)/map_diagonal), max_power-fabs(u));
  fprintf(stderr, "move_to_target: u %f, power %f\n", u, power);
  if (u > 0)
  {
    BT_turn(LEFT_MOTOR, power, RIGHT_MOTOR, power+u);
  }
  else
  {
    BT_turn(LEFT_MOTOR, power-u, RIGHT_MOTOR, power);
  }

}

void move_away_from_obstacles(struct RoboAI *ai, struct blob *blobs)
{
  // Move away from obstacles by determining the direction that
  //  obstacles are in and trying to move opposite that
}

int ball_in_pincers(struct RoboAI *ai, struct blob *blobs)
{
  // read multiple times
  const int N = 7;
  int count = 0;
  // majority is black blue or white then it's inside
  for (int i=0; i<N; i++)
  {
    int colour = BT_read_colour_sensor(COLOUR_SENSOR);
    if (colour == 1 || colour == 2 || colour == 6)
    {
      count++;
    }
  }
  return count > 3;
}

void kick(struct RoboAI *ai, struct blob *blobs)
{
  BT_drive(RIGHT_MOTOR, LEFT_MOTOR, 100);
  // add a small delay so we drive forward before kicking
  BT_timed_motor_port_start_v2(KICK_MOTOR, 1, 250);
  BT_timed_motor_port_start_v2(KICK_MOTOR, -100, 75);
  BT_timed_motor_port_start_v2(KICK_MOTOR, 20, 500);
  BT_all_stop(0);
}
