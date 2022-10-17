%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CSC C85 - Fundamentals of Robotics and Automated Systems
% UTSC - Fall 2021
%
% Starter code (c) F. Estrada, August 2021
% 
% Simple Controllers
%
%  Suppose we have managed to land a nice all-terrain
% rover on Mars - we're very proud of it, it can 
% actually go pretty fast and we plan to use it to
% drive around collecting dust (lifted by the front
% wheels and sucked into mass spectrometers for
% analysis). 
%
%  In order for the rover to accomplish its mission,
% it's essential that it drive around Mars at a 
% constant speed. We need a constant speed because
% the soil analysis uses density and particle size
% as part of the process of determining soil
% composition, and these are affected by driving 
% speed.
%
% Your task here is to implement a
% very simple controller that will allow the rover
% to achieve a speed that is as near to constant as
% possible, in the presence of external forces that
% affect the rover's driving.
%
% Your task is to:
%
% a) Understand the problem at hand and the forces
%    that affect how the rover drives
% b) Implement a simple, carefully calibrated PID
%    controller to quickly and smoothly correct for
%    any deviations from the target speed, so as to
%    keep the rover moving at a near-constant pace.
% c) Test your controller under different conditions
%    and see how it behaves.
%
% []=Driving_around_MARS(Xc, Yc, Tgt_speed, Rover_mass, delta_t, max_torque, Kp, Ki, Kd)
%
% - (Xc, Yc) - initial rover location on the map, don't bother with pixels, these
%              are values in 0,1 such that:
%
%              (0,0) -----------------------------------------
%                |             THE MAP                       |
%                |                                           |
%                |                                           |
%                |                                           |
%                |                                           |
%                ------------------------------------------(1,1)
%               
%   INPUT PARMS:
% - Tgt_speed - target rover speed in m/s
% - Rover_mass - self explanatory. In kg
% - delta_t - maximum change in rover direction per unit of time, in radians
% - max_torque - maximum torque produced by the rover's engine assembly, also
%                used for braking.
% - Kp, Ki, Kd - Constants for the proportional, integral and differential
%                terms in your PID controller. You'll need to tune these!
%
%  Now for a bit of physics (which I just looked up, highschool was a long
%  time ago! let me know if you find an error in my setup below!)
%
%    - Given your control input U in [-1,1], the torque requested on the
%      wheels of the rover is:
%
%      t=U*max_torque;      % Torque is in units of N.m (Newton meters)
%
%    - Given t, we find the force acting on the rover (either to accelerate
%      or brake) by dividing the torque by the radius of the wheels.
%      The Mars sojourner rover had a wheel diameter of 13 cm, let's go with
%      that:
%
%      F=t/.065;        % Radius in meters, so Force is now in Newtons (kg*m/s^2)
%
%      ** CAVEAT: Torque is not infinite! the amount of force put out by the
%                 engine decreases very fast as we approach the rover's top
%                 speed of 25m/s. You can't accelerate endlessly!
%
%    - Acceleration is obtained by Newton's 2nd law :)
%
%      a=F/Rover_mass;  % Mass is in kg, which leaves acceleration in m/s^2
%
%      ** HOWEVER - Note that the rover is moving in 3D, so we actually need
%                   an acceleration vector. 
%                   The above equations are scalar, we will use vector forms
%                   for everything below - Force will be a vector in 3D,
%                   yielding a 3D vector for acceleration, which gives a
%                   3D vector for velocity, which gives a 3D vector for 
%                   position over time.
%
%    - External forces - Obtained from the rover's mass and the gravity on
%                        Mars surface - The rover's weight causes a force
%                        always pointing down. We'll figure out what 
%                        the contribution of this force is in the direction
%                        of motion to determine the required acceleration
%
%      The basic computation looks like so (in scalar form)
%
%      Ext=Rover_mass*Mars_gravity;     
%
%      Rover mass is in kg, Mars gravity is 3.721 m/s^2, which gives units
%      of m*kg/m^2 -> Newtons again! We should always make sure units are
%      consistent in our operations, this gives us some certainty we're
%      doing the right thing.
%
%      ** Final notes : Your control input U does NOT take effect instantly. Like any
%      ** real system there is lag. The simulation will account for that.
%
%      ** Because velocity is a vector, your target speed is in terms of the
%      ** magnitude of the velocity vector.
%
%      ** The simulation reports the RMS (Root-Mean-Square) error of the
%      ** Rover's actual velocity compared to the target, over the last
%      ** 25 simulation steps. Your goal is to get this as close to 0
%      ** as possible for a sustained amount of time. 
%      ** RMS < .5 is pretty ok, .3 < RMS < .5 is good, less than that is
%      ** very good!
%
%      ** Your controller should work for different settings (rover mass,
%      ** max torque, etc.) and on different locations (more or less hilly)
%
%    You don't need to use any of that! (your controller only cares about
%    applying a control input in [-1,1]). I'm just showing you so you know
%    what the simulation is doing.
%
%    ***** THE SCALE OF THE ROVER'S MOTION IS UNREALISTIC - NOT TO SCALE ******
%    ***** Mars is a BIG planet, if we made it proportional, we'd have to *****
%    ***** wait for hours or days to see the rover move on this map! **********
%    ***** I've set the arbitrary scale so a velocity of 10m/s is equivalent **
%    ***** to the rover moving one pixel in map image distance each frame *****
%
% The elevation map is provided by NASA (Public Domain)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function []=Driving_around_MARS_v_sol(Xc, Yc, Tgt_speed, Rover_mass, delta_t, max_torque, Kp, Ki, Kd)

pkg load image;             %%% UNCOMMENT THIS FOR OCTAVE - Octave is doofus and requires this line... arghh!

close all;

if (Xc<0||Xc>1||Yc<0||Yc>1)
    fprintf(2,'Initial position must be in [0,1] for both X and Y\n');
    return;
end;

if (Tgt_speed<0||Tgt_speed>20)
    fprintf(2,'Target speed must be greater than 0 and no more than 20m/s (72Km/h)\n');
    return;
end;

if (Rover_mass<10||Rover_mass>100)
    fprintf(2,'The mass should be between 10 and 100 kg., Sojourner was about 11 kg.\n');
    return;
end;

if (max_torque<.065||max_torque>2.5)
    fprintf(2,'Max torque should be between .065 and 2.5 N*m\n');
    return;
end;

if (delta_t<0||delta_t>pi/8)
    fprintf(2,'Maximum turning angle must be less than pi/8 radians per simulation step\n');
    return;
end;

% Read the elevation map and convert it to a grayscale
% image where brightness = elevation
map_big=double(imread('mars_elevation.jpg'));
map_big=mean(map_big,3)/255;
gbl=[0.06136 	0.24477 	0.38774 	0.24477 	0.06136];0
map_big=imfilter(map_big,gbl,'replicate');
map_big=imfilter(map_big,gbl','replicate');

Window_Size=512;            % Size of subwindow to display, change this if you want to
                            % see a larger or smaller section of the map (limited by the
                            % edges of the map itself)

% Convert initial coordinates to pixel coordinates
Xc=max(1,min(size(map_big,2),round(Xc*size(map_big,2))));
Yc=max(1,min(size(map_big,1),round(Yc*size(map_big,1))));

v=[1 0 0];                             % Initial rover velocity in m/s - I just chose this!
cur_U=0;
X=[Xc Yc map_big(round(Yc),round(Xc)) v];
H=[0];
Us=[0];
Vs=[v];
old_dang=0;

%%%%%%%%%% YOU CAN ADD ANY VARIABLES YOU MAY NEED BETWEEN THIS LINE... %%%%%%%%%%%%%%%%%

old_err=Tgt_speed-norm(v);
% store the most recent integral at the start. each column has the last 100
% errors at that point in time, with the last error at the top of the column
int_err=zeros(100);              % Integral over last 100 steps

%%%%%%%%%% ... AND THIS LINE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

while(1)        %% Main simulation loop

 % Crop a region around the rover with the desired size
 whs=round(Window_Size/2);
 y1=max(1,round(Yc-whs));
 y2=min(size(map_big,1),round(Yc+whs));
 x1=max(1,round(Xc-whs));
 x2=min(size(map_big,2),round(Xc+whs));
 map=map_big(y1:y2,x1:x2);

 figure(1);clf;imagesc(map);axis image;colormap(jet);title('Currently driving here');hold on;

 % Plot driving locations
 Xt=X(:,1:2);
 Xt(:,1)=Xt(:,1)-Xc+whs;
 Xt(:,2)=Xt(:,2)-Yc+whs;
 id1=find(Xt(:,1)>1);
 id2=find(Xt(:,1)<Window_Size);
 id3=find(Xt(:,2)>1);
 id4=find(Xt(:,2)<Window_Size);
 id=intersect(id4,intersect(id3,intersect(id2,id1)));
 plot(Xt(id,1),Xt(id,2),'.','markersize',15,'color',[0 0 0.01]);
 plot(Xt(id,1),Xt(id,2),'.','markersize',14,'color',[1 0 0]);

 % Get height at current location
 h1=map_big(round(Yc),round(Xc));
 
 % Get next location given current motion direction (which is scaled by current velovity)
 x2=((Xc+(v(1)/10)));
 y2=((Yc+(v(2)/10)));
 if (round(x2)<1||round(x2)>size(map_big,2)||round(y2)<1||round(y2)>size(map_big,1))
     fprintf(2,'The rover left the map!\n');
     return;
 end;
 h2=map_big(round(y2),round(x2));
 % Need a motion direction vector to project forces onto
 dr=[x2-Xc y2-Yc h2-h1];
 dr=dr/norm(dr);

 %%%%%%%%%%%%%%%% DO NOT CHANGE ANY CODE ABOVE THIS LINE %%%%%%%%%%%%%%%%%%%%%
 
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 % TO DO: Implement your PID computation below, your PID must update a
 % variable called U, you need to figure out how to compute error,
 % the derivative of error over time, and the integral of error over
 % time. You *CAN* add your own variables as needed.
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
 % PID controller is Kp * error e (the difference between reference and
 % measured state variable)
 %      + Kd * change in error over time (derivative)
 %      + Ki * accumulated error over time (integral)
 % the state variable we care about is speed
 e=Tgt_speed-norm(v);
 % change in time is from previous step to current step
 d_e=e-old_err;
 % accumulated error
 i_e=sum(int_err(:,1))+e;
 U=Kp*e + Kd*d_e + Ki*i_e;       % Replace this with a computation based on your PID controller. You can
 % add variables to this script as needed
 
 % Please add in the line below the tuning settings that yield a good controller!
 % Kp= 0.3       , Ki= 0.0002         , Kd= 0.08          
 
 % don't forget to update the old error and accumulated error
 old_err = e;
 % newest integral added to front, so discard the last one
 int_err(:, 2:end) = int_err(:, 1:end-1);
 % most recent error is at the top of a column, so discard the last one
 int_err(:, 1) = [e, int_err(1:end-1, 1)']';
  
 %%%%%%%%%%%%%%%%%%  DO NOT CHANGE ANY CODE BELOW THIS LINE %%%%%%%%%%%%%%%%%%%%%
 
 % External force is the weight of the rover, pointing straight down
 Ext=Rover_mass*3.721*[0 0 -1];

 % Force exerted by the rover's engine - starting with the torque and conversion to linear force
 % in the direction of motion.
 tdf=1-(1/(1+exp(-5*(norm(v)-25))));
 F=tdf*max_torque*cur_U/.065;
 
 % The torque applies directly along the direction of motion (and because the location update ensures the rover
 % remains on the surface of the planet, it should be ok to just apply this along whatever motion direction
 % we obtained above without worrying the rover will float or go underground)
 %
 % So - F applies directly along motion direction dr
 % Ext is projected onto dr to determine the amount of force exerted by the rover's weight 
 %   along the current direction of motion. If the motion is horizontal, this is zero.
 %
 % Update forces, acceleration, velocity, and position:
 totalF=(F*dr)+((Ext*dr')*dr);
 a=totalF/Rover_mass;
 v=v+a;
 Xc=x2;
 Yc=y2;
 
 % Control input update is laggy and limited to [-1.1]
 U=min(1,max(-1,U));
 if (cur_U~=U)
     if (cur_U>U)
         cur_U=max(U,cur_U-.25);
     else
         cur_U=min(U,cur_U+.25);
     end;
 end;
 
 % Update historical data
 X(end+1,:)=[x2 y2 h2 v];
 Us(end+1)=U;
 Vs(end+1,:)=v;
 H(end+1)=h2-h1;

 % This is biased to keep turning the same direction... 
 if (old_dang<0)
  dang=((2*rand)-1.25)*delta_t;
 else
  dang=((2*rand)-.75)*delta_t;
 end;
 old_dang=dang;

 % Apply a small rotation to the direction of motion
 v=[cos(dang) -sin(dang)  0
    sin(dang) cos(dang)   0
      0           0       1]*v';
 v=v';
 
 h0=[];
 figure(2);clf;
 V=sum(Vs.^2,2);
 V=V.^.5;
 h0(1)=plot(V(max(1,size(Vs,1)-100):end),'.-','linewidth',1.5,'color',[.2 .5 1]);title('Velocity plot');grid on;
 
 if (length(V)>25)
  fprintf(2,'RMS over past 25 steps: %f\n',sqrt(mean((V(end-25:end)-Tgt_speed).^2)));
 end;
 
end;

