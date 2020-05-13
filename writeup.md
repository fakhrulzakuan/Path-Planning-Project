# CarND-Path-Planning-Project Write Up

## The car drives according to the speed limit.

How I do limit the speed is in `line 118`. I limit the maximum speed at 49.5 mph. 
```
 //Set maximum speed mph so we dont break the law
double maximum_ref_vel = 49.5;
```

I apply the limit between `line 256 to 260` 
```
//To avoid jerk and only limit to maximum speed that has been set earlier. 
ref_vel += 1.0;
if (ref_vel > maximum_ref_vel)
    {
    ref_vel = maximum_ref_vel;
    }
```

## Max Acceleration and Jerk are not Exceeded. 

I avoided the jerk by incrementing the speed by 1 mph for every message event. 

```
//To avoid jerk but also decelerates hard enough if the vehicle in front brakes suddenly. 

ref_vel -= 1.0;

// If there is vehicle in front of ego, the ego will match its speed. 
// Times 2.24 to convert to mph
if (ref_vel <= car_front_speed * 2.24)
    {
    ref_vel = car_front_speed * 2.24;
    }
```

## Car does not have collisions.

To ensure the ego car do not collide with other cars around. First we get the data from sensor fusion from `line 122 to 132`

```
for(int i = 0; i < sensor_fusion.size(); i++)
    {
        //Get info from sensor fusion. Note that the detected vehicle velocity is in m/s
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx*vx+vy*vy);
        double check_car_s = sensor_fusion[i][5];
        
        //Get ego vehicle current lane
        double check_car_d = sensor_fusion[i][6];
        int detected_car_lane = -1;
```

Then I assign the lane ID based on the cars D position in Frenet from `line 135 to 147` where lane = 0 is the far left lane. 

```
//Assign lane ID based on the d value. 0 is far left lane. 
if ((check_car_d < 4) && (check_car_d > 0))
    {
    detected_car_lane = 0;
    }

else if ((check_car_d < 8) && (check_car_d > 4))
    {
    detected_car_lane = 1;
    }
else
    {
    detected_car_lane = 2;
    }
```

Predict detected surrounding vehicles S position based on their velocity at `line 152`

```
check_car_s += ((double)prev_size*0.02*check_speed);
```

Now to trigger flag when there is vehicle at front same lane, left and right lane, from `line 159 to 176`
```
// Check vehicle in front
// First by checking the detected is in the same lane as ego, then check the vehicle is infront of ego instead of behind. 
// We also don't want to trigger flag when the detected vehicle is still too far away, so we only trigger if the detected vehicle is within 20 meter range. 
if ((detected_car_lane == lane) && (check_car_s > (car_s - 2)) && ((check_car_s - car_s) < 20))
    { 
    //Keep the distance between ego and detected vehicle infront
    distance_between= check_car_s - car_s;

    //Keep the relative speed of the detected vehicle infront
    car_front_speed = check_speed;
    car_front = true;
    }
//Check whether there are vehicles at left and right lane. 
else if ((lane - detected_car_lane == 1) && ((check_car_s - car_s) > -5) && ((check_car_s - car_s) < 20))
    { 
    car_left = true;
    }
else if ((lane - detected_car_lane == -1) && ((check_car_s - car_s) > -5) && ((check_car_s - car_s) < 20))
    {
        car_right = true;
    }
```

Now based onthe flag, we can plan the next behaviour of the ego vehicle on the highway. Should it stay in lane (Lane Keeping), change to left or right lane?
The behaviour planning is between `line 189 to 266`

To summarize:

There are 3 states that I observe, CAR_FRONT, CAR_LEFT, and CAR_RIGHT. 

```
IF CAR_FRONT flag is true. 
    Then the ego vehicle will starts to slow down, eventually will match the speed of the the vehicle in front. 

    At a same time I also check whether there is a car on left or right lane. 

    I also check the distance between the ego vehicle and the vehicle in front. 

    IF only the distance between them is far enough, in this case I set it at 15 meter. 
        Then it is allowed for the ego vehicle to change lane. 

        IF CAR_RIGHT is true and CAR_LEFT is false.
            Then it is safer to change to left lane. 
        
        IF CAR_LEFT is true and CAR_RIGHT is false.
            Then it is safer to change to right lane. 
        
        IF both CAR_LEFT and CAR_RIGHT are true.
            Then just stay in lane. 
        
        IF both CAR_LEFT and CAR_RIGHT are false.
            It is safe to change to either lane but I put priority to change to left lane. 

    ELSE, the ego vehicle will keeps slowing down until it is safe to change lane. 

ELSE, there is no vehicle in front, the ego vehicle will gain speed to the speed limit. 
```
# The car stays in its lane, except for the time between changing lanes.

To keep to ego vehicle to stay in vehicle as long as it moves, I applied the spline `line 335` after I manually added points for every 30 meters in front of the vehicle, from `line 311 to 313`

```
// In Frenet add evenly 30 m spaced points ahead of the starting reference. 

vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
```

The way points are added is also based on the D position, to ensure the smooth transition when changing lanes. 

# The car is able to change lanes

The ego vehicle has successfully changed lane, following trajectory created based on those 3 added points. 


















