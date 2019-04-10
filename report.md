# CarND-Path-Planning-Project

![Build Status](https://github.com/Aladdin95/CarND-Path-Planning-Project/blob/master/simulator.png?raw=true)


# how to generate paths.
all code are in file main.cpp

## first we get our state.
code from line 100 to 131.
variable front_close tells us whether there is a car in front of us and distance less than 30.
variable left_close tells us if there isn't a gap in the left lane or not.
and variable right_close tells us if there isn't a gap in the right lane or not.

min_left_s, min_right_s and min_front_s hold the distances between our car and the cars in front of it in the left lane, right lane and our lane respectivly.

in line 122 we define the gap length the car in front of us shouldn be more far than 30 and the car behind should be more far than 10 to keep the variables false.

## second we get all possible paths and choose the best depending on our state.

code from line 142 to 187.

### first we check if there is a car in front of us and too close to us.
then we check our lane.
- if we are in lane 0 then all possible choices is to decrease our speed or make a lane change to right.
we decide that if there is a gap in the right lane or not.
- the same apllies if we are in lane 2 only choices to change to the left lane or to keep lane but decrease out speed
- if we are in lane 1 then we have 3 choices if we have only one gap on the left or the right lane we do the same as before but if we have 2 gaps in the left and right lanes we choose the one that has bigger distance before the car in front of us.

### second if we dont have a car in front of us and very close.
then we check our lane.
- if we are in lane 0 we change to the right lane only if it has bigger distance before the car in front of us
- if we are in lane 2 we do the same
- if we are in lane 1 we choose the the bigger distance and change to it.

in line 187 we check if we dicided to change our lane we decrease the max velocity not to get high acceleration.

we decrease or increase the car velocity in lines 277, 278.
