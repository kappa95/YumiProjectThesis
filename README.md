# YumiProjectThesis

Main points to develop in this project:

* Bimanual robot interaction
* Adaptive picking of test tubes

### Bimanual robot interaction

Yumi has 2 robotic arms. So we could create an algorithm that makes the two arms interact dynamically,
modifying their trajectory in space and time (for example, if one arm is slower
in reading a particular barcode, the other arm can adapt its trajectory to it)
in order to maximize output.
What if we can modify the action of each arm in space and time,
for example if one arm is taking too long to scan the barcode,
the other proceeds to picking up the following tube while the first places the tube it had in hand. 
What if the target position is closer to the left arm or the right one instead of the other?
Can the robot decide if it is faster putting the tubes with the arm closer
to the output rack hole positions?

#### Steps:

- [x] Find a way to command the 2 arms together
- [ ] Defining the constraints in which we plan the trajectory and choose the planner
that guarantees an unique solution for the position
- [ ] Defining the hypothesis under which the arm should decide a particular trajectory instead of another → change of the speed if the constraints are hard that decide an unique trajectory
- [ ] In the case we don’t want to fix an unique trajectory, find the constraints
 and planner for having “reasonable” and not strange trajectories
- [ ] Write the algorithm for choosing the solutions

### Project Demo part

An initial demo with fixed input/output racks --> working with points and constraints

#### Barcode-Camera Side:
 
- [x] Find a way to use the Yumi's Camera
- [x] Find a way to decode the image for Barcode Scanning
- [x] Created a Program for publish barcode on a topic
- [x] Created a Program for managing the barcodes (Barcode Manager) --> Tested
- [ ] To test the Feedback between the BarcodeManager and the motion

#### Motion Side:
- [x] Picking Motion  --> To Test
- [x] Scanning Motion  --> To Test
- [x] Interaction with Barcode Manager -> To Test
- [ ] Buffer Motion (WIP)  --> Probably not necessary
- [ ] Pick from Buffer  --> Probably not necessary
- [ ] Placing
- [ ] Repeat