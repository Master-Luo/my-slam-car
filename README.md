# my-slam-car
try to build my slam car


Based on the knowledge in myslam_learning and actual hardware cost considerations, I originally wanted to build a binocular slam car by myself, so I bought some materials to build the car, motor, control board, raspberry pi and d435i, and planned to build a binocular. After it's built, you can add imu fusion positioning.

 

But this is a plan that is too idealistic and has encountered many problems in practice:


1. The Raspberry Pi is a cheap single-board computer with limited computing power. When I try to run orb-slam2 on the Raspberry Pi, it is very lagging. The Raspberry Pi cannot meet this level of real-time Calculation, so it is unrealistic to install it on the Raspberry Pi based on refined frameworks such as orb-slam2 and vins-mono;
The initial plan was to write a package to control the movement in ros. A package includes slam-related nodes. During the period, a program was written in python to call the GPIO port with a Bluetooth keyboard. It can run, and you can feel a little delay and control range. Within 10 meters, after thinking about it, I removed this part, because I saw on Amazon that there are cheap rc trolleys, dozens of euros, which can control 50 meters, so that only a slam system needs to be built on the Raspberry Pi. It's okay

 

2. The d435i is a binocular. When I tried to call the binocular image, I encountered some problems. The realsense series cameras can publish binocular images, but they need the usb3.0 interface, but the usb interface of raspberry is 2.0. Therefore, if you want to build a slam system with 435i on raspberry, you can only adjust the original binocular to monocular;

 

3. The initial idea at that time was that if it was implemented based on monocular, the amount of calculation would be greater than that of binoculars, and due to the limited computing power of the Raspberry Pi, if you want to initialize the map by (triangulation), then you Frames are calculated to track, and the scale is restored through pnp. Such a scheme is not realistic. Even if the key frames are extracted, the result of monocular calculation is rough without back-end optimization and loop detection correction;

 

4. Later I decided to take the second place, considering it was the first time to write, I planned to make only a front end for positioning, not counting road signs, because there is no way to get an accurate road sign point position with less calculation under monocular; use 2d- The 2d convolutional geometry can restore the relative motion between two frames. If the matching is accurate, there will be obvious motion between the frames, and the calculated pose should also be accurate.



5. So the plan is roughly determined:

First receive the first frame, extract feature points from it, and set it as a key frame, and then track the previous frame in the second frame. The relative motion is judged by the number of tracked feature points. When the number of tracking points is greater than or equal to a certain threshold, Keep tracking

 

When the number of tracking points is less than a certain threshold, it is considered that significant motion has occurred between frames, set as key frames, extract feature points, match adjacent key frames, and calculate the pose;

 

Continue to track the latest key frame in the next frame;

 

6. The displacement calculated in this way is a normalized result, that is, the displacement calculated every time is a unit vector of length 1. I noticed similar commits when learning the convolutional geometry before, but it is not very Understand, I didn’t understand until it was time to use it. If you want to get an accurate displacement, you should also calculate a scale factor S, and multiply this S by the normalized result.
