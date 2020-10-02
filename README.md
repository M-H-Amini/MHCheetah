# MHCheetah
My master's thesis for simulation and implementation of a quadruple robot which I call it "MHCheetah" :)

# Joint Velocity Controls
I've written the following functions to control joint velocities easier.

*  setFrontRightVelocity
*  setFrontLeftVelocity
*  setBackRightVelocity
*  setBackLeftVelocity

For example, ```setBackLeftVelocity(p, quadruped, 10)``` rotates the back left leg with a constant velocity of 10.

# Joint Position Controls
I've written the following functions to control joint positions easier.

*  setFrontRightPosition
*  setFrontLeftPosition
*  setBackRightPosition
*  setBackLeftPosition

For example, ```setFrontLeftPosition(p, quadruped, - np.pi/2)``` sets the back left leg position to pi/2.

