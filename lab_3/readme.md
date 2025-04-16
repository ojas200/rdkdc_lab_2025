**Notes:** <br />

*Create a transformation:* fwdKinToolFrame = tf_frame(‘base_link’,‘fwdKinToolFrame’,eye(4)) <br />
*To move it to transformation g:* fwdKinToolFrame.move_frame(‘base_link’,g) <br />
*Move robot physically to this transform:* “ur5.move_joints()” <br />
*Get transformation from base to tool tip:* "ur5.get_current_transformation(‘base_link’,‘tool0’)" <br />
*Get numeric value and compare to result:* “ur5.get_current_transformation” <br />

**Hierarchy**

Parent Script Lab3.m
calls the following subfunctions

*ur5FwdKin.m* <br />
– **Purpose**: Compute the forward kinematic map of the UR5. All necessary parameters (e.g. the base twists, gst0, etc) should be defined inside the function. <br />
– **Inputs:** q: 6 × 1 joint space variable vector = [θ1, θ2, θ3, θ4, θ5, θ6]T where θn is the angle of joint n for n = 1, · · · , 6. Be careful of sign convention! <br />
– **Output:** gst: end effector pose, gst (4 × 4 matrix) <br />
– **Testing:** <br />
1) define one joint vector, apply the ur5FwdKin function, and compute the resulting homogeneous transformation.
2) Use the frame visualization tool introduced in Lab2 to place a frame at this location and orientation in RVIZ. Then position the robot at the joint angles corresponding to the same joint vector you defined above (see Notes below for reference).
3) Your visualized frame should be at the end effector of the simulated UR5. Try a couple of poses and take some screenshots.

*ur5BodyJacobian.m* <br />
– **Purpose**: Compute the Jacobian matrix for the UR5. All necessary parameters are to be defined inside the function. Again, parameters such as twists and gst(0) (if needed) should be defined in the function. <br />
– **Input**: q: 6 × 1 joint space variables vector (see above) <br />
– **Output:** J: Body Jacobian, Jbst (6 × 6 matrix) <br />
– **Testing:** <br />
1) Calculate the Jacobian matrix using your function for some joint vector q.
2) Then, compute a central-difference approximation to the Jacobian as follows.
3) Compute the forward kinematics at slight offsets from q, 
    i.e. q ± εei where e1 = (1, 0, 0, 0, 0, 0)^⊤, e2 = (0, 1, 0, 0, 0, 0)^⊤, etc. 
    You will then have gst(q + εei) and gst(q − εei), and note that, approximately, we have ∂g/∂qi ≈ 1/2ε(gst(q + εei) − gst(q − εei))
    For a small enough ε, you should have that the ith column of the Jacobian is equal to ξ′i ≈ (g^−1 ∂g/∂qi)^V
4) Note that the term on the right will NOT be exactly a twist! So you’ll want to “twist-ify” it first by taking the skew-symmetric part of the upper left 3 × 3 matrix before finding the twist coordinate. 
5) Your test function should, for each column, compute the approximate twist in Eq. 1. You will then be able to construct an approximate Jacobian, Japprox and compute the matrix norm of the error between Japprox and the actual Jacobian. E.g. in MATLAB something like norm(Japprox -J). Print this on the command line in MATLAB.


*manipulability.m* <br />
– **Purpose**: Compute a measure of manipulability. Implement all three different types: ‘sigmamin’, ‘detjac’, or ‘invcond’, as defined in Chapter 3, Section 4.4 of MLS. This function will return any one of the three measures of manipulability as defined by the second argument. <br />
– **Inputs:** There are two inputs to this function: <br />
∗ J: a 6 × 6 matrix <br />
∗ measure: a single string argument that can only be one of ‘sigmamin’, ‘detjac’, or ‘invcond’. Defines which manipulability measure is used. <br />
– **Output:** mu: The corresponding measure of manipulability <br />
– **Testing:** <br />
1) Plot the value of each manipulability measurement near a singularity as a function of a joint angle. 
    a) For instance, one of the singular configurations of the UR5 identified in HW6 is when θ3 = 0. 
    b) In MATLAB increment θ3   from (−π/4, π/4), compute J at each increment (keep all other joints constant), and at each angle plot the values returned from your manipulability.m function for each of ‘sigmamin’, ‘detjac’, and ‘invcond’ measurement types. 
    Here the x-axis will be θ3 values and the y-axis will be the manipulability measure values.
2) Include these plots in your lab write-up.

*getXi.m* <br />
– **Purpose**: Take a homogenous transformation matrix and extract the unscaled twist <br />
– **Input:** g: a homogeneous transformation <br />
– **Output:** xi: the (un-normalized) twist in 6 × 1 vector or twist coordinate form such that g = exp(ˆξ) <br />
– **Testing:** <br />
1) Choose a couple of arbitrary homogenous transforms and show that to machine precision that when you exponentiate the resulting twist (as an element of se(3)) using MATLAB’s expm command, you wind up at the homogeneous transform you started with.

*ur5RRcontrol.m* <br />
– **Purpose**: Implement a discrete-time resolved-rate control system. This should iteratively implement the following Resolved-Rate control system: q_(k+1) = qk − (K Tstep)[Jbst(qk)]^-1*ξk
where, at each time step, ξk would be taken by running getXi on the appropriate error matrix exp(ˆξk) = g_(t∗t) = (g_st*)^−1 g_st <br />

This script should terminate norm(vk) and norm(ωk) < threshold. (Note they have different units!) You might want to make it something
like 5cm and 15 degrees to start, but as you perfect your code, you should be able to make this tighter. The threshold can be hard-coded in your MATLAB script, but here you are going to need to play around a little based on other parameters. I suggest you start with a fairly forgiving value, to ensure that your code terminates. Also, the step size, Tstep can be hard-coded in your script and may require some experimentation to get a good value that runs smoothly since the MATLAB interface is not real-time.

Also, at each step, you should check for singularities! If you are close to a singularity the system should ABORT, and return -1. <br />
– **Inputs:** <br />
Robot DKDC (EN.530.646) 5
∗ gdesired: a homogeneous transform that is the desired end-effector pose, i.e. gst∗ .
∗ K: the gain on the controller
∗ ur5: the ur5 interface object <br />
– **Output:** finalerr: -1 if there is a failure. If convergence is achieved, give the final positional error in cm. <br />
– **Testing:** <br />
1) Define a desired configuration and a gain, and then use the command to show that the robot moves to the goal.
2) Pick a second initial condition that is near a singularity (or a goal that is near a singularity) and show that your command terminates and returns -1.
