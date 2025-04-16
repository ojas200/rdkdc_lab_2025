%Implement resolved rate control with inputs of desired end-effector pose,
%gain and ur5 object interface. Ensure to check for singularities at each
%instant, if close to it, terminate and return -1. Define threshold of norm of twist components to terminate
%function. Can be hard-coded. Step size T_step can also be hard-coded.
%
function finalerr = ur5RRcontrol(gdesired,K,ur5)

end