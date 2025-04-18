%Implement resolved rate control with inputs of desired end-effector pose,
%gain and ur5 object interface. Ensure to check for singularities at each
%instant, if close to it, terminate and return -1. Define threshold of norm of twist components to terminate
%function. Can be hard-coded. Step size T_step can also be hard-coded.
%
function finalerr = ur5RRcontrol(gdesired,K,ur5)
    Tstep = 0.1;
    threshold_pos = 0.05; %5cm
    threshold_rot = deg2rad(15);

    max_iters = 1000;
    i=0;
    while true
        %get current joint config
        qk = ur5.get_current_joints();

        %forward kinematics 
        gst = ur5FwdKin(qk);

        %calculate g_err
        g_err = invSE3(gst)*gdesired;

        %twist from g_err
        xi = getXi(g_err);
        v = xi(1:3);
        omega = xi(4:6);

        %check thresholds and break if not passing
        if norm(v) < threshold_pos && norm(omega) < threshold_rot
            break;
        end

        %now check for singularity 
        J = ur5BodyJacobian(qk);
        singular = checkSingularity(J);
        if singular == 1
            finalerr = -1;
            return; %return -1 if singular
        end

        %update resolved rate
        qk = qk - (K*Tstep)*pinv(J)*xi;

        %update robot pose
        ur5.move_joints(qk);
        pause(1); %pause for a second to allow RVIZ time

        i=i+1;
        if i > max_iters
            warning('Exceeded maximum iterations');
            break;
        end

    end
    
    finalerr = norm(v)*100; %convert to cm
end