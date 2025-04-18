clear all;
close all;

ur5 = ur5_interface();
tf_frame.get_tf_tree();

%Forward kinematics
q = [];
g = ur5FwdKin(q);
fwdKinToolFrame = tf_frame(‘base_link’,‘fwdKinToolFrame’,eye(4));
pause(0.5);
fwdKinToolFrame.move_frame(‘base_link’,g);
pause(0.5);
ur5.move_joints( );
pause(0.5);
g_rviz = ur5.get_current_transformation(‘base_link’,‘tool0’);
pause(0.5);
%compare g_rviz to g, should be equal
%take screenshots for multiple configs in RVIZ

%Body Jacobian
q_curr = ur5.get_current_joints();
Jbody = ur5BodyJacobian(q);
%compute a central-difference approximation to the Jacobian
%ith column of the jacobian should be = twist_i' = (invSE3(g)*dg/dqi)^v
invg = invSE3(g);
Japprox = zeros(6,6);
for i=1:1:6
    diffg = diff(g, q(i));
    invgdgdq = invg*diffg;
    twisti = zeros(6,1); %placeholder for now, see comment below
%twistify right by taking skew-symmetric of upper left 3x3, then take twist coordinate
%use this to construct Japprox, compare to Jbody
    Japprox(:,i) = twisti;
end
%compute matrix norm -> norm(Japprox-J);
matrixNorm = norm(Japprox-J);

%Manipulability
%start with a singular condition(let theta_3 = 0, but rest should not cause singularity)
%increment theta from -pi/4 to pi/4
theta = linspace(-pi/4, pi/4, 50);
mu = zeros(length(theta), 3);
for i=1:1:length(theta)
    qind = [0,0,theta(i),0,0,0]; %only changing theta_3
    Jind = ur5BodyJacobian(qind);
    mu(i,1) = manipulability(Jind, 'sigmamin');
    mu(i,2) = manipulability(Jind, 'detjac');
    mu(i,3) = manipulability(Jind, 'invcond');
end

figure(1)
plot(theta, mu(:,1));
title('Manipulability: sigmamin for \theta_3 = -\pi/4 to \pi/4');
xlabel('\theta_3 [rad]');
ylabel('Manipulability \mu');

figure(2)
plot(theta, mu(:,2));
title('Manipulability: detjac for \theta_3 = -\pi/4 to \pi/4');
xlabel('\theta_3 [rad]');
ylabel('Manipulability \mu');

figure(3)
plot(theta, mu(:,3));
title('Manipulability: invcond for \theta_3 = -\pi/4 to \pi/4');
xlabel('\theta_3 [rad]');
ylabel('Manipulability \mu');

%Xi
%choose a couple of arbitrary homogenous transforms
%show that expm(twist) in SE(3) will equal the homogeneous transform

%Resolved Rate Control
K = 1; %gain
%desired config, should watch robot move in RVIZ
q_des = [];
g_des = ur5FwdKin(q_des);
q_des_err = ur5RRcontrol(g_des, K, ur5);

%singular config, should see -1 output
q_sing = [0,0,0,0,0,0]';
g_sing = ur5FwdKin(q_sing);
q_sing_err = ur5RRcontrol(g_sing, K, ur5);