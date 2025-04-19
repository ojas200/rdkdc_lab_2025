clear;
close all;

ur5 = ur5_interface();
tf_frame.get_tf_tree();

%Forward kinematics
q = [0,-pi/4,0,-pi/4,pi/4,pi/4]';
g = ur5FwdKin(q);
fwdKinToolFrame = tf_frame('base_link','fwdKinToolFrame',eye(4));
pause(0.5);
fwdKinToolFrame.move_frame('base_link',g);
pause(0.5);
ur5.move_joints(q,10);
pause(0.5);
g_rviz = ur5.get_current_transformation('base_link','tool0');
pause(0.5);

%Manipulability
%start with a singular condition(let theta_3 = 0, but rest should not cause singularity)
%increment theta from -pi/4 to pi/4
%theta = linspace(-pi/4, pi/4, 50);
%mu = zeros(length(theta), 3);
%for i=1:1:length(theta)
    %qind = [0,0,theta(i),0,0,0]; %only changing theta_3
    %Jind = ur5BodyJacobian(qind);
    %mu(i,1) = manipulability(Jind, 'sigmamin');
    %mu(i,2) = manipulability(Jind, 'detjac');
    %mu(i,3) = manipulability(Jind, 'invcond');
%end

%figure(1)
%plot(theta, mu(:,1));
%title('Manipulability: sigmamin for \theta_3 = -\pi/4 to \pi/4');
%xlabel('\theta_3 [rad]');
%ylabel('Manipulability \mu');

%figure(2)
%plot(theta, mu(:,2));
%title('Manipulability: detjac for \theta_3 = -\pi/4 to \pi/4');
%xlabel('\theta_3 [rad]');
%ylabel('Manipulability \mu');

%figure(3)
%plot(theta, mu(:,3));
%title('Manipulability: invcond for \theta_3 = -\pi/4 to \pi/4');
%xlabel('\theta_3 [rad]');
%ylabel('Manipulability \mu');

%getXi
%g = randSE3();
%xi = getXi(g);
%g_test = expm([SKEW3(xi(4:6,:)) xi(1:3,:); 0 0 0 0]);
%disp(g);
%disp(g_test);