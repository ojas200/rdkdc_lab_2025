clear;
close all;

ur5 = ur5_interface();
tf_frame.get_tf_tree();

%Forward kinematics
q = [0,-pi/2,0,-pi/2,pi/3,pi/3]';
g = ur5FwdKin(q);
fwdKinToolFrame = tf_frame('base_link','fwdKinToolFrame',eye(4));
pause(0.5);
fwdKinToolFrame.move_frame('base_link',g);
pause(0.5);
ur5.move_joints(q,10);
pause(0.5);
g_rviz = ur5.get_current_transformation('base_link','tool0');
pause(0.5);
%compare g_rviz to g, should be equal
%take screenshots for multiple configs in RVIZ

