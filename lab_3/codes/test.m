clear all;
close all;

ur5 = ur5_interface();
tf_frame.get_tf_tree();

q = [0,-pi/2,0,-pi/2,0,0]';
g = ur5FwdKin(q);

fwdkinframe = tf_frame('base_link','FwdKin',eye(4));
fwdkinframe.move_frame('base_link',g);
pause(4);
ur5.move_joints(q,10);
pause(10)