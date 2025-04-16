clear all;
close all;

ur5 = ur5_interface();
tf_frame.get_tf_tree();

%Define g_st_0 or initial configuration


% a) Forward kinematics
q_test = [0,-pi/4,pi/6,-pi/4,0,0]'; %Column vector, add more if needed
g_st_estimated = ur5FwdKin(q_test);
viz_fwd_frame = tf_frame('base_link','Viz_Frame',g_st_estimated);
ur5.move_joints(q_test,5);
%ur5.get_current_transformation('base_link','tool0'); Use for numeric
%verification
pause(3); %Pause before next command

% b) Testing Body Jacobian
q_jac = [0,-pi/2,pi/3,-pi/6,0,pi/10]';
J_res = ur5BodyJacobian(q_jac);
e_1 = [1,0,0,0,0,0]';  %Function-ify this
e_2 = [0,1,0,0,0,0]';
e_3 = [0,0,1,0,0,0]';
e_4 = [0,0,0,1,0,0]';
e_5 = [0,0,0,0,1,0]';
e_6 = [0,0,0,0,0,1]';
e = 1e-3;

del_g_1 = (1/2*e)*(ur5FwdKin(q_jac + e*e_1) - ur5FwdKin(q_jac - e*e_1));
del_g_2 = (1/2*e)*(ur5FwdKin(q_jac + e*e_2) - ur5FwdKin(q_jac - e*e_2));
del_g_3 = (1/2*e)*(ur5FwdKin(q_jac + e*e_3) - ur5FwdKin(q_jac - e*e_3));
del_g_4 = (1/2*e)*(ur5FwdKin(q_jac + e*e_4) - ur5FwdKin(q_jac - e*e_4));
del_g_5 = (1/2*e)*(ur5FwdKin(q_jac + e*e_5) - ur5FwdKin(q_jac - e*e_5));
del_g_6 = (1/2*e)*(ur5FwdKin(q_jac + e*e_6) - ur5FwdKin(q_jac - e*e_6));

%%Resolve



% c) Testing Manipulability
range = -pi/4 : pi/20 : pi/4 ; %Define range
angles = zeros(size(range));
sig = zeros(size(range));
det = zeros(size(range));
inv = zeros(size(range));

i=1;
for theta_3 = range
    q_inp = [0,-pi/4,theta_3,-pi/2,0,0]';
    J_calc = ur5BodyJacobian(q_inp);
    angles(i) = theta_3;
    sig(i) = manipulability(J_calc,'sigmamin');
    det(i) = manipulability(J_calc,'detjac');
    inv(i) = manipulability(J_calc,'invcond');
end

% Plot all three on the same graph
figure;
plot(angles, sig, 'r-', 'LineWidth', 2); hold on;
plot(angles, detJ, 'g--', 'LineWidth', 2);
plot(angles, invC, 'b-.', 'LineWidth', 2);
xlabel('Theta\_3 (rad)');
ylabel('Manipulability Measures');
legend('Sigma Min', 'Determinant of Jacobian', 'Inverse Condition Number');
title('Manipulability Measures vs Theta\_3');
grid on;

% Test getXi
g_1 = randSE3();
g_2 = randSE3();
g_3 = randSE3();

xi_1 = getXi(g_1);
xi_2 = getXi(g_2);
xi_3 = getXi(g_3);

disp(norm(expm(xi_1)-g_1));
disp(norm(expm(xi_2)-g_2));
disp(norm(expm(xi_3)-g_3));

% Testing resolved rate control
q_des = [0,-pi/6,0,-pi/6,0,0]';
q_sing = [0,-pi/2,0,-pi/2,0,0]';
gain = 10;

disp(ur5RRcontrol(q_des,gain,ur5));
disp(ur5RRcontrol(q_sing,gain,ur5));
