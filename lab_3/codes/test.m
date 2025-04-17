clear all;
close all;

g_test = randSE3();
disp(g_test);
c_hat = logm(g_test);
disp(c_hat);
v = c_hat(1:3,4);
w = [-c_hat(2,3),c_hat(1,3),-c_hat(1,2)]';
twist = [v;w];
disp(twist);
