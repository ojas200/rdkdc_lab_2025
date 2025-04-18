%Takes in 6x1 joint space vector. Outputs the forward kinematics map g_st
function g_st = ur5FwdKin(q)
    % Taking g_st(0) from RViz and using product of exponentials calculation from Mathematica 
    % Check link lengths (from RViz)
    g_st_0 = [1 0 0 0.817;0 0 1 0.191;0 1 0 0;0 0 0 1];
    c_hat_1 = [0    -1     0     0;
     1     0     0     0;
     0     0     0     0;
     0     0     0     1];
    c_hat_2 = [0     0     0     0;
     0     0    -1     0;
     0     1     0     0;
     0     0     0     1];
    c_hat_3 = [0         0         0         0;
         0         0   -1.0000         0;
         0    1.0000         0    0.4250;
         0         0         0    1.0000];
    c_hat_4 = [0         0         0         0;
         0         0   -1.0000         0;
         0    1.0000         0    0.8170;
         0         0         0    1.0000];
    c_hat_5 = [0   -1.0000         0    0.1090;
    1.0000         0         0         0;
         0         0         0    0.8170;
         0         0         0    1.0000];
    c_hat_6 = [0         0         0    0.2008;
         0         0   -1.0000         0;
         0    1.0000         0    0.9117;
         0         0         0    1.0000];
    g_st = expm(c_hat_1*q(1)) * expm(c_hat_2*q(2)) * expm(c_hat_3*q(3)) * expm(c_hat_4*q(4)) * expm(c_hat_5*q(5)) *  expm(c_hat_6*q(6)) * g_st_0;
end