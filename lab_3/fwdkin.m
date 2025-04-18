clear;
close all;

function c = calc(w,v)
    c = [v,w]';
end
c
function c_hat = get(c)
    w_hat = SKEW3(c(4:6,:));
    v = c(1:3,:);
    c_hat = [w_hat,v;0 0 0 1];
end


w1 = [0,0,1];
v1 = [0,0,0];

w2 = [1,0,0];
v2 = [0,0,0];

w3 = [1,0,0];
v3 = [0,0,0.425];

w4 = [1,0,0];
v4 = [0,0,0.817];

w5 = [0,0,1];
v5 = [0.109,0,0.817];

w6 = [1,0,0];
v6 = [0.2008,0,0.91175];


c_1 = calc(w1,v1);
c_2 = calc(w2,v2);
c_3 = calc(w3,v3);
c_4 = calc(w4,v4);
c_5 = calc(w5,v5);
c_6 = calc(w6,v6);

c_hat_1 = get(c_1);
c_hat_2 = get(c_2);
c_hat_3 = get(c_3);
c_hat_4 = get(c_4);
c_hat_5 = get(c_5);
c_hat_6 = get(c_6);