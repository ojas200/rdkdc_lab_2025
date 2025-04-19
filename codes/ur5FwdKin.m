function gst = ur5FwdKin(q)
    
    L0 = 0.0892;
    L1 = 0.425;
    L2 = 0.392;
    L4 = 0.09475;
    L3 = 0.1093;

    e1 = [1; 0; 0];
    e2 = [0; 1; 0];
    e3 = [0; 0; 1];

    xi1 = RevoluteTwistSkew([0;0;0],e3);%Unchanged
    xi2 = RevoluteTwistSkew([-L0;0;0],e2);
    xi3 = RevoluteTwistSkew([-L0;0;L1],e2);
    xi4 = RevoluteTwistSkew([-L0;0;L1+L2],e2);
    xi5 = RevoluteTwistSkew([-L3;L1+L2;0],-e3);%Unchanged
    xi6 = RevoluteTwistSkew([L4-L0;0;L1+L2],e2);
    xis = {xi1, xi2, xi3, xi4, xi5, xi6};

    gst0 = [-1.000  0.000  0.000  0.817;
  0.000  0.000  1.000  0.191;
  0.000  1.000 -0.000 -0.005;
  0.000  0.000  0.000  1.000];

    gst = gst0;
   
    for i = length(q):-1:1

        gst = expm(xis{i}*q(i))*gst;

    end

end