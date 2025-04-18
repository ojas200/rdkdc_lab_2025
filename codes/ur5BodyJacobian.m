function Jb = ur5BodyJacobian(q)

    L1 = 425;
    L2 = 392;
    L4 = 94.75;
    L3 = 109.3;

    e1 = [1; 0; 0];
    e2 = [0; 1; 0];
    e3 = [0; 0; 1];

    xi1 = RevoluteTwistSkew([0;0;0],e3);
    xi2 = RevoluteTwistSkew([0;0;0],e1);
    xi3 = RevoluteTwistSkew([0;0;L1],e1);
    xi4 = RevoluteTwistSkew([0;0;L1+L2],e1);
    xi5 = RevoluteTwistSkew([L3;0;L1+L2],e3);
    xi6 = RevoluteTwistSkew([0;0;L1+L2+L4],e1);
    xis = {xi1, xi2, xi3, xi4, xi5, xi6};

    gst0 = [eye(3) [L1+L2; 0; 0]; 0 0 0 1];

    Jb = [];

    for i = 1:length(q)

        twist_skew = xis{i};
        
        rotm = gst0;

        for j = length(q):-1:i

            rotm = expm(xis{j}*q(j))*rotm;

        end

        adjinv = adjointInverse(rotm);
        
        wsk_i = twist_skew(1:3, 1:3);
        vi = twist_skew(1:3, 4);

        wi = [wsk_i(3,2); wsk_i(1,3); wsk_i(2,1)];

        tw_i = [vi; wi];

        Jb_col = adjinv*tw_i;

        Jb = [Jb, Jb_col];

    end

end