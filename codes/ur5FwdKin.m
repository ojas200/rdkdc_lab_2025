function T = ur5FwdKin(theta)
    % UR5FWDKIN Computes the forward kinematics of the UR5 robot.
    %   T = UR5FWDKIN(theta) returns the end-effector pose (a 4x4 matrix)
    %   computed from the 6x1 joint angle vector theta (in radians).

    %% Robot Parameters (in meters)
    W1 = 0.1093;
    W2 = 0.0825;
    L1 = 0.425;
    L2 = 0.392;
    H1 = 0.0892;
    H2 = 0.09475;

    %% Joint Axes and Positions
    % Angular velocity directions (w vectors)
    w1 = [0, 0, 1];
    w2 = [0, 1, 0];
    w3 = [0, 1, 0];
    w4 = [0, 1, 0];
    w5 = [0, 0, -1];
    w6 = [0, 1, 0];

    % Position vectors (v vectors)
    v1 = [0, 0, 0];
    v2 = [-H1, 0, 0];
    v3 = [-H1, 0, L1];
    v4 = [-H1, 0, L1+L2];
    v5 = [-W1, L1+L2, 0];
    v6 = [H2-H1, 0, L1+L2];

    %% Home Configuration of the End Effector
    gst0 = [-1, 0, 0, L1+L2;
             0, 0, 1, W1+W2;
             0, 1, 0, H1-H2;
             0, 0, 0, 1];

    %% Build the Screw Axes
    v = [v1; v2; v3; v4; v5; v6];
    w = [w1; w2; w3; w4; w5; w6];
    screw = zeros(6,6);
    for i = 1:6
         omega = w(i, :)';
         vi    = v(i, :)';
         screw(:, i) = [vi; omega];
    end

    %% Compute the Forward Kinematics using the Product of Exponentials Formula
    T = eye(4);  % Initialize transformation as the identity matrix
    for i = 1:6
         % Multiply the transformation by each twist exponential:
         T = T * TwistExponential(screw(:, i), theta(i));
    end
    T = T * gst0;  % Multiply by the home configuration at the end

    % At this point, T is the transformation matrix for the end effector.

end

function T_exp = TwistExponential(xi, theta_val)
    % TWISTEXPONENTIAL Computes the matrix exponential for a twist.
    %   T_exp = TWISTEXPONENTIAL(xi, theta_val) calculates the 4x4 transformation
    %   matrix from the twist xi (6x1 vector: [v; w]) and the joint angle theta_val.

    v = xi(1:3);  % Linear part
    w = xi(4:6);  % Angular part

    % Create the skew-symmetric matrix for w:
    w_hat = [  0   -w(3)  w(2);
             w(3)   0    -w(1);
            -w(2)  w(1)   0  ];
    
    % Form the 4x4 matrix representation of the twist:
    xi_mat = [w_hat, v;
              0, 0, 0, 0];

    % Compute the matrix exponential:
    T_exp = expm(xi_mat * theta_val);

end