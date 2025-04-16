%Generate a random SE3 transformation
function X = randSE3()
    % TODO: Generate a random rotation matrix
    %For a rotation matrix det(R) = I and RR' = I
    %Using a quaternion approach and defining quaternion from scratch
    theta  = randi([0,6],1,1);%Values between 0 to 2pi
    A = rand(1,3); 
    A = A/(norm(A));          %A 3D unit vector in space

    a = cos(theta/2);
    b = sin(theta/2)*(A);     %Defining components of a quaternion
    q = [a,b];                %Quaternion
    R = quat2rotm(q);         %Rotation matrix
    % TODO: Generate a random translation
    t = rand(3,1);
    X = [ R t; 0 0 0 1];
end