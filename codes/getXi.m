%Takes in homogeneous transformation matrix as input. Outputs unnormalized
%twist vector c_hat_prime such that g = exp(c_hat_prime)
function xi = getXi(g)
    c_hat = logm(g); %Matrix logarithm returns skew symmetric unnormalized form
    v = c_hat(1:3,4);
    w = [-c_hat(2,3),c_hat(1,3),-c_hat(1,2)]';
    xi = [v;w]; 
end