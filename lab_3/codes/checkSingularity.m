%Takes in a Jacobian and checks if the configuration is singular or not.
%Returns bool value according to presence of singularity.
function singularity = checkSingularity(J)
    %Using concept that minimum singular value of J at singularity is
    %nearly zero
    singularity = 0;
    A = svd(J);
    min = A(end);
    if min < 1e-5
        singularity = 1;
    end
end