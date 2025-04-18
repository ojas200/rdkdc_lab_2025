%Takes in 6x6 Jacobian and a string which can be either of sigmamin, detjac
%or invcond as defined in Chapter 3, 4.4, MLS. Return manipulability
%measure 
function mu = manipulability(J,measure)
    if strcmp(measure,'sigmamin')
        %Return minimum singular value of J. If singular, return 0
        A = svd(J); %Retuns singular values in descending order
        mu = A(end); %Gets minimum element
        if mu < 1e-5 %Checking for minuscule element and returning zero
            mu = 0;
        end

    elseif strcmp(measure,'invcond')
        %Return ratio of min and max singular values
        B = svd(J);
        mu = B(end)/B(1);
        if mu < 1e-5 %Checking for minuscule element and returning zero
            mu = 0;
        end
        
    else
        %Detjac. Return determinant of matrix
        mu = det(J);
        if mu < 1e-5
            mu = 0;
        end
    end
end