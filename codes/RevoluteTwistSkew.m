function xihat = RevoluteTwistSkew(q, w)

    %xi = [cross(-w, q); w];

    xihat = [SKEW3(w) cross(-w, q); 0 0 0 0];

end