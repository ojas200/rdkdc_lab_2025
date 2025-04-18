function adjinv = adjointInverse(g)

    R = g(1:3, 1:3);
    p = g(1:3, 4);

    p_hat = SKEW3(p);

    adjinv = [R.', -R.' * p_hat;
              zeros(3,3), R.'];


end