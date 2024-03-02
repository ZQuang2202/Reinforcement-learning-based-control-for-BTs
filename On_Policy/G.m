function G = G(q, Theta)
    G1 = Theta(4)*cos(q(1)) + Theta(5)*cos(q(1) + q(2));
    G2 = Theta(5)*cos(q(1) + q(2));
    G = [G1; G2];
end
