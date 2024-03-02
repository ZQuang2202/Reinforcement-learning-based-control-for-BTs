function M = M(q, Theta)
    M11 = Theta(1) + 2*Theta(2)*cos(q(2));
    M12 = Theta(3) + Theta(2)*cos(q(2));
    M21 = M12;
    M22 = Theta(3);
    M = [M11 M12; M21 M22];
end
