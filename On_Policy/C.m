function C = C(q, dq, Theta)
    C11 = -Theta(2)*dq(2)*sin(q(2));
    C12 = -Theta(2)*(dq(2)+dq(1))*sin(q(2));
    C21 = Theta(2)*dq(1)*sin(q(2));
    C22 = 0;
    C = [C11 C12; C21 C22];
end
