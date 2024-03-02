function f = torque_En(t, i, te)
    if i > 13000 && i <= 13800
        f = [-4.8  + exp(-4*t(i - 13000));
                 -2.5 + exp(-3*t(i - 13000))];
    elseif i > 13800 && i <= 14000
        f = [te{13800}(1)  + exp(-10*t(i - 13800)) - 1;
                -2.5 + exp(-3*t(i - 13000))];
    elseif i > 14000 && i <= 14800
        f = [te{13800}(1)  + exp(-10*t(i - 13800)) - 0.3;
                te{14000}(2) + exp(-6*t(i - 14000))] - 1;
    elseif i > 14800 && i < 25000
        f = te{14800};
    else
        f = [0; 0];
    end
end