function f = torque_En(t, k, te)
    if k > 13000 && k <= 13800
        f = [-4.8  + exp(-4*t(k - 13000));
                 -2.5 + exp(-3*t(k - 13000))];
    elseif k > 13800 && k <= 14000
        f = [te{13800}(1)  + exp(-10*t(k - 13800)) - 1;
                -2.5 + exp(-3*t(k - 13000))];
    elseif k > 14000 && k <= 14800
        f = [te{13800}(1)  + exp(-10*t(k - 13800)) - 0.3;
                te{14000}(2) + exp(-6*t(k - 14000))] - 1;
    elseif k > 14800 && k < 25000
        f = te{14800};
    else
        f = [0; 0];
    end
end
