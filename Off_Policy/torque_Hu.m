function f = torque_Hu(t, i, tu)
    if i > 10000 && i <= 11000
        f = [-10; -10];
    elseif i > 11000 && i <= 11500
        f = [-10; 
               -10 + 5.1*(1 - exp(-10*t(i - 11000)))];
    elseif i > 11500 && i <= 12200
        f = [-10 + 2*(1 - exp(-5*t(i - 11500)));
                tu{11500}(2) + 1*(1 - exp(-10*t(i - 11500)))];
    elseif i > 12200 && i <= 13000
        f = [tu{12200}(1) + 1.5*(1 - exp(-10*t(i - 12200)));
                tu{12200}(2)];
    elseif i > 13000 && i <= 20000
        f = tu{13000};
    elseif i > 20000 && i <= 20200
        f = [10*exp(-3*t(i - 20000));
                10];
    elseif i > 20200 && i <= 20600
        f = tu{20150};
    elseif i > 20600 && i <= 22000
        f = [tu{20600}(1)*exp(-1*t(i - 20600));
                10];
    elseif i > 22000 && i <= 27000
    f = [tu{20600}(1)*exp(-1*t(i - 20600));
            tu{22000}(2)*exp(-1*t(i - 22000))];     
    else
        f = [0; 0];
    end
end