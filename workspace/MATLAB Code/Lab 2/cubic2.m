t = 1:0.01:2

a0 = -2;
a1 = 6;
a2 = -4.5;
a3 = 1;
tt = a0 + a1*t + a2*t.^2+a3*t.^3;
tdot = a1 + 2*a2*t+3*a3*t.^2;
tddot = 2*a2+6*a3*t;
thetas = [t;tdot;tddot]

plot(t, tt)
