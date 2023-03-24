t = 0:0.01:1;

a0 =0;
a1 = 0;
a2 = 1.5;
a3 = -1;
tt = a0 + a1*t + a2*t.^2+a3*t.^3;
tdot = a1 + 2*a2*t+3*a3*t.^2;
tddot = 2*a2+6*a3*t;
thetas = [t;tdot;tddot];

plot(t, tt)

% a=-2\:b=6,\:c=-4.5,\:d=1