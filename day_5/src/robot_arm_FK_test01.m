clc; clear;
syms d  [1 6] real
syms th [1 6] real
syms a  [1 6] real
syms al [1 6] real

T = cell(1,6);
for i = 1:6
    T{i} = [cos(th(i)), -sin(th(i))*cos(al(i)),  sin(th(i))*sin(al(i)), a(i)*cos(th(i));
            sin(th(i)),  cos(th(i))*cos(al(i)), -cos(th(i))*sin(al(i)), a(i)*sin(th(i));
            0,           sin(al(i)),             cos(al(i)),            d(i);
            0,           0,                      0,                     1];
end

d_r  = [171.00 0 0 380.73 0 212.35];          % mm
a_r  = [0 436.83 27.17 0 0 0];           % mm
al_r = sym(pi)*[1/2 0 1/2 -1/2 1/2 0];   % rad

off  = sym(pi)/180 * [-90, 180-3.52, -90+3.52, 0, 0, -90];   % 고정 오프셋 (정확한 값)

q    = sym(pi) * [0 0 0 0 0 0];
th_r = q + off;

for i = 1:6
    T{i} = subs(T{i}, [d a al], [d_r a_r al_r]);
end

for i = 1:6
    T{i} = subs(T{i}, th, th_r);
end

T0_6 = simplify(T{1}*T{2}*T{3}*T{4}*T{5}*T{6});

vpa(T0_6, 6)
