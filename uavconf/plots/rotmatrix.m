pkg load symbolic;

syms r p y m;

pm = [[1, 0, 0]; [0, cos(p), -sin(p)]; [0, sin(p), cos(p)]];
rm = [[cos(r), 0, sin(r)]; [0, 1, 0]; [-sin(r), 0, cos(r)]];
ym = [[cos(y), -sin(y), 0]; [sin(y), cos(y), 0]; [0, 0, 1]];

vv = [0; 0; 1];
fv = [1; 0; 0];
sv = [0; 1; 0];

m = transpose(rm * pm);
#m * vv
#m * fv
#m * sv


rm = [[1, 0, 0]; [0, cos(r), -sin(r)]; [0, sin(r), cos(r)]];
pm = [[cos(p), 0, sin(p)]; [0, 1, 0]; [-sin(p), 0, cos(p)]];
ym = [[cos(y), -sin(y), 0]; [sin(y), cos(y), 0]; [0, 0, 1]];

ym * pm * rm
