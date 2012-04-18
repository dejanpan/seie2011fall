


vz = 10;            % Velocity
a = -32;            % Acceleration

t = 0:.1:1;
z = vz*t + 1/2*a*t.^2;
vx = 2;
x = vx*t;
vy = 3;
y = vy*t;
u = gradient(x);
v = gradient(y);
w = gradient(z);
scale = 0;
quiver3(x,y,z,u,v,w,scale)
view([70 18])