e=250000000000;
l=24;
ro=69300;
u=0.28
x=-1:0.1:1;
y=x;
z1=1;
[X,Y]=meshgrid(x);
prt1=((z1.*z1)/(l*l));
prt2=(((x.*x)+(y.*y))/(l*l));
eqn=(((ro*l)/(2*e))*(prt1+(u*prt2)-1));
plot3(X,Y,eqn)
title('z=1 STEEL');
zlabel=('Uz/L');
grid;
