function f=fun(x,reflect_pool_xy,reflect_pool_ID);
%??????????
%??x1 x2
%??f1 f2
%syms x1 x2
syms x y
%f1 = sqrt((x1-4)^2 + x2^2)-sqrt(17);
%f2 = sqrt(x1^2 + (x2-4)^2)-5;
% a0=1795;
% b0=-1630;
% a1=8953;
% b1=-1700;
% a2=1885;	
% b2=1882;
% A=55/180*pi;
% B=32/180*pi;
a0=reflect_pool_xy(1,1);
b0=reflect_pool_xy(1,2);
a1=reflect_pool_xy(2,1);
b1=reflect_pool_xy(2,2);
a2=reflect_pool_xy(3,1);
b2=reflect_pool_xy(3,2);
A=angle(2);
B=angle(1);
f1=cos(A)-((x-a1)^2+(y-b1)^2+(x-a2)^2+(y-b2)^2-(a1-a2)^2-(b1-b2)^2)/(2*((x-a1)^2+(y-b1)^2)^0.5*((x-a2)^2+(y-b2)^2)^0.5)

f2=cos(B)-((x-a0)^2+(y-b0)^2+(x-a1)^2+(y-b1)^2-(a0-a1)^2-(b0-b1)^2)/(2*((x-a0)^2+(y-b0)^2)^0.5*((x-a1)^2+(y-b1)^2)^0.5)

f=[f1 f2];