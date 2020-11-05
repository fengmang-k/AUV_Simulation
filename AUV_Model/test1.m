clc
clear all
close all

R2D=180/pi;
D2R=pi/180;
h=0.1;      %步长：秒
m= 3000;    %节拍

%力和舵角初始�?
Fu = 0;
deltav = 0;
deltah = 0;

Force0 = [];
deltav0 = [];
deltah0 = [];

x = zeros(10,1); %临时状�?变量
T = zeros(m,1);  %时间
Y = zeros(m,10); %状�?变量

u0 = 0; v0 = 0; w0 = 0; q0 = 0; r0 = 0; x0 = 0; y0 = 0; z0 = 0; theta0 = 0; psi0 = 0;
Initial = [u0; v0; w0; q0; r0; x0; y0; z0; theta0;psi0];
x = Initial;

for i=1:1:m
    t = h*i;
    T(i,1)=t;
    
    Fu = 20;
    deltav = 0;
    deltah =5;
 
    %龙哥库塔�?
    k1=Infante_3d(t,x,[Fu deltav deltah]);
    k2=Infante_3d(t + h/2, x+h/2.*k1,[Fu deltav deltah]);
    k3=Infante_3d(t + h/2, x+h/2.*k2,[Fu deltav deltah]);
    k4=Infante_3d(t + h, x+h.*k3,[Fu deltav deltah]);
    x = x + h/6.*(k1 + 2.*k2 + 2.*k3 + k4);  
    x(1)=LimitMaxMin(x(1),6,0);
    x(2)=LimitMaxMin(x(2),2,-2);
	x(3)=LimitMaxMin(x(3),2,-2);
    x(4)=LimitMaxMin(x(4),0.1,-0.1);
    x(5)=LimitMaxMin(x(5),0.1,-0.1);
    x(9)=LimitMaxMin(x(9),D2R*20,-D2R*20);
    x(10)=AdjustAngle(x(10));
    
    Y(i,1) = x(1);%u
    Y(i,2) = x(2);%v
    Y(i,3) = x(3);%w
    Y(i,4) = x(4);%q
    Y(i,5) = x(5);%r
    Y(i,6) = x(6);%x
    Y(i,7) = x(7);%y
    Y(i,8) = x(8);%z
    Y(i,9) = x(9);%theta
    Y(i,10) = x(10);%psi 
    Force0 =[Force0 Fu];
    deltav0=[deltav0 deltav];
    deltah0=[deltah0 deltah];
    
end
U  = Y(:,1);
V  = Y(:,2);
W  = Y(:,3);
Q  = Y(:,4);
R  = Y(:,5);
Xp = Y(:,6);
Yp = Y(:,7);
Zp = Y(:,8);
theta = Y(:,9);
psi = Y(:,10);

figure(1);
plot3(Xp,Yp,Zp,'lineWidth',2);
set(get(gca, 'XLabel'), 'String', 'x');
set(get(gca, 'YLabel'), 'String', 'y');
set(get(gca, 'ZLabel'), 'String', 'deepth');

% figure(2);
% plot(T,deltah0);