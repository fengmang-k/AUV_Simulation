clc
clear all
close all

R2D=180/pi;
D2R=pi/180;
h=0.1;      %步长：秒
m= 3000;    %节拍

%力和舵角初始化
Fu = 0;
deltav = 0;
deltah = 0;

Force0 = [];
deltav0 = [];
deltah0 = [];

x = zeros(10,1); %临时状态变量
T = zeros(m,1);  %时间
Y = zeros(m,10); %状态变量

u0 = 0; v0 = 0; w0 = 0; q0 = 0; r0 = 0; x0 = 0; y0 = 0; z0 = 0; theta0 = 0; psi0 = 0;
Initial = [u0; v0; w0; q0; r0; x0; y0; z0; theta0;psi0];
x = Initial;

for i=1:1:m
    t = h*i;
    T(i,1)=t;
    
    Fu = 200;
    deltav =10;
    deltah =100;
 
    %龙哥库塔法
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
subplot(3,1,1);plot(T,U,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('纵向速度u[m/s]');grid on;hold on;
subplot(3,1,2);plot(T,V,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('横向速度w[m/s]');grid on;hold on;
subplot(3,1,3);plot(T,W,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('垂向速度w[m/s]');grid on;hold on;

figure(2);
subplot(3,1,1);plot(T,Xp,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('纵向位移[m]');grid on;hold on;
subplot(3,1,2);plot(T,Yp,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('横向位移[m]');grid on;hold on;
subplot(3,1,3);plot(T,Zp,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('垂向位移[m]');grid on;hold on;

figure(3);
plot3(Xp,Yp,Zp,'b--','LineWidth',2);xlabel('纵向位移[m]');ylabel('横向位移[m]');zlabel('垂向位移[m]');title('UUV航迹');grid on;hold on;%

figure(4);
subplot(2,1,1);plot(T,psi*180/pi,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('艏向角[角.deg]');grid on;hold on;%
subplot(2,1,2);plot(T,theta*180/pi,'b','LineWidth',2);xlabel('仿真时间[s]');ylabel('纵倾角[角.deg]');grid on;hold on;%%Delatv垂直舵

figure(5);
subplot(2,1,1);plot(Xp,Yp,'b--','LineWidth',2);xlabel('纵向位移[m]');ylabel('横向位移[m]');title('UUV航迹');grid on;hold on;
subplot(2,1,2);plot(Xp,Zp,'b--','LineWidth',2);xlabel('纵向位移[m]');ylabel('[垂向位移m]');title('UUV航迹');grid on;hold on;