function dydt = Infante_3d(t,x,formom)

% an Autnomous Underwater Vehicle (AUV) of Infante. While the state vector is defined as follows:
% u     = surge velocity          (m/s)
% v     = sway velocity           (m/s)
% w     = heave velocity          (m/s)
% p     = roll velocity           (rad/s) //don't care
% q     = pitch velocity          (rad/s)
% r     = yaw velocity            (rad/s)
% x_p  = position in x-direction (m)
% y_p  = position in y-direction (m)
% z_p  = position in z-direction (m)
% phi   = roll angle              (rad) //don't care
% theta = pitch angle             (rad)
% psi   = yaw angle               (rad)


dydt = zeros(10,1);

%控制输入
Fu = formom(1,1); 
deltav = formom(1,2); 
deltah = formom(1,3);

%状态变量
u = x(1); v = x(2); w = x(3);
q = x(4); r = x(5); xx = x(6); yy = x(7); zz = x(8);
theta = x(9); psi = x(10);

%模型水动力系数
m =185;g =9.85;B =m*g; W =B;zg =0.01;zb =-0.01;

Xu =70;Xuu =100;Yv=100;Yvv=200;Zw=100;Zww =200;
Mq =50;Mqq =100;Nr= 50;Nrr=100;
m1 =215;m2 =265;m3 =265;
m4 =80;m5 =80;

g1 = (W - B)*cos(theta); g2 = (zg*W - zb*B)*sin(theta);
d1 = Xu + Xuu*abs(u); d2 = Yv + Yvv*abs(v); d3 = Zw + Zww*abs(w);
d4 = Mq + Mqq*abs(q); d5 = Nr + Nrr*abs(r); 

%AUV 数学模型
dydt(1) = (1/m1)*(m2*v*r - m3*w*q - d1*u + Fu); %du   
dydt(2) = -(1/m2)*(m1*u*r + d2*v);%dv
dydt(3) = (1/m3)*(m1*u*q - d3*w + g1);%dw                         
dydt(4) = (1/m4)*((m1 - m3)*u*w - d4*q - g2 +deltah );%dq
dydt(5) = (1/m5)*((m1 - m2)*u*v - d5*r +deltav);%dr

dydt(6) = cos(psi)*cos(theta)*u -sin(psi)*v + cos(psi)*sin(theta)*w; %dx
dydt(7) = sin(psi)*cos(theta)*u + cos(psi)*v + sin(theta)*sin(psi)*w; %dy
dydt(8) = -sin(theta)*u + cos(theta)*w;%dz
dydt(9) = q;%dtheta
dydt(10) = r/cos(theta);%dpsi

dydt(1)=LimitMaxMin(dydt(1),0.5,-0.1);
dydt(2)=LimitMaxMin(dydt(2),0.5,-0.5);
dydt(3)=LimitMaxMin(dydt(3),0.5,-0.5);
dydt(4)=LimitMaxMin(dydt(4),0.1,-0.1);
dydt(5)=LimitMaxMin(dydt(5),0.1,-0.1);


