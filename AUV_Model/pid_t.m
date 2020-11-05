clc
clear all;
close all;

R2D=180/pi;
D2R=pi/180;

%% target 
target = [40; 40; 40];
expect_velocity = 0.6;
%% PID parameters
pid_param = [20.0, 5.0, 0.0; % velocity angle control kp,ki,kd
            10.0, 0.05, 0.0;  % yaw angle control kp,ki,kd
            0.0, 1.8, 0.0;];  % pitch control kp,ki,kd

e_0 = zeros(3,1);   % post time error
e_acc = zeros(3,1); %error accumulation
e_c = e_0';

%% model start state

% force and fin angle
Fu = 0.0;
deltar =0.0;
deltah =0.0;
tau_c = [Fu deltar deltah];

% simulation start state
h=0.1;      % step

u0 = 0; v0 = 0; w0 = 0; q0 = 0; r0 = 0; x0 = 0; y0 = 0; z0 = 0; theta0 = 0; psi0 = 0;
Initial = [u0; v0; w0; q0; r0; x0; y0; z0; theta0;psi0];
x = Initial;    %AUV state

tt = 0; % time scale
t_c = tt;
x_c = x';

%% velocity 
vel_x0 = 0; vel_y0 = 0; vel_z0 = 0;
vel = [vel_x0; vel_y0; vel_z0];
vel_c = norm(vel);

%% simulation
while norm(target-x(6:8,1)) > 1 && tt <= 3000
       
    t = tt*h;
    % runge-Kutta method
    k1=AUV_model(t,x,[Fu deltar deltah]);
    k2=AUV_model(t + h/2, x+h/2.*k1,[Fu deltar deltah]);
    k3=AUV_model(t + h/2, x+h/2.*k2,[Fu deltar deltah]);
    k4=AUV_model(t + h, x+h.*k3,[Fu deltar deltah]);
    x = x + h/6.*(k1 + 2.*k2 + 2.*k3 + k4);  
    x(1)=LimitMaxMin(x(1),6,0);
    x(2)=LimitMaxMin(x(2),2,-2);
	x(3)=LimitMaxMin(x(3),2,-2);
    x(4)=LimitMaxMin(x(4),0.1,-0.1);
    x(5)=LimitMaxMin(x(5),0.1,-0.1);
    x(9)=LimitMaxMin(x(9),D2R*70,-D2R*70);
    x(10)=AdjustAngle(x(10));

    %% ===========================velocity control==================================%
    % J1 transform matrix (u;v;w)->d(X;Y;Z)
    J1=[cos(x(10))*cos(x(9)) -sin(x(10)) cos(x(10))*sin(x(9));
    sin(x(10))*cos(x(9)) cos(10) sin(x(10))*sin(9);
    -sin(x(9)) 0 cos(x(9))];
    vel = J1*x(1:3,1);
    vel_t = norm(vel);
    err_vel = expect_velocity - vel_t;
    Fu = pid_param(1,1)*err_vel + pid_param(1,2)*e_acc(1,1) + pid_param(1,3)*(err_vel-e_0(1,1));
    Fu = LimitMaxMin(Fu,350,0);

    e_acc(1,1) = e_acc(1,1) + err_vel;
    e_0(1,1) = err_vel;

    vel_c = [vel_c;vel_t];

    
    %% ===========================yaw control===========================%
    
    expect_psi = atan2(target(2)-x(7),target(1)-x(6));
    err_psi = expect_psi - x(10);
    deltar = pid_param(2,1)*err_psi + pid_param(2,2)*e_acc(2,1) + pid_param(2,3)*(err_psi-e_0(2,1));
    deltar = LimitMaxMin(deltar,10,-10);


    e_acc(2,1) = e_acc(2,1) + err_psi;
    e_0(2,1) = err_psi;

  
    %% ===========================pitch control=====================================%

    expect_theta = atan2(target(1)-x(6),target(3)-x(8));
    err_theta = expect_theta - (pi/2+x(9));
    deltah = pid_param(3,1)*err_theta + pid_param(3,2)*e_acc(3,1) + pid_param(3,3)*(err_theta-e_0(3,1));
    
    deltah = LimitMaxMin(deltah,30,-30);

    e_acc(3,1) = e_acc(3,1) + err_theta;
    e_0(3,1) = err_theta;


    %%
    tau = [Fu deltar deltah];
    tt = tt+1;
    t_c = [t_c; tt*h];
    x_c = [x_c; x'];
    tau_c = [tau_c;tau];
    e_c = [e_c;e_0'];

end

disp(tt);
%% plot
figure(1);
plot3(x_c(:,6),x_c(:,7),x_c(:,8),'lineWidth',2);
% hold on;
% plot3(x_c(1,6),x_c(1,7),x_c(1,8),'y^','MarkerSize',10);
hold on;
plot3(target(1),target(2),target(3),'rs','MarkerSize',10);
set(get(gca, 'XLabel'), 'String', 'x');
set(get(gca, 'YLabel'), 'String', 'z');
set(get(gca, 'ZLabel'), 'String', 'depth');

figure(2);
plot(t_c,vel_c,t_c,tau_c(:,1)*0.01,t_c,e_c(:,1));
legend('velocity','force','err-vel');
figure(3);
plot(t_c,x_c(:,10),t_c,tau_c(:,2),t_c,e_c(:,2));
legend('\psi','deltar','err-\psi');
grid on;
figure(4);
plot(t_c,x_c(:,9),t_c,tau_c(:,3)*0.1,t_c,e_c(:,3));
legend('\theta','deltah','err-\theta');
grid on;
