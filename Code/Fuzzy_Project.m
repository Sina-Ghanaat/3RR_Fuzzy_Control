clc;clear;
T = 10; % Survey Time
syms theta1 theta2 theta3
L1=1.5;
L2=0.8;
L3=0.7;
T_01 = [cos(theta1) -sin(theta1) 0  0;
        sin(theta1)  cos(theta1) 0  0;
        0            0           1  0;
        0            0           0  1];
    
T_12 = [cos(theta2) -sin(theta2) 0  0;
        0            0          -1  0;
        sin(theta2)  cos(theta2) 0  0;
        0            0           0  1];
    
T_23 = [cos(theta3) -sin(theta3) 0  L2;
        sin(theta3)  cos(theta3) 0  0;
        0            0           1  0;
        0            0           0  1];

T_34 = [1            0           0  L3;
        0            1           0  0;
        0            0           1  0;
        0            0           0  1];
    
T_04 = simplify(T_01*T_12*T_23*T_34);

x = T_04(1,4);
y = T_04(2,4);
z = T_04(3,4)+L1;
%%
n = 1;
m = 1;
v = 1;
for i = 0:0.5:2*pi
    for j = 0:0.5:2*pi
        for k = 0:0.5:2*pi
            X(n) = (cos(i)*(7*cos(j + k) + 8*cos(j)))/10;
            Y(m) = (sin(i)*(7*cos(j + k) + 8*cos(j)))/10;
            Z(v) = (7*sin(j + k))/10 + (4*sin(j))/5 + 3/2;
            n = n+1;
            m = m+1;
            v = v+1;
        end
    end
end
set(gcf,'color','w');
plot3(X,Y,Z)
title ('Task Space')
xlabel('X') 
ylabel('Y')
zlabel('Z')
grid on
%%

Pee_i = [0.5 , 0 , 1.5]; % End-Effector Position at its Initial State
Pee_f = [-0.5 , -0.7 , 1.5]; % End-Effector Position at its Final State
theta_q1i = atan2(Pee_i(2), Pee_i(1));
theta_q3i = acos((Pee_i(1)^2+Pee_i(2)^2+(Pee_i(3)-L1)^2-L2^2-L3^2)/(2*L2*L3)).*[1,-1]; % Construction (Theta2i) at The Initial Position and (pi - theta2i) as Complements via Inverse Kinematics
phi_i = atan2(L3*sin(theta_q3i), L3*cos(theta_q3i)+L2);
theta_q2i = asin((Pee_i(3)-L1)*cos(phi_i)/(L3*cos(theta_q3i)+L2))-phi_i; % Construction Two (Theta1i) Values at The Initial Position Based on Theta2f Choice
theta_q1f = atan2(Pee_f(2), Pee_f(1));
theta_q3f = acos((Pee_f(1)^2+Pee_f(2)^2+(Pee_f(3)-L1)^2-L2^2-L3^2)/(2*L2*L3)).*[1,-1]; % Construction (Theta2f) at The Final Position and (pi - theta2f) as Complements via Inverse Kinematics
phi_f = atan2(L3*sin(theta_q3f), L3*cos(theta_q3f)+L2);
theta_q2f = asin((Pee_f(3)-L1)*cos(phi_f)/(L3*cos(theta_q3f)+L2))-phi_f; % Construction Two (Theta1f) Values at The Final Position Based on Theta2f Choice
invmatrix = [theta_q3i,theta_q3i;theta_q2i,theta_q2i;([theta_q3f;theta_q2f]*[1,1,0,0,;0,0,1,1])]; % This Matrix is an Indicator for All Possible Configuration frome initial Position to Final Position
%% q1
theta_i_q1 = theta_q1i;
theta_f_q1 = theta_q1f;
t_b_q1 = 0.8; % Blend Time
theta_dd_q1 = -1.*(theta_f_q1 - theta_i_q1)./(t_b_q1.^2 - T .* t_b_q1); % Acceleration of q1
theta_b_q1 = theta_i_q1 + 1/2 * theta_dd_q1.*t_b_q1.^2; % Blend Magnitude
traj_q1=[0,0,0,0];
b1 = theta_b_q1 - theta_i_q1; % Blend Offest
m1 = ((theta_f_q1-b1)-(theta_i_q1 + b1))./(T - 2.*t_b_q1); % Linear Slope
i1=1;
ts1 = 0;
for t1=0:0.01:t_b_q1 % First Blend
    traj_q1 (i1,:) = theta_i_q1 + 1/2.*theta_dd_q1*t1^2;
    ts1(i1) = t1;
    i1 = i1 + 1;
end
for t1=t_b_q1+0.01:0.01:T-t_b_q1 % Linear Part
    traj_q1 (i1,:) = theta_b_q1 + m1.*(t1-t_b_q1);
    ts1(i1) = t1;
    i1 = i1 + 1;
end
for t1=-t_b_q1:0.01:0 % Final Blend
    traj_q1 (i1,:) = -1/2.*theta_dd_q1.*(t1)^2 + (theta_f_q1);
    ts1(i1) = t1 + t_b_q1 + (T-t_b_q1);
    i1 = i1 + 1;
end
%% q2
theta_i_q2=invmatrix (2,:); % Choosing All q2 Possible Configuration at The Initial Position (With Respect to q1)
theta_f_q2 = invmatrix (4,:); % Choosing All q2 Possible Configuration at The Final Position (With Respect to q1)
t_b_q2 = 0.8; % Blend Time
theta_dd_q2 = -1.*(theta_f_q2 - theta_i_q2)./(t_b_q2.^2 - T .* t_b_q2); % Acceleration of q1
theta_b_q2 = theta_i_q2 + 1/2. * theta_dd_q2.*t_b_q2^2; % Blend Magnitude
traj_q2 = [0,0,0,0];
b2 = theta_b_q2 - theta_i_q2;% Blend Offest
m2 = ((theta_f_q2-b2)-(theta_i_q2 + b2))./(T - 2*t_b_q2); % Linear Slope
i2=1;
ts2 = 0;
for t2=0:0.01:t_b_q2 % First Blend
    traj_q2 (i2,:) = theta_i_q2 + 1/2.*theta_dd_q2.*t2^2;
    ts2(i2) = t2;
    i2 = i2 + 1;
  
end
for t2=t_b_q2+0.01:0.01:T-t_b_q2 % Linear Part
    traj_q2 (i2,:) = theta_b_q2 + m2.*(t2-t_b_q2);
    ts2(i2) = t2;
    i2 = i2 + 1;
end
for t2=-t_b_q2:0.01:0 % Final Blend
    traj_q2 (i2,:) = -1/2*theta_dd_q2.*(t2).^2 + (theta_f_q2);
    ts2(i2) = t2 + t_b_q2 + (T-t_b_q2);
    i2 = i2 + 1;
end
%%
theta_i_q3 = invmatrix (1,:); % Choosing All q1 Possible Configuration at The Initial Position (With Respect to q2)
theta_f_q3 = invmatrix (3,:); % Choosing All q1 Possible Configuration at The Final Position (With Respect to q2)
t_b_q3 = 0.8; % Blend Time
theta_dd_q3 = -1.*(theta_f_q3 - theta_i_q3)./(t_b_q3.^2 - T .* t_b_q3); % Acceleration of q1
theta_b_q3 = theta_i_q3 + 1/2 * theta_dd_q3.*t_b_q3.^2; % Blend Magnitude
traj_q3=[0,0,0,0];
b3 = theta_b_q3 - theta_i_q3; % Blend Offest
m3 = ((theta_f_q3-b3)-(theta_i_q3 + b3))./(T - 2.*t_b_q3); % Linear Slope
i3=1;
ts3 = 0;
for t3=0:0.01:t_b_q3 % First Blend
    traj_q3 (i3,:) = theta_i_q3 + 1/2.*theta_dd_q3*t3^2;
    ts3(i3) = t3;
    i3 = i3 + 1;
end
for t3=t_b_q3+0.01:0.01:T-t_b_q3 % Linear Part
    traj_q3 (i3,:) = theta_b_q3 + m3.*(t3-t_b_q3);
    ts3(i3) = t3;
    i3 = i3 + 1;
end
for t3=-t_b_q3:0.01:0 % Final Blend
    traj_q3 (i3,:) = -1/2.*theta_dd_q3.*(t3)^2 + (theta_f_q3);
    ts3(i3) = t3 + t_b_q3 + (T-t_b_q3);
    i3 = i3 + 1;
end

%%
set(gcf,'color','w');
Theta_1 = traj_q1;
Theta_2 = traj_q2;
Theta_3 = traj_q3;
x_ee = (cos(Theta_1).*(7*cos(Theta_2 + Theta_3) + 8*cos(Theta_2)))/10;
y_ee = (sin(Theta_1).*(7*cos(Theta_2 + Theta_3) + 8*cos(Theta_2)))/10;
z_ee = (7*sin(traj_q2 + traj_q3))/10 + (4*sin(traj_q2))/5 + 3/2;
for n = 1:4
    plot3 (x_ee(:,n),y_ee(:,n),z_ee(:,n),'.-')
    hold on
end
title ('End-Effector Path')
xlabel('X') 
ylabel('Y')
zlabel('Z')
legend ('Config 1','Config 2','Config 3','Config 4','Location','southeast')
grid on

%% PID Controller Design
init_Con = [theta_q1i; 0; theta_q2i(1,1); 0; theta_q3i(1,1); 0];
 %Joint 1
Kp1 = 30;
Kd1 = 20;
Ki1 = 0;

 %Joint 2
Kp2 = 40;
Kd2 = 15;
Ki2 = 30;

 %Joint 3
Kp3 = 50;
Kd3 = 15;
Ki3 = 30;

open_system('Test')
Test = sim('Test',T);

%% Plot for PID
Err = Test.Error;
CartPos = Test.Cart_Pos;
JointPos = Test.Joint_Pos;
Ceff = Test.C_Eff;
%%
set(gcf,'color','w');
plot(Err,'LineWidth',1.2)
title ('Joint Position Error')
xlabel('Time') 
ylabel('Angular Position(rad)')
legend ('Joint 1','Joint 2','Joint 3','Location','northeast')
grid on
%%
set(gcf,'color','w');
plot(Ceff,'LineWidth',1.2)
title ('Control Effort')
xlabel('Time') 
ylabel('Control Effort')
legend ('Joint 1','Joint 2','Joint 3','Location','northeast')
grid on
%%
set(gcf,'color','w');
plot(JointPos,'LineWidth',1.2)
title ('Joints Position')
xlabel('Time') 
ylabel('Joints Values')
legend ('Joint 1','Joint 2','Joint 3','Location','northeast')
grid on
%% Fuzzy Controller Design
FC1 = readfis('FC1.fis');
FC2 = readfis('FC2.fis');
FC3 = readfis('FC3.fis');
Testf = sim('Testf',T);
%%
Errf = Testf.Error1;
CartPosf = Testf.Cart_Pos1;
JointPosf = Testf.Joint_Pos1;
Ceff_f = Testf.C_Eff1;
%%
set(gcf,'color','w');
plot(Errf,'LineWidth',1.2)
title ('Joint Position Error')
xlabel('Time') 
ylabel('Angular Position(rad)')
legend ('Joint 1','Joint 2','Joint 3','Location','northeast')
grid on
%%
set(gcf,'color','w');
plot(Ceff_f,'LineWidth',1.2)
title ('Control Effort')
xlabel('Time') 
ylabel('Control Effort')
legend ('Joint 1','Joint 2','Joint 3','Location','northeast')
grid on
%%
set(gcf,'color','w');
plot(JointPosf,'LineWidth',1.2)
title ('Joints Position')
xlabel('Time') 
ylabel('Joints Values')
legend ('Joint 1','Joint 2','Joint 3','Location','northeast')
grid on