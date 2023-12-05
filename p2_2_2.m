clc;

% general parameters
global l1 l2 T w tstep; 
% ellipse/circle parameters
global S1 S2 R; 

l1 = 2.5; 
l2 = 2;  
T = 20;
w = 2*pi / T; % angular velocity 
R = 1.5;
S1 = 2.5;
S2 = 0.5;
tstep = 0.1;
time_vec = 0:tstep:3*T;


%% Ellipse trajectory

% ellipse trajectory desired position and velocity 
xd_e = 0.5 + S1*cos(pi/3)*cos(w*time_vec) - S2*sin(pi/3)*sin(w*time_vec);
yd_e = 0.5 + S1*sin(pi/3)*cos(w*time_vec) + S2*cos(pi/3)*sin(w*time_vec);
xddot_e = -S1*cos(pi/3)*w*sin(w*time_vec) -S2*sin(pi/3)*w*cos(w*time_vec);
yddot_e = -S1*sin(pi/3)*w*sin(w*time_vec) +S2*cos(pi/3)*w*cos(w*time_vec);

figure(1);
plot(xd_e, yd_e,'o');
daspect([1 1 1]);
xlabel("X - Position");
ylabel("Y - Position");
title("Desired Ellipse Trajectory - Y vs X");

% initial joint values: assume phi = theta1 = theta2 = 0
th_ic_e = [xd_e(1) - l1 - l2; yd_e(1); 0; 0; 0]; % x, y, phi, theta1, theta2

% integrate angles using ODE45
[t, Th] = ode45(@(t, th)openLoopJointSpaceControl(t, th, xddot_e,yddot_e), time_vec, th_ic_e);

% compute actual end-effector trajectory 
% xe = x + l1*cos(phi+theta1) + l2*cos(phi+theta1+theta2)
x_act_e = Th(:,1) + l1 * cos(Th(:,3) + Th(:,4)) + l2 * cos(Th(:,3) + Th(:,4) + Th(:,5)); 
% ye = y + l1*sin(phi+theta1) + l2*sin(phi+theta1+theta2)
y_act_e = Th(:,2) + l1 * sin(Th(:,3) + Th(:,4)) + l2 * sin(Th(:,3) + Th(:,4) + Th(:,5)); 

% Display the results 
figure(2);
plot(x_act_e, y_act_e,'o');
daspect([1 1 1]);
xlabel("X - Position");
ylabel("Y - Position");
title("Actual Ellipse Trajectory - Y vs X");


% actual trajectory animation
figure(3);
for i=1:length(time_vec) 
    clf('reset')
    % plot end effector pos
    plot(x_act_e(i), y_act_e(i), 'o','Color','blue'); hold on
    % plot base center
    plot(Th(i, 1), Th(i, 2), 'o','Color','black');
    % plot base forward direction 
    fdirx = [Th(i, 1) Th(i, 1)+cos(Th(i, 3))];
    fdiry = [Th(i, 2) Th(i, 2)+sin(Th(i, 3))];
    plot(fdirx, fdiry,'color', "red", 'LineWidth', 3);

    % plot links
    link1x = [Th(i, 1) Th(i, 1)+l1*cos(Th(i, 3)+Th(i, 4))];
    link1y = [Th(i, 2) Th(i, 2)+l1*sin(Th(i, 3)+Th(i, 4))];
    link2x = [Th(i, 1)+l1*cos(Th(i, 3)+Th(i, 4)) Th(i, 1)+l1*cos(Th(i, 3)+Th(i, 4))+l2*cos(Th(i, 3)+Th(i, 4)+Th(i, 5))];
    link2y = [Th(i, 2)+l1*sin(Th(i, 3)+Th(i, 4)) Th(i, 2)+l1*sin(Th(i, 3)+Th(i, 4))+l2*sin(Th(i, 3)+Th(i, 4)+Th(i, 5))];
    plot(link1x, link1y,'color', "#0072BD", 'LineWidth', 1);
    plot(link2x, link2y,'color', "#D95319", 'LineWidth', 1);

    title('Ellipse Trajectory Animation');
    grid on;
    
    xlim([-5 5]);
    ylim([-5 5]);
    drawnow;
end






% square trajectory 
time_vec_side_normalized = 0:tstep/(T/4):1;
time_vec_side_normalized(end)=[]; % remove last item to avoid corner duplication 
side = 3.5;
side_min = 1 - side / 2;
side_max = 1 + side / 2;
% POSITION
% bottom side
xd_s = side_min + time_vec_side_normalized * (side_max - side_min);
yd_s = side_min + time_vec_side_normalized * 0;
% right side 
xd_s = [xd_s, side_max + time_vec_side_normalized * 0];
yd_s = [yd_s, side_min + time_vec_side_normalized * (side_max - side_min)];
% top side
xd_s = [xd_s, side_max - time_vec_side_normalized * (side_max - side_min)];
yd_s = [yd_s, side_max + time_vec_side_normalized * 0]; 
% left side 
xd_s = [xd_s, side_min + time_vec_side_normalized * 0];
yd_s = [yd_s, side_max - time_vec_side_normalized * (side_max - side_min)];

step_vel = (xd_s(2) - xd_s(1)) / tstep;
% VELOCITY
% bottom side 
xddot_s = step_vel * ones(1, length(time_vec_side_normalized));
yddot_s = zeros(1, length(time_vec_side_normalized));
% right side 
xddot_s = [xddot_s, zeros(1, length(time_vec_side_normalized))];
yddot_s = [yddot_s, step_vel * ones(1, length(time_vec_side_normalized))];
% top side
xddot_s = [xddot_s, -step_vel * ones(1, length(time_vec_side_normalized))];
yddot_s = [yddot_s, zeros(1, length(time_vec_side_normalized))];
% left side 
xddot_s = [xddot_s, zeros(1, length(time_vec_side_normalized))];
yddot_s = [yddot_s, -step_vel * ones(1, length(time_vec_side_normalized))];

% three cycles 
xd_s = [xd_s, xd_s, xd_s];
yd_s = [yd_s, yd_s, yd_s];
xddot_s = [xddot_s, xddot_s, xddot_s];
yddot_s = [yddot_s, yddot_s, yddot_s];
% add last corner for size consistency with time_vec
xd_s = [xd_s, xd_s(1)];
yd_s = [yd_s, yd_s(1)];
xddot_s = [xddot_s, xddot_s(1)];
yddot_s = [yddot_s, yddot_s(1)];

figure(4);
plot(xd_s, yd_s,'o');
daspect([1 1 1]);
xlabel("X - Position");
ylabel("Y - Position");
title("Desired Square Trajectory - Y vs X");



% initial joint values: assume phi = theta1 = theta2 = 0
th_ic_s = [xd_s(1) - l1 - l2; yd_s(1); 0; 0; 0]; % x, y, phi, theta1, theta2


% integrate angles using ODE45
[t, Th] = ode45(@(t, th)openLoopJointSpaceControl(t, th, xddot_s,yddot_s), time_vec, th_ic_s);

% compute actual end-effector trajectory 
% xe = x + l1*cos(phi+theta1) + l2*cos(phi+theta1+theta2)
x_act_s = Th(:,1) + l1 * cos(Th(:,3) + Th(:,4)) + l2 * cos(Th(:,3) + Th(:,4) + Th(:,5)); 
% ye = y + l1*sin(phi+theta1) + l2*sin(phi+theta1+theta2)
y_act_s = Th(:,2) + l1 * sin(Th(:,3) + Th(:,4)) + l2 * sin(Th(:,3) + Th(:,4) + Th(:,5)); 

% Display the results 
figure(5);
plot(x_act_s, y_act_s,'o');
daspect([1 1 1]);
xlabel("X - Position");
ylabel("Y - Position");
title("Actual Square Trajectory - Y vs X");

% actual trajectory animation
figure(6);
for i=1:length(time_vec) 
    clf('reset')
    % plot end effector pos
    plot(x_act_s(i), y_act_s(i), 'o','Color','blue'); hold on
    % plot base center
    plot(Th(i, 1), Th(i, 2), 'o','Color','black');
    % plot base forward direction 
    fdirx = [Th(i, 1) Th(i, 1)+cos(Th(i, 3))];
    fdiry = [Th(i, 2) Th(i, 2)+sin(Th(i, 3))];
    plot(fdirx, fdiry,'color', "red", 'LineWidth', 3);

    % plot links
    link1x = [Th(i, 1) Th(i, 1)+l1*cos(Th(i, 3)+Th(i, 4))];
    link1y = [Th(i, 2) Th(i, 2)+l1*sin(Th(i, 3)+Th(i, 4))];
    link2x = [Th(i, 1)+l1*cos(Th(i, 3)+Th(i, 4)) Th(i, 1)+l1*cos(Th(i, 3)+Th(i, 4))+l2*cos(Th(i, 3)+Th(i, 4)+Th(i, 5))];
    link2y = [Th(i, 2)+l1*sin(Th(i, 3)+Th(i, 4)) Th(i, 2)+l1*sin(Th(i, 3)+Th(i, 4))+l2*sin(Th(i, 3)+Th(i, 4)+Th(i, 5))];
    plot(link1x, link1y,'color', "#0072BD", 'LineWidth', 1);
    plot(link2x, link2y,'color', "#D95319", 'LineWidth', 1);

    title('Square Animation');
    daspect([1 1 1]);
    grid on;
    xlim([-8 5]);
    ylim([-8 5]);
    drawnow;
end





function dth = openLoopJointSpaceControl(t, th, xddot, yddot)
    global l1 l2 tstep;

    % compute Jacobian with added constraint: theta1dot = 0
    Jp = [1, 0, -l1*sin(th(3)+th(4))-l2*sin(th(3)+th(4)+th(5)), -l1*sin(th(3)+th(4))-l2*sin(th(3)+th(4)+th(5)), -l2*sin(th(3)+th(4)+th(5));
          0, 1, l1*cos(th(3)+th(4))+l2*cos(th(3)+th(4)+th(5)), l1*cos(th(3)+th(4))+l2*cos(th(3)+th(4)+th(5)), l2*cos(th(3)+th(4)+th(5));     
          -sin(th(3)), cos(th(3)), 0, 0, 0;
          0, 0, 0, 1, 0];

    % compute open-loop theta dot
    t_idx = 1+int32(t/tstep);
    dth_openloop = pinv(Jp) * [xddot(t_idx); yddot(t_idx); 0; 0]; 
    
    % no feedback
    dth = dth_openloop;
end
