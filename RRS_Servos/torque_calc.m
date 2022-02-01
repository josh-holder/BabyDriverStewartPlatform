%Determine what torque and RPM we need to achieve our accelerations

%% Z accleration torque calculation
a = 2.5;
t = .25;
tstep = .001;
zdots = a*tstep:a*tstep:a*t;

theta_curr = pi/2;
pos_curr = 0;
theta_dots = [];
thetas = [];
positions = [];
for zdot = zdots
    theta_dot_curr = findThetaDot(zdot,theta_curr);
    theta_dots = [theta_dots theta_dot_curr];
    theta_curr = theta_curr + theta_dot_curr*tstep;
    thetas = [thetas theta_curr];
    pos_curr = pos_curr + zdot*tstep;
    positions = [positions pos_curr];
end
figure(1)
subplot(2,2,2)
plot(tstep:tstep:tstep*length(zdots),theta_dots./(2*pi).*60)
ylabel("$\dot{\theta}$ (rpm)",'Interpreter','Latex','FontSize',14)
xlabel("Time (s)",'FontSize',14)
subplot(2,2,1)
plot(tstep:tstep:tstep*length(zdots),thetas.*180./pi)
ylabel("$\theta$ (deg)",'Interpreter','Latex','FontSize',14)
xlabel("Time (s)",'FontSize',14)
subplot(2,2,3)
plot(tstep:tstep:tstep*length(zdots),positions)
ylabel("Z position (m)",'FontSize',14)
xlabel("Time (s)",'FontSize',14)

%calculate torque
theta_change = thetas(1) - thetas(end);
m = 20;
v_final = zdots(end);
num_motors = 3;

avg_torque = ((.5*m*v_final^2)+m*9.81*positions(end))/num_motors/theta_change

taus = [0];
max_tau = 0;
vel_at_max_tau = 0;
for i = 2:length(thetas)
    
    curr_tau = abs((.5*m*(zdots(i)^2-zdots(i-1)^2) + (positions(i)-positions(i-1))*m*9.81)/(thetas(i)-thetas(i-1))/num_motors);
    if  curr_tau > max_tau
        max_tau = curr_tau;
        vel_at_max_tau = theta_dots(i);
    end
    taus = [taus curr_tau];    
end
subplot(2,2,4)
plot(tstep:tstep:tstep*length(zdots),taus)
ylabel("Torque (N*m)",'FontSize',14)
xlabel("Time (s)",'FontSize',14)
max_tau
vel_at_max_tau/2/pi*60
max_rpm = max(abs(theta_dots./(2*pi).*60))


%% Angular Acceleration
% plat_rad = .508/2;
% alpha = 1.5; %rad/s^2
% omegas = alpha*tstep:alpha*tstep:alpha*t;
% 
% theta_curr = pi/2;
% pos_curr = 0;
% theta_dots = [];
% thetas = [];
% positions = [];
% 
% theta_dots = [];
% thetas = [];
% positions = [];
% for omega = omegas
%     %calculate corresponding linear velocity 
%     v_curr = omega*plat_rad;
%     theta_dot_curr = findThetaDot(v_curr,theta_curr);
%     theta_dots = [theta_dots theta_dot_curr];
%     theta_curr = theta_curr + theta_dot_curr*tstep;
%     thetas = [thetas theta_curr];
%     pos_curr = pos_curr + zdot*tstep;
%     positions = [positions pos_curr];
% end
% figure(4)
% plot(tstep:tstep:tstep*length(zdots),theta_dots./(2*pi).*60)
% ylabel("$\dot{\theta}$ (rpm)",'Interpreter','Latex')
% xlabel("time")
% figure(5)
% plot(tstep:tstep:tstep*length(zdots),thetas.*180./pi)
% ylabel("theta (deg)")
% xlabel("time")
% figure(6)
% plot(tstep:tstep:tstep*length(zdots),positions)
% ylabel("z pos")
% xlabel("time")

%calculate torque
% theta_change = thetas(1) - thetas(end);
% m = 30;
% v_final = omegas(end)*plat_rad;
% num_motors = 3;
% 
% torque = (.5*m*v_final^2)/num_motors/theta_change
% max_rpm = max(abs(theta_dots./(2*pi).*60))

function theta_dot = findThetaDot(zdot,theta)
    a_len = 0.1;
    s_len = 0.4;
    
    theta_dot = zdot / (-a_len*sin(theta) + 0.5*s_len/sqrt(1-a_len^2/s_len^2*sin(theta)^2)*(-2*a_len^2/s_len^2*sin(theta))*cos(theta));
end