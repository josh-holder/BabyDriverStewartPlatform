%Determine what torque and RPM we need to achieve our accelerations
clf
%% Geometry input
a = 0.1;
s = 0.5;
base_rad = .508/2;

starting_pos = sqrt(s^2-a^2);

%% Z accleration torque calculation
accel = 2.5;
t = .25;
tstep = .001;
zdots = accel*tstep:accel*tstep:accel*t;

thetas_curr = [0 0];
pos_curr = starting_pos;
thetas = [];
theta_dots = [];
positions = [];
for zdot = zdots
    pos_curr = pos_curr + zdot*tstep;
    old_theta = thetas_curr(1);
    thetas_curr = inverseKin2DOF(a,s,base_rad,0,pos_curr,0);
    thetas = [thetas thetas_curr(1)];
    theta_dots = [theta_dots (thetas_curr(1)-old_theta)/tstep];
    positions = [positions pos_curr];
end
figure(1)
subplot(2,2,2)
secs_per_60_deg = 1./theta_dots.*60;
plot(tstep:tstep:tstep*length(zdots),secs_per_60_deg)
ylabel("$\dot{\theta}$ (sec/60 deg)",'Interpreter','Latex','FontSize',14)
xlabel("Time (s)",'FontSize',14)
subplot(2,2,1)
plot(tstep:tstep:tstep*length(zdots),thetas)
ylabel("$\theta$ (deg)",'Interpreter','Latex','FontSize',14)
xlabel("Time (s)",'FontSize',14)
subplot(2,2,3)
plot(tstep:tstep:tstep*length(zdots),positions)
ylabel("Z position (m)",'FontSize',14)
xlabel("Time (s)",'FontSize',14)

%% calculate torque
theta_change = (thetas(end)-thetas(1))*pi/180;
position_change = (positions(end)-positions(1));
m = 20;
v_final = zdots(end);
num_motors = 2;

avg_torque = ((.5*m*v_final^2)+m*9.81*position_change)/num_motors/theta_change;
energy_linear = (.5*m*v_final^2)+m*9.81*position_change;

taus = [0];
max_tau = 0;
vel_at_max_tau = 0;
for i = 2:length(thetas)
    
    curr_tau = abs((.5*m*(zdots(i)^2-zdots(i-1)^2) + (positions(i)-positions(i-1))*m*9.81)/((thetas(i)-thetas(i-1))*pi/180)/num_motors);
    if  curr_tau > max_tau
        max_tau = curr_tau;
        vel_at_max_tau = secs_per_60_deg(i);
    end
    taus = [taus curr_tau];    
end
subplot(2,2,4)
plot(tstep:tstep:tstep*length(zdots),taus)
ylabel("Torque (N*m)",'FontSize',14)
xlabel("Time (s)",'FontSize',14)

secs_per_60_at_max_tau = vel_at_max_tau/2/pi*60;
min_secs_per_60_deg = min(abs(secs_per_60_deg));

disp("Average Torque: " + avg_torque + " N*m, Max Torque: " + max_tau + " N*m")
disp("Min s/60 deg: " + min_secs_per_60_deg + ", s/60 at max torque: " + secs_per_60_at_max_tau);



%% Angular Acceleration
plat_rad = .508/2;
alpha = 1.5; %rad/s^2
t = 0.25;
omegas = (alpha*tstep:alpha*tstep:alpha*t)*180/pi; % deg/s

pos_curr = .535; %m
roll_curr = -10; %deg
thetas_curr = inverseKin2DOF(a,s,base_rad,roll_curr,pos_curr,0);

rolls = [];
theta_dots = []; %2 x n matrix with each theta_dot
thetas = []; %2 x n matrix with each theta

for omega = omegas
    roll_curr = roll_curr + omega*tstep;
    old_theta = thetas_curr;
    thetas_curr = inverseKin2DOF(a,s,base_rad,roll_curr,pos_curr,0);
    rolls = [rolls roll_curr];
    thetas = [thetas thetas_curr'];
    theta_dots = [theta_dots (thetas_curr'-old_theta')/tstep];
end
figure(2)
secs_per_60_deg = 1./theta_dots.*60;
subplot(2,2,2)
hold on
plot(tstep:tstep:tstep*length(zdots),secs_per_60_deg(1,:))
plot(tstep:tstep:tstep*length(zdots),secs_per_60_deg(2,:))
hold off
ylabel("$\dot{\theta}$ (s/60)",'Interpreter','Latex')
xlabel("time")
legend("Theta1","Theta2")
legend()
subplot(2,2,1)
hold on
plot(tstep:tstep:tstep*length(zdots),thetas(1,:))
plot(tstep:tstep:tstep*length(zdots),thetas(2,:))
hold off
ylabel("theta (deg)")
xlabel("time")
legend("Theta1","Theta2")

subplot(2,2,3)
plot(tstep:tstep:tstep*length(zdots),rolls)
ylabel("roll (deg)")
xlabel("time")

%% calculate torque for angular accel
theta_change = thetas(1,1) - thetas(1,end);
m = 20/2;%dividing by two bc not each motor has to 
J = 1/4*m*base_rad^2; %approximating as simple circle of mass 20kg

% torque is difficult to calculate, so lets just calculate the total energy
% needed for angular rotation, show that it is significantly less than that
% for linear acceleration
energy_angular = 1/2*J*(alpha*t)^2;
disp("Energy needed for linear: " + energy_linear + ", energy needed for angular: " + energy_angular)

% taus = [0; 0];
% max_tau = 0;
% vel_at_max_tau = 0;
% for i = 2:length(thetas)
%     curr_taus = [];
%     for j = 1:2
%         curr_taus = [curr_taus; abs((.5*J*(theta_dots(j,i)^2-theta_dots(j,i-1)^2)));
%         if  curr_tau > max_tau
%             max_tau = curr_tau;
%             vel_at_max_tau = secs_per_60_deg(i);
%         end
%         
%     end
%     taus = [taus curr_taus];
% end
% subplot(2,2,4)
% plot(tstep:tstep:tstep*length(zdots),taus)
% ylabel("Torque (N*m)",'FontSize',14)
% xlabel("Time (s)",'FontSize',14)
% 
% secs_per_60_at_max_tau = vel_at_max_tau/2/pi*60;
% min_secs_per_60_deg = min(abs(secs_per_60_deg));
% 
% disp("Average Torque: " + avg_torque + " N*m, Max Torque: " + max_tau + " N*m")
% disp("Min s/60 deg: " + min_secs_per_60_deg + ", s/60 at max torque: " + secs_per_60_at_max_tau);