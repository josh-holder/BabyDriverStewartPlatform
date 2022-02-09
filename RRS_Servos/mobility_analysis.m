clear all
clf
%Formal analysis
r_base = .508;
pz = .51;

gamma = 0;
beta = deg2rad(8.5);
% alpha = 0;

% vx = sin(alpha)*sin(beta)*cos(gamma) - cos(alpha)*sin(gamma);
% vy = sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma);
% ux = cos(beta)*cos(gamma);
% uy = cos(beta)*sin(gamma);

%px = px_coef_alpha
pxs = [];
pys = [];
for alphad = 0:45
    alpha = deg2rad(alphad);
    px = 3/2*r_base*(sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma))+1/2*r_base*cos(beta)*cos(gamma)-2*r_base;
    py = -1/2*r_base*cos(beta)*sin(gamma)+1/2*r_base*(sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma));
    
    pxs = [pxs px];
    pys = [pys py];
end

figure(1)
hold on
plot(0:45,pxs)
plot(0:45,pys)
xlabel("Alpha (deg)")
ylabel("position (m)")
legend("px","py")
hold off

%%Equations for non universal joint
% px = r_base - r_base*ux;
% py = 1/2*(r_base*uy+r_base*vx);
% alpha = asin(-ux+2-cos(alpha)*cos(gamma)

