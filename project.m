clear all;close all;clc
%% Define system
Ac_low_l = [0 -7.492 0.7985
            0 74.9266 -33.7147
            0 -59.9373 52.1208];
Ac_low_r = zeros(3,3);
Ac = [zeros(3,3),eye(3,3);
      Ac_low_l,Ac_low_r];

Bc = [zeros(3,1);-0.6070;1.4984;-0.2839];

Cc = [eye(3,3),zeros(3,3)];

Dc = 0;

sys = ss(Ac,Bc,Cc,Dc);
sysd = c2d(sys,0.1);
[Ad,Bd,Cd,Dd] = ssdata(sysd);
%% MPC controller formulation
nx = 6;
nu = 1;
N = 70;
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x_init = sdpvar(6,1);
objective = 0;
constraints = [x{1}==x_init];
for k = 1:N
    
    Q = [150 0 0 0 0 0;
        0 150 0 0 0 0;
        0 0 150 0 0 0;
        0 0 0 0 0 0;
        0 0 0 0 0 0;
        0 0 0 0 0 0;];
    R = 1/144;

  


constraints = constraints + [x{k+1} == Ad*x{k} + Bd*u{k}];
constraints = constraints + [-0.5<=x{1,k}<=0.5];
objective = objective + x{k}'*Q*x{k} + u{k}'*R*u{k};
%objective = objective + norm(x{k},2) + norm(u{k},2);
end
parameters_in = {x_init};
solutions_out = {[u{:}],[x{:}]};
%controller_psr = optimizer(constraints, objective,sdpsettings('solver','gurobi'),parameters_in,solutions_out);
controller_psr = optimizer(constraints, objective,[],parameters_in,solutions_out);
%% Closed loop simulation
x_init_in = [0.02;deg2rad(3);deg2rad(4);0;0;0];

for time = 0:1:70
   
    inputs = {x_init_in};
    [solutions, diagnostics] = controller_psr{inputs};
    U = solutions{1};
    X = solutions{2};

    
    x_optimal(:,time+1)= x_init_in;
    x_init_in = X(:,2);
    u_optimal(time+1)= U(1);
    
end

t = 0:0.1:7;
plot(t,rad2deg(x_optimal(2,:)))
xlabel('time [s]')
ylabel('lower angle [degree]')
title('lower angle')
figure
plot(t,rad2deg(x_optimal(3,:)))
xlabel('time [s]')
ylabel('upper angle [degree]')
title('upper angle')