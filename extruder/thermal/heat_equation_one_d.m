%% heat_equation_one_d.m
% MATLAB adaptation of FORTRAN code listing E.2.0.31 from Bill Goodwine's
% "Engineering Differential Equations: Theory and Applications"
%
% Description:
%   This program solves the one-dimensional heat conduction equation with
%   alpha=2, L=10. The initial heat distribution is triangular with a
%   maximum at x=5 and a maximum value of 5.

%% Setup
clear;
close all;
clc;

dim = 20;
alpha = 2;
L = 10.0;

u = zeros(dim+1,1);
un = zeros(dim+1,1);

dx = L/dim;
dt = 0.031;
tsteps = 0:dt:8; % simulate for 8 seconds

fprintf('dt should be less than %f.\n',0.5*(dx/alpha)^2);
fprintf('dt is %f.\n',dt);

%% Set the initial conditions
t = 0;

u(1) = 0;
for i = 1:dim
    if i < dim/2
        u(i+1) = 2*5*(i/dim);
    else
        u(i+1) = 5 - 2*5*(i - dim/2)/dim;
    end
end

figure;plot(0:dx:L,u,'k.-');
xlabel('x');ylabel('u(x,0)');
title('initial temperature profile');

%% Step forward in time
u_rec = zeros(numel(tsteps),dim+1);
for i = 1:numel(tsteps)
    % save a record of the current temperature profile before simulating
    % the temperature profile at the next timestep:
    u_rec(i,:) = u;
    for j = 2:dim % the first and last (x=0 and x=L) are unchanged
       un(j) = u(j) + (alpha^2)*(u(j+1) - 2*u(j) + u(j-1))*(dt/(dx^2));
       u(j) = un(j); % update for next time
    end
end

%% Animate
plot(0:dx:L,u_rec(1,:),'k.-')
xlabel('x');ylabel('u(x,t)');
title(['temperature profile, t = ',num2str(tsteps(1)),' [s].']);
axis([0 L 0 5])
pause;
for i = 2:(numel(tsteps))
    plot(0:dx:L,u_rec(i,:),'k.-')
    title(['temperature profile, t = ',num2str(tsteps(i)),' [s].']);
    axis([0 L 0 5])
    drawnow;
%     pause(0.1);
end