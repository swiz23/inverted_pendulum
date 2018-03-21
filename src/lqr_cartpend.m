clear all, close all, clc

m = .160;
M = 5;
L = .84;
g = 9.81;
d = 1;

s = -1; % pendulum up (s=1)

A = [0 1 0 0;
    0 0 0 0;
    0 0 0 1;
    0 0 g/L 0];

B = [0; 1; 0; -1/L];
eig(A)

Q = [.3 0 0 0;
    0 .001 0 0;
    0 0 100 0;
    0 0 0 .1];
R = 1;

%%
det(ctrb(A,B))

%%
K = lqr(A,B,Q,R)

tspan = 0:.001:30;
if(s==-1)
    y0 = [0; 0; 0; -1];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K*(y-[0; 0; 0; 0])),tspan,y0);
elseif(s==1)
    y0 = [-3; 0; pi+.1; 0];
% % [t,y] = ode45(@(t,y)((A-B*K)*(y-[0; 0; pi; 0])),tspan,y0);
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K*(y-[1; 0; pi; 0])),tspan,y0);
else
    
end

for k=1:100:length(t)
    drawcartpend_bw(y(k,:),m,M,L);
end

% function dy = pendcart(y,m,M,L,g,d,u)