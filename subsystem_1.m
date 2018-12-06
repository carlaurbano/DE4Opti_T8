%% OPTIMISATION WITH FMINCON
%initial guess : using values from already existing system with similar
%parameters
carriersGuess = 10;
velocityGuess = 1;
diameterGuess = 0.01;
seatsGuess = 2;

%load guess values into array
x0 = [carriersGuess, velocityGuess, diameterGuess, seatsGuess];

% Bounds: adding lower and upper bounds to constraint the design variables
% carriers are bounded in terms of efficiency
% velocity is bounded in terms of logistics, although this will be altered in the combination of subsystems 
% diameter is bounded in terms of availability in the market
lb = [1, 1, 0.0005,2];
ub = [50, 10, 0.01,6];

%call solver to minimise
Minimisers_fmincon = fmincon(@objective, x0, [],[],[],[],lb,ub,@constraint)

%retrieve optimized 
MaximumCapacity_fmincon = calcCapacity(Minimisers_fmincon)

%% OPTIMISATION WITH FMINCON updated
%In this case, once the results are shown, the variable x1 should be a
%discrete variable, that is why in this updated version, the variable can
%just take the value of the optimised solution from the previous one (x1 = 19 carriers)
lb_2 = [1, 1, 0.0005,2];
ub_2 = [19, 10, 0.01,6];

Minimisers_fmincon_updated = fmincon(@objective, x0, [],[],[],[],lb_2,ub_2,@constraint)

%retrieve optimized 
MaximumCapacity_fmincon_updated = calcCapacity(Minimisers_fmincon_updated)


%% OPTIMISATION WITH GENETIC ALGORITHM
% This is the approach taken to find the minimisers and the minised
% ojective function including discrete variables.

% Bounds: adding lower and upper bounds to constraint the design variables
% carriers are bounded in terms of efficiency
% velocity is bounded in terms of logistics, although this will be altered in the combination of subsystems 
% diameter is bounded in terms of availability in the market

lb = [1, 1, 0.005,2];
ub = [200, 10, 0.01,6];

nVar = 4; % there are four variables
%call solver to minimise

Minimisers_ag = ga(@objective, nVar, [],[],[],[],lb,ub,@constraint,[1 4])

%retrieve optimized 
MaximumCapacity_ga = calcCapacity(Minimisers_ag)

%calculate constraint
%% FUNCTIONS

%% objective
function capacity = calcCapacity(x)
    carriers = x(1);
    velocity = x(2);
    diameter = x(3);
    seats = x(4);
    capacity = x(1)*3600/x(2)*x(4);
end
%% constraint 1
function interval = calcInterval(x)
     %% variables
    carriers = x(1);
    velocity = x(2);
    diameter = x(3);
    seats = x(4);
    %% parameters
    L = 1000; %overall length (m)
    h = sqrt(3.)*L/3; %overall height (m)
    theta = tan(h/L); %slope
    %% constraint g1
    interval = (L*cos(theta))/(x(1)*x(2));
end 

%% constraint 2
function support = calcSupport(x)
    %% variables
    carriers = x(1);
    velocity = x(2);
    diameter = x(3);
    seats = x(4);
    %% parameters
    L = 1000; %overall length (m)
    h = sqrt(3.)*L/3; %overall height (m)
    theta = tan(h/L); %slope
    rho = 8050; %density of steel (kg/m^3)
    E = 200*10.^9; %Young's Modulus of steel (kg/ms^2)
    g = 9.8;
    qs = 392 + 20*g; %load of seat (N)
    qp = 784; %load of passenger (N)
    delta = 0.2; %deflection (m)
    k = delta*pi*E/(L*4); %constant 
    %% constraint g2
    support = x(4)*0.5*(x(1) + 1)*L*(qs + qp) - x(3).^2*(k*(cos(theta)*h + sin(theta)*L) + L*pi*rho*g*0.5);
end

%% obj
function obj = objective(x)
    obj = -calcCapacity(x);
end

%% c, ceq
function [c, ceq] = constraint(x)
    N = 2500*9.8;
    L = 1000;
    c = [calcInterval(x) - 5; calcSupport(x) - N*L];
    ceq = [];
end



