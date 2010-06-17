% test for the joint limiter code

format long;

v0 = 100;       % initial velocity
acc_max = 100;  % maximum deceleration
T = 2;          % time in which to simulate

dt_real = 0.0001;  % timestep for "ideal" behaviour
dt_sim =  0.1;

% compute desired behavior
t=0; x=0;
desired = zeros(T/dt_real,3);
for i = 1:T/dt_real
  v = v0 - acc_max*t;  % decelerate with maximum deceleration

  if v < 0
    v = 0;
  end

  desired(i,1) = v;
  desired(i,3) = x;
  desired(i,2) = t;

  t = t + dt_real;
  x = x + v*dt_real;
end

x % after this distance be braked with max. deceleration

% test limiter code
% this code always computes the maximum allowed velocity 
% so that we can still brake within limits.
t=0; x=0;
limit = 50;

computed = zeros(T/dt_sim,3);
for i = 1:T/dt_sim

  a_m = acc_max*dt_sim*dt_sim;

  c = 1.0; % number of timestamps to look ahead
  
  dist_to_limit = limit - x;
  if dist_to_limit > 0
    v = (-a_m*c + sqrt(a_m*(2*dist_to_limit + a_m*c*c)) )/dt_sim;
  else
    v = v - acc_max*dt_sim;
  end

  if v < 0
    v = 0;
  end

  computed(i,1) = v;
  computed(i,3) = x;
  computed(i,2) = t;

  t = t + dt_sim;
  x = x + v*dt_sim;
end

x_brake = x % after this distance we braked using our algorithm

plot(desired(:,2), desired(:,1), 'b');
hold on;
plot(desired(:,2), desired(:,3), 'b');

plot(computed(:,2), computed(:,1), 'r');
plot(computed(:,2), computed(:,3), 'r');


hold off;