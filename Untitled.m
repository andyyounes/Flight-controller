% Quadcopter control with LQR

% Define physical parameters
m = 1.0;    % mass of the quadcopter (kg)
I = 0.1;    % moment of inertia about the vertical axis (kg*m^2)
g = 9.81;   % gravitational acceleration (m/s^2)

% Define the system matrices
A = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 0 0];
B = [0 0; 1/m 0; 0 0; 0 1/I];
C = [1 0 0 0; 0 1 0 0];
D = [0 0; 0 0];

% Define the weighting matrices for LQR
Q = diag([10 1 10 1]);    % state weights
R = 0.1*eye(2);           % control weights

% Compute the optimal feedback gain with LQR
[K, S, ~] = lqr(A, B, Q, R);

% Define the closed-loop system with the optimal gain
Ac = (A - B*K);
Bc = B;
Cc = C;
Dc = D;

% Define the initial condition and simulation time
x0 = [0; 0; 0; 0];
tf = 10;

% Simulate the closed-loop system with a step input
t = linspace(0, tf, 1000);
r = ones(size(t));
r = [r' 0*r']';  % reshape r to be a matrix with 2 columns
[y, t, x] = lsim(ss(Ac, Bc, Cc, Dc), r, t, x0);

% Plot the results
figure();
plot(t, y(:,1), 'b-', 'LineWidth', 2);
hold on;
plot(t, y(:,2), 'r-', 'LineWidth', 2);
grid on;
legend('Position (m)', 'Angle (rad)');
xlabel('Time (s)');
ylabel('State');
title('Quadcopter Control with LQR');

% Add titles to individual lines
title('Quadcopter Position and Angle');

