%%
filename = 'model_measurement_0326_trx_auto_02.txt';

fid = fopen(filename);
firstline = fgetl(fid);
fclose(fid);
var_names = strsplit(erase(firstline, ["%", "field."]), ',');

fileID = fopen(filename);
C = textscan(fileID,'%f %s %f %f %f %f %f %f %f %f %f %f', ...
    'Delimiter', ',', 'HeaderLines', 1);
fclose(fileID);

standstill_indices = C{6} == 0 & C{7} == 0;   % Standstill. 

time = C{1}(~standstill_indices);
time = (time - time(1))/1000000000;

x = C{3}(~standstill_indices);
y = C{4}(~standstill_indices);
yaw = C{5}(~standstill_indices);
yaw_rate = C{6}(~standstill_indices);
v = C{7}(~standstill_indices);
a = C{8}(~standstill_indices);
r = C{9}(~standstill_indices);
steering = C{10}(~standstill_indices);
throttle = C{11}(~standstill_indices);
gear = C{12}(~standstill_indices);

alpha = atan(yaw_rate.*0.33./v);

%%
%left = and(yaw_rate > 0, steering < 1500);
left = and(alpha > 0, and(alpha < 0.55, steering < 1500));
%right = and(yaw_rate < 0, steering > 1520);
right = and(alpha < 0, and(alpha > -0.6, steering > 1520));

steering_left = steering(left);
steering_right = steering(right);
r_left = r(left);
r_right = r(right);
yaw_rate_left = yaw_rate(left);
yaw_rate_right = yaw_rate(right);
alpha_left = alpha(left);
alpha_right = alpha(right);

forward = and(throttle > 1500, v < 3);

v_forward = v(forward);
throttle_forward = throttle(forward);
a_forward = a(forward);

% Steering as function of wheel angle.
angle_left_k = [alpha_left.^0 alpha_left alpha_left.^2] \ steering_left;
angle_right_k = [alpha_right.^0 alpha_right alpha_right.^2] \ ...
    steering_right;

% Wheel angle as function of steering. 
steering_right_k = [steering_right.^0 steering_right steering_right.^2] ...
    \ alpha_right;
steering_left_k = [steering_left.^0 steering_left steering_left.^2] ...
    \ alpha_left;

% Throttle as function of speed. 
v_forward_k = [v_forward.^0 v_forward v_forward.^2] \ throttle_forward;

% Speed as function of throttle
throttle_forward_k = [throttle_forward.^0 throttle_forward ...
    throttle_forward.^2] \ v_forward;

%% speed to throttle input
v_forward_sorted = sort(v_forward);
throttle_forward_test = v_forward_k(1)*v_forward_sorted.^0 + ...
    v_forward_k(2)*v_forward_sorted + ...
    v_forward_k(3)*v_forward_sorted.^2;

figure
plot(v_forward, throttle_forward, '.')
hold on
plot(v_forward_sorted, throttle_forward_test, 'r')

%% right turn (wheel angle < 0) to steering input
alpha_right_sorted = sort(alpha_right);
steering_right_test = angle_right_k(1)*alpha_right_sorted.^0 + ...
    angle_right_k(2)*alpha_right_sorted + ...
    angle_right_k(3)*alpha_right_sorted.^2;

figure
plot(alpha_right, steering_right, '.')
hold on
plot(alpha_right_sorted, steering_right_test, 'r')

%% left turn (wheel angle > 0) to steering input
alpha_left_sorted = sort(alpha_left);
steering_left_test = angle_left_k(1)*alpha_left_sorted.^0 + ...
    angle_left_k(2)*alpha_left_sorted + ...
    angle_left_k(3)*alpha_left_sorted.^2;

figure
plot(alpha_left, steering_left, '.')
hold on
plot(alpha_left_sorted, steering_left_test, 'r')

%% throttle input to speed
throttle_forward_sorted = sort(throttle_forward);
v_forward_test = throttle_forward_k(1)*throttle_forward_sorted.^0 + ...
    throttle_forward_k(2)*throttle_forward_sorted + ...
    throttle_forward_k(3)*throttle_forward_sorted.^2;

figure
plot(throttle_forward, v_forward, '.')
hold on
plot(throttle_forward_sorted, v_forward_test, 'r')

%% right (> 1500) steering input to wheel angle
steering_right_sorted = sort(steering_right);
alpha_right_test = steering_right_k(1)*steering_right_sorted.^0 + ...
    steering_right_k(2)*steering_right_sorted + ...
    steering_right_k(3)*steering_right_sorted.^2;

figure
plot(steering_right, alpha_right, '.')
hold on
plot(steering_right_sorted, alpha_right_test, 'r')

%% left (< 1500) steering input to wheel angle
steering_left_sorted = sort(steering_left);
alpha_left_test = steering_left_k(1)*steering_left_sorted.^0 + ...
    steering_left_k(2)*steering_left_sorted + ...
    steering_left_k(3)*steering_left_sorted.^2;

figure
plot(steering_left, alpha_left, '.')
hold on
plot(steering_left_sorted, alpha_left_test, 'r')

