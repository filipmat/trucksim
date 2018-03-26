%%
filename = 'out00.txt';

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
velocity = C{11}(~standstill_indices);
gear = C{12}(~standstill_indices);

alpha = atan(yaw_rate.*0.33./v);

%%
left = yaw_rate > 0;

steering_left = steering(left);
steering_right = steering(~left);
r_left = r(left);
r_right = r(~left);
yaw_rate_left = yaw_rate(left);
yaw_rate_right = yaw_rate(~left);
alpha_left = alpha(left);
alpha_right = alpha(~left);

forward = velocity < 1500;

v_forward = v(forward);
velocity_forward = velocity(forward);
a_forward = a(forward);

% Steering as function of wheel angle.
angle_left_k = [alpha_left alpha_left.^0] \ steering_left;
angle_right_k = [alpha_right alpha_right.^0] \ steering_right;

% Wheel angle as function of steering. 
steering_right_k = [steering_right steering_right.^0] \ alpha_right;
steering_left_k = [steering_left steering_left.^0] \ alpha_left;

% Throttle as function of speed. 
v_forward_k = [v_forward v_forward.^0] \ velocity_forward;


%%
v_forward_sorted = sort(v_forward);
v_forward_test = v_forward_k(1)*v_forward_sorted + ...
    v_forward_k(2)*v_forward_sorted.^0;

figure
plot(v_forward, velocity_forward, '.')
hold on
plot(v_forward_sorted, v_forward_test, 'r')

%%
alpha_right_sorted = sort(alpha_right);
steering_right_test = angle_right_k(1)*alpha_right_sorted + ...
    angle_right_k(2)*alpha_right_sorted.^0;

figure
plot(alpha_right, steering_right, '.')
hold on
plot(alpha_right_sorted, steering_right_test, 'r')

%%
alpha_left_sorted = sort(alpha_left);
steering_left_test = angle_left_k(1)*alpha_left_sorted + ...
    angle_left_k(2)*alpha_left_sorted.^0;

figure
plot(alpha_left, steering_left, '.')
hold on
plot(alpha_left_sorted, steering_left_test, 'r')

