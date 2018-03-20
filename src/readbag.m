%%
filename = 'output06.txt';

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

v_f = v(forward);
velocity_f = velocity(forward);
a_f = a(forward);

% Steering as function of wheel angle.
angle_left_k = [alpha_left alpha_left.^0] \ steering_left;
angle_right_k = [alpha_right alpha_right.^0] \ steering_right;

% Wheel angle as function of steering. 
srk = [steering_right steering_right.^0] \ alpha_right;
slk = [steering_left steering_left.^0] \ alpha_left;

% Throttle as function of speed. 
speedf_k = [v_f v_f.^0] \ velocity_f;


%%
vvf = sort(v_f);
vuf = speedf_k(1)*vvf + speedf_k(2)*vvf.^0;

figure
plot(v_f, velocity_f, '.')
hold on
plot(vvf, vuf, 'r')

%%
ar = sort(alpha_right);
sr = angle_right_k(1)*ar + angle_right_k(2)*ar.^0;

figure
plot(alpha_right, steering_right, '.')
hold on
plot(ar, sr, 'r')

%%
al = sort(alpha_left);
sl = angle_left_k(1)*al + angle_left_k(2)*al.^0;

figure
plot(alpha_left, steering_left, '.')
hold on
plot(al, sl, 'r')

