function [flight_path, ref] = ellipse_flight_path(r, a, b, az, el, tilt_long, tilt_lat,steps, dir) 

%% Parameters
% r = 135;
% a = 65.5;
% b = 65.5;
% 
% az = deg2rad(-5.5);
% el = deg2rad(-12);
% tilt_long = deg2rad(0);
% tilt_lat = deg2rad(0);
% 
% step_sz = .01;


%% Planar Ellipse
theta = pi:-pi/(steps/2):0;
%theta = theta(1:(end-1));

% y1 = -b:step_sz:b;
% y2 = b:-step_sz:-b;
% 
% y2 = y2(2:end-1);

y1 = a*cos(theta);
y2 = flip(y1); y2 = y2(2:end-1);
y = [y1, y2];

z = [sqrt((1-y1.^2/a^2)*b^2), -sqrt((1-y2.^2/a^2)*b^2)];
x = zeros(1, length(y));
pts = [x; y; z];
%% Rotation Tilt
R_y = [cos(tilt_long), 0, sin(tilt_long); 0, 1, 0; -sin(tilt_long), 0, cos(tilt_long)];
R_z = [cos(tilt_lat), -sin(tilt_lat), 0; sin(tilt_lat), cos(tilt_lat), 0; 0, 0, 1];
pts = R_y*R_z*pts;

%% Move Out
pts(1, :) = pts(1, :)+r;
R_el = [cos(el), 0, sin(el); 0, 1, 0; -sin(el), 0, cos(el)];
R_az = [cos(az), -sin(az), 0; sin(az), cos(az), 0; 0, 0, 1];
pts = R_el*R_az*pts;

%% Plot
figure(1);
hold on
scatter3(pts(1, :), pts(2, :), pts(3, :), 1);
axis equal;
xlabel('x');
ylabel('y');
zlabel('z');
view([-90, 0]);
zlim([-80, 75]);
ylim([-85, 85]);
set ( gca, 'ydir', 'reverse' );
set ( gca, 'zdir', 'reverse' )


%% Tangent

if dir == 1
vel_dir = normalize(([pts(:, 2:end), pts(:, 1)] - pts), 'norm');
else
vel_dir = -normalize(([pts(:, 2:end), pts(:, 1)] - pts), 'norm');
end

pos_norm = normalize(pts, 'norm');
in_plane_vel = vel_dir - dot(vel_dir, pos_norm).*pos_norm;
in_plane_ref = ones(size(pos_norm)).*[0, 0, -1]';
in_plane_ref = in_plane_ref - dot(in_plane_ref, pos_norm).*pos_norm;
in_plane_ref = normalize(in_plane_ref, 'norm');

sign_val = cross(in_plane_ref, in_plane_vel);
sign_val = sign_val(1, :)./abs(sign_val(1, :));

gamma = dot(in_plane_vel, in_plane_ref)./(sqrt(dot(in_plane_vel, in_plane_vel)).*sqrt(dot(in_plane_ref, in_plane_ref)));
gamma = sign_val.*real(acos(gamma));


%% Center
center = R_el*R_az*[r, 0, 0]';
pos_center = pts - center;
ref_vec = ones(size(pos_center)).*(R_el*R_az*([r, 0, 0]'+R_y*R_z*[0, 0, -b]') - center);

sign_val = cross(ref_vec, pos_center);
sign_val = sign_val(1, :)./abs(sign_val(1, :));

lambda = dot(pos_center, ref_vec)./(sqrt(dot(pos_center, pos_center)).*sqrt(dot(ref_vec, ref_vec)));
lambda = sign_val.*real(acos(lambda));


%% Get Points
flight_path = [pts; vel_dir; gamma; lambda]';
[~, I] = sort(lambda);
flight_path = flight_path(I, :);
flight_path(1, 8) = -pi;
flight_path(end, 8) = pi;

ref = [center, ref_vec(:, 1)];
end
