% This script tries to find the pose of 3D a rigid body

% Get the rigid body
X = rigid_body(2);

P = get_camera();

% Generate rotation and translation
R = rotate_y(30*pi/180) * rotate_z(45*pi/180);
t = translate(0, 0, 5);

T = t * R;

% Coorindates in 3D
Y = (T * R * [X ones(size(X,1),1)]')';

% Project Y to camera
Xb = (P * Y')';

Xb = Xb ./ Xb(:,3);

scatter(X(:,1), X(:,2), 'b');
hold on;
scatter(Xb(:,1), Xb(:,2), 'g');
hold off;