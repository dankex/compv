% This example loads a scene and two poses
% It then animates between the two poses in a naive way
%  (linear interpolation between angle values)
 
hand_system_setup;
 
scene_file = open_dialog('*.yml', 'Please choose a scene_spec file');
if isempty(strfind(scene_file, 'yml'))
    scene_file = 'hand_model/scene_spec.yml'
end

load_scene(scene_file);
 
% Let the user choose a starting pose
pose1_file = open_dialog('*.yml', 'Please choose a starting pose');
if isempty(strfind(pose1_file, 'yml'))
    pose1_file = 'poses/stop_it.yml'
end

% Load the starting pose
[pose1, rotation1] = load_hand_pose(pose1_file);
 
% In the same way, we choose an ending pose
pose2_file = open_dialog('*.yml', 'Please choose a final pose');
if isempty(strfind(pose2_file, 'yml'))
    pose2_file = 'poses/a_ok.yml';
end

% Load it. We'll ignore the rotation for pose2
[pose2, rotation2] = load_hand_pose(pose2_file);
 
% alpha is going to be our linear blending factor
for alpha = [0:0.05:1]
     
    % Compute the intermediate pose as a linear interpolation
    pose_i = pose1 + (pose2 - pose1) * alpha;
    rotation_i = rotation1 + (rotation2 - rotation1) * alpha;
    
    % Render it and show it
    set_hand_pose(pose_i, rotation_i);    
    imshow(render_hand);
 
    % Force MATLAB to draw the figure every time through the loop
    drawnow
 
end
