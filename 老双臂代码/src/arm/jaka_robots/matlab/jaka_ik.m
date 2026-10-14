close;
% clear;
% clc;

jaka_zu3_model;
% jaka_zu5_model;
% jaka_minicobo_model;

% 起始位姿
config = homeConfiguration(jaka);
config(1).JointPosition = deg2rad(0);
config(2).JointPosition = deg2rad(0);
config(3).JointPosition = deg2rad(0);
config(4).JointPosition = deg2rad(0);
config(5).JointPosition = deg2rad(0);
config(6).JointPosition = deg2rad(0);

% ==================== 运动控制 ====================
% 到达位姿
co_X = end_pos(1); 
co_Y = end_pos(2); 
co_Z = end_pos(3);
co_roll = end_pos(4);
co_pitch = end_pos(5);
co_yaw = end_pos(6);

eulZYX = [co_yaw, co_pitch, co_roll];
R = eul2rotm(eulZYX,'ZYX');
pose = [R [co_X; co_Y; co_Z]; [0 0 0 1]];
disp(pose);

ik = inverseKinematics('RigidBodyTree', jaka);
weights = [0.5 0.5 0.5 1 1 1];
qInitial = config;

try
    [qSol,qInfo] = ik('wrist_3_link', pose, weights, qInitial);
    % **************************************************
    
    % 结果显示
    fprintf('Inverse kinematics result\r\n');
    disp(qInfo);
    for i = 1:length(config)
        iviz.Configuration(i) = qSol(i).JointPosition;
        config(i).JointPosition = qSol(i).JointPosition;
    end
    fprintf('joint[1]->joint[6]\r\n');
    disp(rad2deg(wrapToPi(iviz.Configuration)'));
    disp((wrapToPi(iviz.Configuration))');
catch ME
    disp('奇异解');
end

clear co_X co_Y co_Z co_roll co_pitch co_yaw R eulZYX ...
    iviz ik qInitial pose qSol qInfo weights i config joints robot