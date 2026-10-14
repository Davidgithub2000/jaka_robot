close;
clear;
clc;

dual_zu5_model;
% iviz.ShowMarker = true;
jaka.DataFormat = 'column';
% [0 45 -90 180 180 0 180 135 90 0 180 0]
% ==================== 运动控制起点 ====================
startConfig = deg2rad([0 45 -90 180 180 0 180 135 90 0 180 0]');
% **************************************************

fprintf('1.初始化双臂轨迹\r\n');
iviz.Configuration = startConfig;
ik = inverseKinematics('RigidBodyTree', jaka);
weights = [0.5 0.5 0.5 1 1 1];
qInitial = startConfig;

iviz.MarkerBodyName = "l_wrist_3_link";
iviz.ShowMarker = false;
fprintf('1.1左臂初始姿态\r\n');
disp(iviz.MarkerBodyPose);
R_L = iviz.MarkerBodyPose(1:3,1:3);
% eulZYX = rotm2eul(R_L, 'ZYX');
% fprintf('dcm2angle:[roll pitch yaw]\r\n    %f %f %f\r\n', eulZYX(3), eulZYX(2), eulZYX(1));
traj_L = traj_circle_3d(iviz.MarkerBodyPose(13),iviz.MarkerBodyPose(14),iviz.MarkerBodyPose(15));

iviz.MarkerBodyName = "r_wrist_3_link";
iviz.ShowMarker = false;
fprintf('1.2右臂初始姿态\r\n');
disp(iviz.MarkerBodyPose);
R_R = iviz.MarkerBodyPose(1:3,1:3);
% eulZYX = rotm2eul(R_R, 'ZYX');
% fprintf('dcm2angle:[roll pitch yaw]\r\n    %f %f %f\r\n', eulZYX(3), eulZYX(2), eulZYX(1));
traj_R = traj_circle_3d(iviz.MarkerBodyPose(13),iviz.MarkerBodyPose(14),iviz.MarkerBodyPose(15));

r = rateControl(5);
trajConfig_L = zeros(6,length(traj_L));
trajConfig_R = zeros(6,length(traj_R));
% trajConfig = zeros(12,length(traj_L));

fprintf('2.进行左臂轨迹逆解\r\n');
iviz.MarkerBodyName = "l_wrist_3_link";
iviz.ShowMarker = false;
for i = 1:length(traj_L)
    pose_L = [R_L [traj_L(1,i); traj_L(2,i); traj_L(3,i)]; [0 0 0 1]];
    [qSol_L,~] = ik('l_wrist_3_link', pose_L, weights, qInitial);
    qInitial = qSol_L;
    for l_arm=1:6
        trajConfig_L(l_arm,i) = qSol_L(l_arm);
    end
end

fprintf('3.进行右臂轨迹逆解\r\n');
iviz.MarkerBodyName = "r_wrist_3_link";
iviz.ShowMarker = false;
for i = 1:length(traj_R)
    pose_R = [R_R [traj_R(1,i); traj_R(2,i); traj_R(3,i)]; [0 0 0 1]];
    [qSol_R,~] = ik('r_wrist_3_link', pose_R, weights, qInitial);
    qInitial = qSol_R;
    for r_arm = 1:6
        trajConfig_R(r_arm,i) = qSol_R(r_arm+6);
    end
end

fprintf('4.合成双臂轨迹\r\n');
trajConfig = [trajConfig_L', trajConfig_R']';

fprintf('5.启动双臂轨迹仿真\r\n');
hold on;
for i = 1:length(traj_L)
    iviz.Configuration = trajConfig(:,i);
    plot3(traj_L(1,:), traj_L(2,:), traj_L(3,:),'r.');
    plot3(traj_R(1,:), traj_R(2,:), traj_R(3,:),'g.');

    waitfor(r);
end


