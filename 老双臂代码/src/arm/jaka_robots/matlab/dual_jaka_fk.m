close;
% clear;
% clc;

dual_zu5_model;
% [0 45 -90 180 180 0 180 135 90 0 180 0]
% ==================== 运动控制 ====================
% 左臂
iviz.Configuration(1) = deg2rad(0); 
iviz.Configuration(2) = deg2rad(-45); 
iviz.Configuration(3) = deg2rad(-90); 
iviz.Configuration(4) = deg2rad(180);        
iviz.Configuration(5) = deg2rad(180);
iviz.Configuration(6) = deg2rad(0);
% 右臂
iviz.Configuration(7) = deg2rad(180);
iviz.Configuration(8) = deg2rad(135);
iviz.Configuration(9) = deg2rad(90);
iviz.Configuration(10) = deg2rad(0);
iviz.Configuration(11) = deg2rad(180);
iviz.Configuration(12) = deg2rad(0);

% **************************************************
% 设置左臂
iviz.MarkerBodyName = "l_wrist_3_link";
% 设置右臂
% iviz.MarkerBodyName = "r_wrist_3_link";

iviz.ShowMarker = false;

% 仿真显示
fprintf('Transform:joint[1]->joint[6]\r\n');
disp(iviz.MarkerBodyPose);
fprintf('dcm2quat:[W X Y Z]\r\n');
R = iviz.MarkerBodyPose(1:3,1:3);
disp(rotm2quat(R));
eulZYX = rotm2eul(R, 'ZYX');
fprintf('dcm2angle:[roll pitch yaw]\r\n    %f %f %f\r\n', eulZYX(3), eulZYX(2), eulZYX(1));
fprintf('\r\n    %f %f %f\r\n', rad2deg(eulZYX(3)), rad2deg(eulZYX(2)), rad2deg(eulZYX(1)));

end_pos=[iviz.MarkerBodyPose(13),iviz.MarkerBodyPose(14),iviz.MarkerBodyPose(15),eulZYX(3), eulZYX(2), eulZYX(1)];
end_qpos = [0,iviz.MarkerBodyPose(13),iviz.MarkerBodyPose(14),iviz.MarkerBodyPose(15),rotm2quat(R)];
clear yaw pitch roll robot eulZYX

% ==================== 碰撞估计 ====================
jaka.DataFormat = 'column';
[isColliding, selfSeparationDist, selfWitnessPts] = checkCollision(jaka,iviz.Configuration,'SkippedSelfCollisions','parent');
if isColliding
    [body1Idx,body2Idx] = find(isnan(selfSeparationDist));
    collidingPairs = unique(sort([body1Idx,body2Idx],2));
    highlightColor = [1 0.8 0];
    for i = 1:size(collidingPairs,1)
        p = findall(gca, 'type', 'patch', 'displayname', [jaka.Bodies{collidingPairs(i)}.Name, '_mesh']);
        if isempty(p)
            continue;
        else
            p(1).FaceColor = highlightColor;
        end
    end
end
