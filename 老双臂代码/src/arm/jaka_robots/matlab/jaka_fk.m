close;
% clear;
% clc;

jaka_zu3_model;
% jaka_zu5_model;
% jaka_minicobo_model

%0 90 0 90 180 0
% ==================== 运动控制 ====================
iviz.Configuration(1) = deg2rad(0); 
iviz.Configuration(2) = deg2rad(0); 
iviz.Configuration(3) = deg2rad(0); 
iviz.Configuration(4) = deg2rad(0);        
iviz.Configuration(5) = deg2rad(0);
iviz.Configuration(6) = deg2rad(0);
% **************************************************

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
clear yaw pitch roll robot eulZYX iviz