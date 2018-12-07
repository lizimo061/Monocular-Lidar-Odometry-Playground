pre_poses = importdata('/home/zimol/Data/poses.txt');
opt_poses = importdata('/home/zimol/Data/test.txt');
%example_poses = importdata('/home/zimol/Documents/SFM_example/example_poses.txt');
example_poses = importdata('/home/zimol/Data/lidar_pose.txt');
poses_sfm = importdata('/home/zimol/Data/poses_sfm.txt');



figure(3);
for i = 1:size(example_poses,1)
    tmp = example_poses(i,:);
    R = [tmp(1) tmp(2) tmp(3);
         tmp(4) tmp(5) tmp(6);
         tmp(7) tmp(8) tmp(9)];
    t = [tmp(10);tmp(11);tmp(12)];
    drawFrame(t, R, 0.2);
    hold on;
    text(t(1),t(2),t(3), int2str(i));
    hold on;
end
xlabel('x');
ylabel('y');
zlabel('z');
axis equal;
grid on;

% For pre optimization

%figure(1);

% pre_poses_T = zeros(3,4,size(pre_poses,1));
% 
% for i = 1:size(pre_poses,1)
%     tmp = pre_poses(i,:);
%     R = [tmp(1) tmp(2) tmp(3);
%          tmp(4) tmp(5) tmp(6);
%          tmp(7) tmp(8) tmp(9)];
%     t = [tmp(10);tmp(11);tmp(12)];
%     %pre_poses_T(:,:,i) = [R,t];
%     drawCamera([R,t], 300,300,700,0.001,1);
%     %plotCamera('Location', (-R'*t)', 'Orientation', R', 'Size', 0.1);
%     hold on;
%     t_w = (-R'*t)';
%     text(t_w(1),t_w(2),t_w(3), int2str(i));
%     hold on;
% end
% axis equal;
% grid on;

% 
% 
% 
% % For post optimization
% 
figure(2);

for i = 1:size(opt_poses,1)
    tmp = opt_poses(i,:);
    R = quat2rotm([tmp(1),tmp(2),tmp(3),tmp(4)]);
    t = [tmp(5);tmp(6);tmp(7)];
    
    drawFrame(t, R, 0.4);
    hold on;
    text(t(1),t(2),t(3), int2str(i));
    hold on;
end
xlabel('x');
ylabel('y');
zlabel('z');
axis equal;
grid on;

% figure(5);
% 
% for i = 1:size(poses_sfm,1)
%     tmp = poses_sfm(i,:);
%     R = quat2rotm([tmp(1),tmp(2),tmp(3),tmp(4)]);
%     t = [tmp(5);tmp(6);tmp(7)];
%     
%     drawCamera([R,t], 300, 300, 700, 0.001,1)
%     hold on;
%     t_w = (-R'*t)';
%     text(t_w(1),t_w(2),t_w(3), int2str(i));
%     hold on;
% end
% xlabel('x');
% ylabel('y');
% zlabel('z');
% axis equal;
% grid on;