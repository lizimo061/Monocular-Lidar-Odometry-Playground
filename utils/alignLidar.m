lidar_poses = importdata('/home/zimol/Data/lidar_pose.txt');
pc_path = '/home/zimol/Data/small_set/scans/';

pos_fix = '*.ply';
pc_files = dir(strcat(pc_path,pos_fix));
pc_set = cell(length(pc_files),1);
N = length(pc_files);

for i = 1:N
    tmp = pcread(strcat(pc_path, pc_files(i).name));
    pc_set{i} = tmp.Location;
end

total_pc = [];

for i = 1:N
    i
    tmp = lidar_poses(i,:);
    R = [tmp(1) tmp(2) tmp(3);
         tmp(4) tmp(5) tmp(6);
         tmp(7) tmp(8) tmp(9)];
    t = [tmp(10);tmp(11);tmp(12)];
    T= [R,t;zeros(1,3),1];
    tmp_pc = pc_set{i};
    new_pc = T*[tmp_pc';ones(1,length(tmp_pc))];
    total_pc = [total_pc new_pc];
    
end

scatter3(total_pc(1,:),total_pc(2,:),total_pc(3,:),'b.');
axis equal;

% for i = 1:N
%     i
%     tmp = opt_poses(i,:);
%     R = quat2rotm([tmp(1),tmp(2),tmp(3),tmp(4)]);
%     t = [tmp(5);tmp(6);tmp(7)];
%     T= [R,t;zeros(1,3),1];
%     tmp_pc = pc_set{i};
%     new_pc = T*[tmp_pc';ones(1,length(tmp_pc))];
%     total_pc = [total_pc new_pc];
%     
% end
% 
% scatter3(total_pc(1,:),total_pc(2,:),total_pc(3,:),'b.');
% axis equal;