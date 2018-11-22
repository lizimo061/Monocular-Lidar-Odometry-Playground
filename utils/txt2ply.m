input_dir = '/home/zimol/Data/output/';

curr_dir = pwd;

cd(input_dir);

files = dir(strcat(input_dir, '*.txt'));
for i=1:length(files)
    tmp_file = files(i).name;
    tmp_data = importdata(tmp_file);
    xyzPoints = tmp_data(:,1:3);
    intensity = tmp_data(:,5);
    ptCloud = pointCloud(xyzPoints, 'Intensity', intensity);
    
    out_name = strcat(tmp_file(1:end-3), 'ply');
    pcwrite(ptCloud, out_name);
end

cd(curr_dir);