curr_dir = pwd;
scans_dir = '/home/zimol/Data/hamerschleg/scans/';
images_dir = '/home/zimol/Data/hamerschleg/images/';

target_images = '/home/zimol/Data/hamerschleg/s_images/';
target_scans = '/home/zimol/Data/hamerschleg/s_scans/';

starting_id = 0;
ending_id = 811;
interval = 6;

for i = starting_id:interval:ending_id
    img_f = strcat(sprintf('%06d', i),'.png');
    scan_f = strcat(sprintf('%06d', i),'.ply');
    img_src = strcat(images_dir, img_f);
    scan_src = strcat(scans_dir, scan_f);
    copyfile(img_src, target_images);
    copyfile(scan_src, target_scans);
    
end