curr_dir = pwd;
scans_dir = '/home/zimol/Data/scans/';
images_dir = '/home/zimol/Data/images/';

target_images = '/home/zimol/Data/new_images/';
target_scans = '/home/zimol/Data/new_scans/';

starting_id = 16;
ending_id = 722;
interval = 6;

for i = starting_id:interval:ending_id
    img_f = strcat(sprintf('%06d', i),'.png');
    scan_f = strcat(sprintf('%06d', i),'.ply');
    img_src = strcat(images_dir, img_f);
    scan_src = strcat(scans_dir, scan_f);
    copyfile(img_src, target_images);
    copyfile(scan_src, target_scans);
    
end