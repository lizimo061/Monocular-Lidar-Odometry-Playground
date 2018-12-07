function drawFrame(origin, rotm, axis_len)
    hold on;
    scatter3(origin(1),origin(2),origin(3),'bo');
    x_vec = rotm(:,1);
    y_vec = rotm(:,2);
    z_vec = rotm(:,3);
    x_end = axis_len * x_vec';
    y_end = axis_len * y_vec';
    z_end = axis_len * z_vec';

    quiver3(origin(1),origin(2),origin(3),x_end(1),x_end(2),x_end(3),'r','LineWidth',3);
    quiver3(origin(1),origin(2),origin(3),y_end(1),y_end(2),y_end(3),'g','LineWidth',3);
    quiver3(origin(1),origin(2),origin(3),z_end(1),z_end(2),z_end(3),'b','LineWidth',3);
    
end

