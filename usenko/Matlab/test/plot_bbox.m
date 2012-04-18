
% plot bbox

function plot_bbox(bbox, color)

    point = bbox(1:2);
    inc_x = bbox(3);
    inc_y = bbox(4);
    
    orig = [point(1)-inc_x, point(2)-inc_y];
    quiver3(orig(1), orig(2), 0,  2*inc_x, 0, 0, 1, color); 
    quiver3(orig(1), orig(2), 0,  0, 2*inc_y, 0, 1, color); 

    orig = [point(1)+inc_x, point(2)+inc_y];
    quiver3(orig(1), orig(2), 0,  -2*inc_x, 0, 0, 1, color); 
    quiver3(orig(1), orig(2), 0,  0, -2*inc_y, 0, 1, color);                 
end


