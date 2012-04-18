
% clear bbox

inc_x = bbox(1);
inc_y = bbox(2);

orig = [point(1)-inc_x, point(2)-inc_y];
quiver3(orig(1), orig(2), 0,  2*inc_x, 0, 0, 1, 'w'); 
quiver3(orig(1), orig(2), 0,  0, 2*inc_y, 0, 1, 'w'); 

orig = [point(1)+inc_x, point(2)+inc_y];
quiver3(orig(1), orig(2), 0,  -2*inc_x, 0, 0, 1, 'w'); 
quiver3(orig(1), orig(2), 0,  0, -2*inc_y, 0, 1, 'w');                 

