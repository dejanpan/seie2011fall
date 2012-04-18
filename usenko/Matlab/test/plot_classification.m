%------------------------------------
% Plot points according to the classification
%------------------------------------
class_colors= 'rbgcmykw';

% chair = 1 = r
% table = 2 = b

figure();
hold on;
max_views = max(n_views);

for v = 0:n_views-1
    v

    [n_f l_f] = size(filtered_features);    
    % look for parts pertaining to this view
    indexes = find( filtered_features(:, l_f-2) ==  v );
    c_parts = filtered_features( indexes, : );
    c_classification = classification( indexes, :);
    
    [n_f l_f] = size(c_parts);    
    for i = 1:n_f
        % view and part identifiers
        v = c_parts(i, l_f-2);
        p = c_parts(i, l_f-1);
        o = c_parts(i, l_f);

        % look for points pertaining to this part
        [n_p l_p] = size(points);
        part_points_indexes = find( points(:, l_p-2)==v & points(:, l_p-1)==p & points(:, l_p)==o);
        part_points = points( part_points_indexes, : );    

        % plot points
        c = class_colors(classification(i));
        plot_part( part_points, c ) ;       
        hold on;
    end    
    pause;
    clf;
end    
close;



