%------------------------------------
% Plot points + votes
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
    c_codebook = codebook_votes(indexes);    
    
    
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
        plot_part( part_points, 'k' ) ;      
        hold on;
        
        % plot centers
        p_center = [ c_parts(i, end-5:end-3) ];
        %scatter3( p_center(1), p_center(2), p_center(3), 30, 'g','filled' );
        %hold on;
        
        % plote votes
        c_votes = c_codebook(i).votes;
        c_objects = c_codebook(i).objects;
        
        % by object
        for o=1:n_training_objects
            o_indexes = find(c_objects==o) ;
            o_votes = c_votes(o_indexes,:);
            plot_codebook_votes( o_votes, class_colors(o) );
            hold on;
            
            %meanshift_votes( o_votes, class_colors(o) );
        end
        
    end    
    pause;
    clf;
end    
close;



