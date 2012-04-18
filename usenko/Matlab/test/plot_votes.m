%------------------------------------
% Plot points + votes
%------------------------------------


class_colors= 'rbgcmyk';

% chair = 1 = r
% table = 2 = b


hold on;
    
[n_f l_f] = size(filtered_features);    

% look for parts pertaining to current view
c_parts = filtered_features;


[n_f l_f] = size(c_parts);    
for i = 1:n_f
    % part identifiers
    p = c_parts(i, l_f);

    % look for points pertaining to this part
    [n_p l_p] = size(points);
    part_points_indexes = find(  points(:, l_p)==p );
    part_points = points( part_points_indexes, : );  
    
    % plot points
    plot_part_test( part_points, 'k' ) ;      
%     plot_part_test( part_points, class_colors( mod(i,7)+1 ) ) ;      
    hold on;

    
%     % find center
%     [n_p l_p] = size(points);
%     ind = find( part_centers(:, l_p) == p ) ;
%     p_center = part_centers(ind, 1:3);   
%     
%     % plot center
%     scatter3( p_center(1), p_center(2), p_center(3), 30, 'g','filled' );
    hold on;

    
    %
    % plote vote arrows
    %
%     c_votes = codebook_votes(i).votes;
%     c_objects = codebook_votes(i).objects;
% 
%     if isempty(c_votes)== 0 % not empty        
%         % by object
%         for o=1:n_training_objects
%             o_indexes = find(c_objects==o) ;
%             o_votes = c_votes(o_indexes,:);
% 
% 
%             % project votes on Z
%             %o_votes(:,3) = 0;
%             plot_codebook_votes( o_votes, class_colors(o) );
%             hold on;
%         end
%     end                     

end    %for i = 1:n_f % parts of one view



% %
% % Plot votes
% %
% for o=1:n_training_objects            
%     v = scene_object(o).votes;
%     %v(:,3) = 0;
%      %scatter3(v(:,1), v(:,2), v(:,3), 10, class_colors(o), 'filled');        
%      scatter3(v(:,1), v(:,2), v(:,3), 10, class_colors(o));        
%  end    


        

%pause;
%close;





