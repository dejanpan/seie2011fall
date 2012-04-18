

%-------------------------
% Select local maxima
%-------------------------

% discretize the space
one_side_width = 10;  %  meters
one_side_deep = 5;
width = 2 * one_side_width;
deep = 2 * one_side_deep;


cell_size = cell_size_2d;

n_rows = floor(width / cell_size);
n_columns = floor(width / cell_size);

for o=1:n_training_objects            
    space_2d(o).votes = zeros(n_rows, n_columns);    
    for i=1:n_rows
        for j=1:n_columns
             space_2d(o).parts(i,j).list = [];
        end
    end    
end    


% votes
for o=1:n_training_objects            
    list_votes = scene_object(o).votes;
    [n_r n_c] = size(list_votes);
    for i=1:n_r
        vote = list_votes(i,:);
        x = vote(1) + one_side_width;
        y = vote(2) + one_side_deep;
        vote_weight = vote(4);
        x_coor = floor( x / cell_size) + 1;
        y_coor = floor( y / cell_size) + 1;
        r = n_columns - y_coor;
        c = x_coor;
        space_2d(o).votes(r, c) = space_2d(o).votes(r, c) + vote_weight;        % for visualization
        
        % store part identifier
        p_id = vote(end);
        space_2d(o).parts(r, c).list = [space_2d(o).parts(r,c).list;  vote_weight p_id];
        
     end
end    


% % debug 
% for o=1:n_training_objects   
%    figure();
%    imagesc( space_2d(o).votes );   
% end    


%---------------------------------------
% look for local maxima
%---------------------------------------
for o=1:n_training_objects   
    space_2d(o).local_maxima= zeros(n_rows, n_columns);   
end    


for o=1:n_training_objects   
    votes = space_2d(o).votes;
    w_size = search_window(o);
    local_maxima= ordfilt2(votes, w_size*w_size, ones(w_size, w_size) );        
    binary_local_maxima = (votes >= local_maxima & votes ~= 0);        
    
    % debug
%     figure();  
%     imagesc(local_maxima);
%     colormap('default');
%     title(  strcat('object ',int2str(o) ) );
%     
%     figure();
%     imagesc(binary_local_maxima);   
%     colormap('gray');
    
    
    %---------------------------------------
    % look for the best hypotheses for each object
    %---------------------------------------    
    [inx] = find(binary_local_maxima== 0);
    votes(inx) = 0;
    
    space_2d(o).local_maxima = votes;

%     % debug
%     figure();
%     imagesc( space_2d(o).local_maxima );   
%     title(  strcat('object ',int2str(o) ) );
end    



%---------------------------------------
% look for the best hypotheses for each object
%---------------------------------------
for o=1:n_training_objects       
    votes = space_2d(o).local_maxima;
    idx = find(votes);
    [n_best_hypotheses, kk] = size(idx);

    for n=1:n_best_hypotheses        
        [m_v m_ix] = max(votes(:));

        % coordinates 
        [row_coor, col_coor]  = ind2sub( size(votes), m_ix);
        
        y = n_columns - row_coor;
        x = col_coor;
        
        w_x = (x-1)*cell_size + cell_size/2;
        w_y = (y-1)*cell_size + cell_size/2 ;
        w_y = w_y -  one_side_deep ;
        w_x = w_x -  one_side_width ;
                
        hypo_id = n;
        best_hypotheses(o).list = [best_hypotheses(o).list; row_coor col_coor w_x w_y m_v hypo_id];
        votes(row_coor, col_coor) = 0;
    end
end    







