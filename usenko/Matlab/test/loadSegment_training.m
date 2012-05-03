


function segment = loadSegment(filename, view, part, object)
    
    %------------------------------------------
    % load training_set
    %------------------------------------------
    fprintf('Loading segments...');
    
    
    fid = fopen(filename, 'r');
    
    if fid == -1
        error( strcat('Error opening: ', filename) );
        return;
    end
    
    
    while feof(fid) == 0 
         tline = fgetl(fid);
         % comments
         if ( strncmp(tline, '#', 1)  )
             continue
         end  
         
         if ( strncmp(tline, 'DATA', 4)  )
             break
         end    
         
         if ( strncmp(tline, 'COLUMNS', 7)  )
             continue
         end    
         
         if ( strncmp(tline, 'POINTS', 6)  )
             n_points = sscanf(tline, '%*s %d', inf);
             continue
         end   
    end

    
    %% data points
    segment = zeros(n_points, 6);
    point= zeros(3,1);
    for i=1:n_points
        tline = fgetl(fid);
        point = sscanf(tline, ' %f', 3);        
        segment(i, 1:3) = point';
        segment(i,4) = view;
        segment(i,5) = part;
        segment(i,6) = object;
    end
    
    fclose(fid);
    
    fprintf('DONE\n');
end

















