

function n = getNParts(filename)

    fid = fopen(filename, 'r');
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
             n = sscanf(tline, '%*s %d', inf);
             break
         end   
    end

    fclose(fid);
    
end