

function [feature_v feature_ids] = loadFeatures(filename, view, part, object)
    
      
    fid = fopen(filename, 'r');
    while feof(fid) == 0 
         tline = fgetl(fid);  % description
         if ( strncmp(tline, 'nr_features:', 12)  )
             break
         else             
            tline = fgetl(fid);  % value         
         end
    end
        
    
    feature_v =[];
    feature_ids = [];
    
       
    %% features
	tline = fgetl(fid);  % value         
    n_features = sscanf(tline, ' %d', 1);        
    
       
    for i=1:n_features
        desc = fgetl(fid);  % description         
        val = fgetl(fid);  % value             

        
        %-------------------------------
        % center                
        %-------------------------------
        if ( strncmp(desc, 'x:', 2)  ) 
            v =  sscanf(val, ' %f', 1);
            center_x = v;
        end        
        if ( strncmp(desc, 'y:', 2)  ) 
            v =  sscanf(val, ' %f', 1);
            center_y = v;
        end        
        if ( strncmp(desc, 'z:', 2)  ) 
            v =  sscanf(val, ' %f', 1);
            center_z = v;
        end        
        
        %-------------------------------
        % used features                
        %-------------------------------
        
        % #1 
        % number of points
        if ( strncmp(desc, 'k:', 2)  ) 
            v =  sscanf(val, ' %d', 1);                                                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 1];
            
            % only segments with more than 20 points
%             if v < 20 
%                 feature_v = [];
%                 feature_ids = [];
%                 return;
%             end
            
        end        
        
        
        
        % #2 
        % Proportion of boundary points
        % Boundary points lie on the bundary of the object
        if ( strncmp(desc, 'bp:', 3)  ) 
            v =  sscanf(val, ' %f', 1);                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 2];
        end        
        
        % #3 
        % Min curvature: curvature of the point with minimum local curvature
        if ( strncmp(desc, 'minc:', 5)  ) 
            v =  sscanf(val, ' %f', 1);                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 3];
        end        
        
        % #4 
        % Max curvature: curvature of the point with maximum local curvature
        if ( strncmp(desc, 'maxc:', 5)  ) 
            v =  sscanf(val, ' %f', 1);                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 4];
        end        

        % #5 
        % Average curvature of the segment 
        if ( strncmp(desc, 'c:', 2)  ) 
            v =  sscanf(val, ' %f', 1);                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 5];
        end        
        
        
        %
        % eigen values
        %
        
        % #6 
        % smallest one
        if ( strncmp(desc, 'eval0:', 6)  ) 
            v =  sscanf(val, ' %f', 1);                        
            eval0=v;
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 6];
            
        end        
        
        
        % #7 
        if ( strncmp(desc, 'evec0x:', 7)  ) 
            v =  sscanf(val, ' %f', 1);     
            evec0x=v;
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 7];
        end   
        
        
        % #8 
        if ( strncmp(desc, 'evec0y:', 7)  ) 
            v =  sscanf(val, ' %f', 1);                        
            evec0y=v;
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 8];
        end   
        
                
        % #9 
        if ( strncmp(desc, 'evec0z:', 7)  ) 
            v =  sscanf(val, ' %f', 1);                        
            evec0z=v;
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 9];
        end        
        
        
        
        % #10 
        % middle
        if ( strncmp(desc, 'eval1:', 6)  ) 
            v =  sscanf(val, ' %f', 1);                        
            eval1 = v;
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 10];
        end        
        
        % #11 
        if ( strncmp(desc, 'evec1x:', 7)  ) 
            v =  sscanf(val, ' %f', 1);                
            evec1x=v;
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 11];
        end   
        
        % #12 
        if ( strncmp(desc, 'evec1y:', 7)  ) 
            v =  sscanf(val, ' %f', 1);                        
            evec1y=v;
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 12];
        end   
        
                
        % #13 
        if ( strncmp(desc, 'evec1z:', 7)  ) 
            v =  sscanf(val, ' %f', 1);                        
            evec1z=v;
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 13];
        end        
        
        
        %# 14
        % biggest
        if ( strncmp(desc, 'eval2:', 6)  ) 
            v =  sscanf(val, ' %f', 1);                        
            eval2 = v;
            feature_v = [feature_v, v];            
            feature_ids =[ feature_ids, 14];
           
        end   
            
        % #15 
        if ( strncmp(desc, 'evec2x:', 7)  ) 
            v =  sscanf(val, ' %f', 1);          
            evec2x=v;
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 15];
        end   
        
        
        % #16 
        if ( strncmp(desc, 'evec2y:', 7)  ) 
            v =  sscanf(val, ' %f', 1);                        
            evec2y=v;
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 16];
        end   
        
                
        % #17 
        if ( strncmp(desc, 'evec2z:', 7)  ) 
            v =  sscanf(val, ' %f', 1);                        
            evec2z=v;
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 17];
        end        
        
            
        %
        % proyections on the eigenvectors
        %
        
        % #18 
        if ( strncmp(desc, 'var0:', 5)  ) 
            v =  sscanf(val, ' %f', 1);                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 18];
            var0=v;
        end        
        
        % #19 
        if ( strncmp(desc, 'var1:', 5)  ) 
            v =  sscanf(val, ' %f', 1);                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 19];
            var1=v;
        end        
        
        % #20 
        if ( strncmp(desc, 'var2:', 5)  ) 
            v =  sscanf(val, ' %f', 1);                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 20];
            var2 = v;
        end        
        
        %
        % Invariants
        %
        % #21 
        if ( strncmp(desc, 'j1:', 3)  ) 
            v =  sscanf(val, ' %f', 1);                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 21];
        end        
        
        % #22 
        if ( strncmp(desc, 'j2:', 3)  ) 
            v =  sscanf(val, ' %f', 1);                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 22];
        end        
        
        % #23 
        if ( strncmp(desc, 'j3:', 3)  ) 
            v =  sscanf(val, ' %f', 1);                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 23];
        end        
        
        % volume ocupied by points (voxelized)
        % #24 
        if ( strncmp(desc, 'volume:', 7)  ) 
            v =  sscanf(val, ' %f', 1);                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 24];
            volume = v;
        end        
        
        % l0min l0max l1min l1max l2min l2max = minimum and maximum
        % distances from centroid along the corresponding eigenvector 
        % (the min+max sum is the length of the oriented bounding box's corresponding edge)
        
        % lmin/lmax
        
        % #25 
        if ( strncmp(desc, 'l0min:', 6)  ) 
            v =  sscanf(val, ' %f', 1);                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 25];
            l0min=v;
        end        
                
        % #26 
        if ( strncmp(desc, 'l0max:', 6)  ) 
            v =  sscanf(val, ' %f', 1);                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 26];
            l0max = v;
        end        
        
        % #27 
        if ( strncmp(desc, 'l1min:', 6)  ) 
            v =  sscanf(val, ' %f', 1);                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 27];
            l1min=v;
        end        
          
        
        % #28 
        if ( strncmp(desc, 'l1max:', 6)  ) 
            v =  sscanf(val, ' %f', 1);                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 28];
            l1max=v;
        end        
        
        
        % #29 
        if ( strncmp(desc, 'l2min:', 6)  ) 
            v =  sscanf(val, ' %f', 1);                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 29];
            l2min=v;
        end        
           
        
        % #30 
        if ( strncmp(desc, 'l2max:', 6)  ) 
            v =  sscanf(val, ' %f', 1);                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 30];
            l2max=v;                        
        end        
        
        %skew0abs skew1abs skew2abs = absolute value of the skew of distribution
        %along the corresponding eigenvector (0 means mean is centered)        
        
        % #31 
        if ( strncmp(desc, 'skew0abs:', 9)  ) 
            v =  sscanf(val, ' %f', 1);            
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 31];
        end        

        % #32 
        if ( strncmp(desc, 'skew1abs:', 9)  ) 
            v =  sscanf(val, ' %f', 1);                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 32];
        end        
        
        % #33 
        if ( strncmp(desc, 'skew2abs:', 9)  ) 
            v =  sscanf(val, ' %f', 1);                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 33];
        end        
        
        
        % kurtosis0 kurtosis1 kurtosis2 = kurtosis of distribution along the
        % corresponding eigenvector
        
        % #34 
        if ( strncmp(desc, 'kurtosis0:', 10)  ) 
            v =  sscanf(val, ' %f', 1);               
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 34];
        end        
        
        % #35 
        if ( strncmp(desc, 'kurtosis1:', 10)  ) 
            v =  sscanf(val, ' %f', 1);                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 35];
        end        
        
        % #36 
        if ( strncmp(desc, 'kurtosis2:', 10)  ) 
            v =  sscanf(val, ' %f', 1);                        
            feature_v = [feature_v, v];
            feature_ids =[ feature_ids, 36];
        end        

   end %for i=1:n_features

    % extra features
    
    % add proportions
    % #37 
    v = eval0 / (eval0 + eval1  + eval2);
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 37];

    % #38 
    v = eval1 / (eval0 + eval1  + eval2);
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 38];

    % #39 
    v = eval2 / (eval0 + eval1  + eval2);
    feature_v = [feature_v, v];    
    feature_ids =[ feature_ids, 39];

    % #40 
    v = eval0 / eval1;
    feature_v = [feature_v, v];    
    feature_ids =[ feature_ids, 40];

    % #41 
    v = eval0 / eval2;
    feature_v = [feature_v, v];    
    feature_ids =[ feature_ids, 41];

    % #42 
    v = eval1 / eval2;
    feature_v = [feature_v, v];      
    feature_ids =[ feature_ids, 42];

           
    % #43   
    % sqrt(var0) / (l0min + l0max)
    sum0 =   (l0min +l0max);
    if sum0 == 0
        %error('Error reading features: sum0 == 0');
        v = 1;
    else
        v = sqrt(var0) /sum0;
    end    
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 43];


    % #44   
    % sqrt(var1) / (l1min + l1max)
    sum1 =   (l1min +l1max);
    if sum1 == 0
        error('Error reading features: sum1 == 0');            
        %v = 1;
    else
        v = sqrt(var1) / sum1;
    end
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 44];


    % #45   
    % sqrt(var2) / (l2min + l2max)
    sum2 =   (l2min +l2max);
    if sum2 == 0
        error('Error reading features: sum2 == 0');            
    end
    v = sqrt(var2) / sum2;
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 45];


    % #46   
    % l0min / l0max
    if l0max == 0
        v = 1;
    else
        v =  l0min / l0max;
    end
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 46];

    % #47   
    % l1min / l1max
    v =  l1min / l1max;
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 47];

    % #48   
    % l2min / l2max
    v =  l2min / l2max;
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 48];

    % #49   
    % volume / ( (l0min+l0max)*(l1min+l1max)*(l2min+l2max) )
    v =  volume / ( (l0min+l0max)*(l1min+l1max)*(l2min+l2max) );
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 49];              
    
    
    % 50 acos( abs(evec0x) )
    v = acos( abs(evec0x) );
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 50];                      

    % 51 acos( abs(evec0y) )
    v = acos( abs(evec0y) );
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 51];                      
    
    % 52 acos( abs(evec0z) )
    v = acos( abs(evec0z) );
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 52];                      
    
    
    % 53 acos( abs(evec1x) )
    v = acos( abs(evec1x) );
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 53];                      

    % 54 acos( abs(evec1y) )
    v = acos( abs(evec1y) );
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 54];                      
    
    % 55 acos( abs(evec1z) )
    v = acos( abs(evec1z) );
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 55];                      
        
    % 56 acos( abs(evec2x) )
    v = acos( abs(evec2x) );
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 56];                      

    % 57 acos( abs(evec2y) )
    v = acos( abs(evec2y) );
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 57];                      
    
    % 58 acos( abs(evec2z) )
    v = acos( abs(evec2z) );
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 58];                      
       
    % 59 (lmin1+lmax1) / (lmin2+lmax2)
    v = (l1min + l1max) / (l2min + l2max);
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 59];                      
    
    % 60 (l0min+l0max)
    v = (l0min + l0max);
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 60];                      

    % 61 (l1min+l1max)
    v = (l1min + l1max);
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 61];                      
    
    % 62 (l2min+l2max)
    v = (l2min + l2max);
    feature_v = [feature_v, v];
    feature_ids =[ feature_ids, 62];                      
    
    % center
    feature_v = [feature_v, center_x];
    feature_v = [feature_v, center_y];
    feature_v = [feature_v, center_z];
    
    % part,view,object identifer
    feature_v = [feature_v, view];                
    feature_v = [feature_v, part];       
    feature_v = [feature_v, object];       
     
    fclose(fid);
    
    
end


















