
%
% calculate Davies-Bouldin (DB) index
%
% In:
%
% "Pattern recognition" 3rd Edition
%
% Section 16.4.1  "Hard Clustering"
%

function db = calculate_db(examples, IDX, n_classes)

    % get number of examples
    [N l]=size(examples);
 
    % representatives == means in this case
    w = zeros(n_classes, l); 
    
    % dispersions
    s = zeros(n_classes, 1); 
    
   % representatives
   for i=1:n_classes
        % extract examples of current class i
        examples_i=examples(IDX==i,:);
        w(i,:) = mean(examples_i);
   end;    
    
   % calculate dispersion of the clusters
   for i=1:n_classes      
        % extract examples of current class i
        examples_i=examples(IDX==i,:);
        [n_i l_i] = size(examples_i);
        for j=1:n_i
            s(i,1) = s(i,1) + norm( examples_i(j,:) - w(i,:) );
            s(i,1) = (1/n_i) * s(i,1);
        end;    
   end;    
   
   
   % disimilarity matrix
   d=zeros(n_classes,n_classes);
   for i=1:n_classes 
        for j=1:n_classes 
            d(i,j) = norm( w(i,:) - w(j,:) );
        end;    
   end; 
   
   
   % R matrix
   R=zeros(n_classes,n_classes);
   for i=1:n_classes 
        for j=1:n_classes 
            if i==j 
                R(i,j) = -inf;
            else   
                R(i,j) = ( s(i,1) + s(j,1) ) / d(i,j);
            end;
        end;    
   end; 
    
   
   % R_max
   R_max = zeros(n_classes,1);
   for i=1:n_classes 
        R_max(i,1) = max( R(i,:) );
   end; 
   
   %db
   db = sum(R_max);
   db = db / n_classes;
   
   
   
    
    
    
    
    
    
    
    
    
    
    