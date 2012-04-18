

%     % #1  number of points
%     % #2  Proportion of boundary points
%     % #3  Min curvature: curvature of the point with minimum local curvature
%     % #4  Max curvature: curvature of the point with maximum local curvature
%     % #5  Average curvature of the segment 
%     % #6 eval0
%     % #7 evec0x
%     % #8 evec0y 
%     % #9 evec0z
%     % #10 eval1
%     % #11 evec1x
%     % #12 evec1y
%     % #13 evec1z
%     %# 14 eval2
%     % #15 evec2
%     % #16 evec2y
%     % #17 evec2z 
%
%     %
%     % proyections on the eigenvectors
%     %
%     % #18 var0
%     % #19 var1
%     % #20 var2

%     %
%     % Invariants
%     %
%     % #21 j1
%     % #22 j2
%     % #23 j3

%     % volume ocupied by points (voxelized)
%     % #24 volume

%     % l0min l0max l1min l1max l2min l2max = minimum and maximum
%     % distances from centroid along the corresponding eigenvector 
%     % (the min+max sum is the length of the oriented bounding box's corresponding edge)
% 
%     % #25 l0min
%     % #26 l0max
%     % #27 l1min
%     % #28 l1max 
%     % #29 l2min
%     % #30 l2max
% 
%     %skew0abs skew1abs skew2abs = absolute value of the skew of distribution
%     %along the corresponding eigenvector (0 means mean is centered)        
% 
%     % #31 skew0abs
%     % #32 skew1abs
%     % #33 skew2abs
% 
%     % kurtosis0 kurtosis1 kurtosis2 = kurtosis of distribution along the
%     % corresponding eigenvector 
%     % #34 kurtosis0
%     % #35 kurtosis1
%     % #36 kurtosis2

% % extra features
% 
% % #37 eval0 / (eval0 + eval1  + eval2);
% % #38 eval1 / (eval0 + eval1  + eval2);
% % #39 eval2 / (eval0 + eval1  + eval2);
% % #40 eval0 / eval1;
% % #41 eval0 / eval2;
% % #42 eval1 / eval2;
% % #43   sqrt(var0) / (l0min + l0max)
% % #44   sqrt(var1) / (l1min + l1max)
% % #45   sqrt(var2) / (l2min + l2max)
% % #46   l0min / l0max
% % #47   l1min / l1max
% % #48   l2min / l2max
% % #49   volume / ( (l0min+l0max)*(l1min+l1max)*(l2min+l2max) )
% % 50 acos( abs(evec0x) )
% % 51 acos( abs(evec0y) )
% % 52 acos( abs(evec0z) )
% % 53 acos( abs(evec1x) )
% % 54 acos( abs(evec1y) )
% % 55 acos( abs(evec1z) )
% % 56 acos( abs(evec2x) )
% % 57 acos( abs(evec2y) )
% % 58 acos( abs(evec2z) )
% % 59 (lmin1+lmax1) / (lmin2+lmax2)
% % 60 (l0min+l0max)
% % 61 (l1min+l1max)
% % 62 (l2min+l2max)

% almost of them 
% selected_indexes = [2 5 18 19 20 21 22 23 24 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 52 59 60 61 62];

selected_indexes = [2 5 18 19 20 24 37 38 39 40 41 42 43 44 45 46 47 48 49 52 59 60 61 62];

% selected indexes + centroid + identifiers
selected_features = filtered_features(:, [selected_indexes 63 64 65 66 67 68]);
selected_feature_ids = feature_ids(:,  selected_indexes);

