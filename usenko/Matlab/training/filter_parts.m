

%-----------------------------
% Filter segments
%-----------------------------

%
% segments with less than N points
%
n_points = minimum_points;
del_ind_f = find(features(:,1) < n_points);

% rejected feature vectors
filtered1_features = features;
filtered1_features(del_ind_f,:) = [];


% eval0==0 means some linear segment
% if eval0 == 0 
%      feature_v =[];
%      feature_ids = [];
% return;



%
% filter lines
%
% l0min= filtered1_features(:, 25);
% l0max= filtered1_features(:, 26);
l1min= filtered1_features(:, 27);
l1max= filtered1_features(:, 28);            

%threshold
threshold = 0.06;  % noise level
del_ind_f = find( ( filtered1_features(:,27) + filtered1_features(:,28) ) < threshold );
filtered1_features(del_ind_f,:) = [];

