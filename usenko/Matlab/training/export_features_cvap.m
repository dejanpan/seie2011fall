

%-----------------------------
% Save data for CVPA
%-----------------------------


dlmwrite('features.dat', final_features, 'delimiter', '\t', 'precision', '%.6f');
