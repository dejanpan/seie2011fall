

%
% Euclidean distance
%

function sedist = EuclideanDistance(A,B)

    sedist = sqrt( sum( (A - B) .^ 2 ) );

end