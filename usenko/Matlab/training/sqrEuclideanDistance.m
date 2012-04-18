

%
% Euclidean distance
%

function sedist = EuclideanDistance(A,B)

    sedist =  sum( (A - B) .^ 2 ) ;

end