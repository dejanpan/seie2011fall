

function ret_dist = gaussianDistance(x, center, max_dist)
    
    d =  sqrEuclideanDistance(x, center);   

    if ( d <= max_dist )
        ret_dist = 1.0;
    else
        a = (d-max_dist) .^ 2;
        b = (max_dist) .^ 2;  % maxt_dist = sigma
        ret_dist = exp( -a ./ b );
    end
        
end