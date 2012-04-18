



function plot_codebook_votes(votes, c)
    scatter3(votes(:,1), votes(:,2), votes(:,3), 20, c, 'filled');

    [n_vo kk] = size(votes); 
    
    for vo = 1: n_vo
        %plot vectors
        vec = votes(vo, 8:10);
        p_center = votes(vo, 5:7);
        quiver3(p_center(1), p_center(2), p_center(3),  vec(1), vec(2), vec(3), 1, 'r');          
    end
end