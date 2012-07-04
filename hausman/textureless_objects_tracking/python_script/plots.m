function plots(features)
	[dist_matrix]=generate_variables(features);
	columns(dist_matrix)
	plot_dists(dist_matrix)
