function plot_dists(dist_matrix)
	hold off
	for i=1:columns(dist_matrix)
		if (i<6)	
			figure(1)
			plot (dist_matrix(:,i), int2str(i), "linewidth", 3)
			hold on
			xlabel("time","fontsize", 14);
			ylabel("relative distances","fontsize", 14);
			axis([0,600, 0.1, 0.22]);
			legend('green line - red cylinder(diff. object)')
			#legend('green cylinder - red cylinder(diff. object)', 'red cylinder - blue circle (same objects)', 'green cylinder - blue circle(diff. object)')
		elseif (i==6)
			plot (dist_matrix(:,i), "y", "linewidth", 3)
			#xlabel("time","fontsize", 14);
			#ylabel("relative distances","fontsize", 14);
			#axis([0,400, 0.0, 0.4]);
			#legend('green line - red line(diff. object)', 'red line - blue corner (same objects)', 'red line - yellow corner(diff. object)', 'green line - blue corner (diff. objects)','green line - yellow corner (same objects)','yellow corner - blue corner (diff.object)')
			#axis([0,400, 0.0, 0.5]);
		elseif (i<11)&&(i>6)
			figure(2)
			plot (dist_matrix(:,i), int2str(i-6+1), "linewidth", 3)
			hold on
		elseif (i<16)&&(i>=11)
			figure(3)
			plot (dist_matrix(:,i), int2str(i-11+1), "linewidth", 3)
			hold on
		elseif (i<21)&&(i>=16)
			figure(4)
			plot (dist_matrix(:,i), int2str(i-16+1), "linewidth", 3)
			hold on
		elseif (i<26)&&(i>=21)
			figure(5)
			plot (dist_matrix(:,i), int2str(i-21+1), "linewidth", 3)
			hold on
		endif
	endfor

