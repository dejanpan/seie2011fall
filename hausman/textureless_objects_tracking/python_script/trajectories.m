function trajectories(features)
	#clear;
	hold off;
	hold on;
	#traj_matrix=load('../poses_final.txt');
	traj_matrix=load('../results/scene1/poses_final.txt')
	rgb_mat=[1,0,0;0,1,0;0,0,1;1,1,0;1,0,1;0,1,1;0.5,0,0;0,0.5,0;0,0,0.5;0.5,0.5,0;0.5,0,0.5;0,0.5,0.5];
	col_mat=[0,0,0];
	for f=1:features
		x=[0 0 0];
		for i=f:features:rows(traj_matrix)
			i;
			x=[x ; traj_matrix(i,:)];
		endfor
		#a=linspace (1, 1, rows(x));
		#b=linspace (1, 1, rows(x));
		a=1:rows(x);
		b=1:rows(x);		
		#mesh(x(:,1),x(:,2),x(:,3))
		#[xx,yy]=meshgrid(a,b);
		f
		figure(1)
		scatter3(x(2:end,1),x(2:end,2),x(2:end,3),[5], rgb_mat(f,:), "s")
		for i=1:(rows(x)-1)
			col_mat(i,:)=rgb_mat(f,:);
		endfor
		#figure(2)
		#plot3(x(2:end,1),x(2:end,2),x(2:end,3),"Marker",".","color",rgb_mat(f,:))
	
		x=[0 0 0];
	endfor
	hold off;
