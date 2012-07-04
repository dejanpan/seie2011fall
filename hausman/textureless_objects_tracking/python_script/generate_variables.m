function [dist_matrix]=generate_variables(features)
	#global dist_matrix
	col=1;
	for i=0:features
		for j=(i+1):(features-1)
			dist_matrix(:,col)=load(strcat('dist',int2str(i),'_',int2str(j),'.txt'));		
			col=col+1;
			#s=strcat('"',"global",' dist',int2str(i),'_',int2str(j),'=load','(',"'",'dist',int2str(i),'_',int2str(j),'.txt',"'",')','"')
			#eval(s)
		endfor
	endfor
