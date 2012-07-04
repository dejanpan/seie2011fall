import os, sys, math

def process(path,out):
    stext = ' '
    rtext = ','   
    input = open(path)
    output = open(out, 'w')
    count = 0
    line = 3
    for s in input.xreadlines():
        if ((count-line) % 4) == 0:
            output.write(s.replace(stext, rtext))
        count = count + 1
    input.close()
    output.close()

def vec_dist (x1, y1, z1, x2, y2, z2):
    dist = math.sqrt ((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) + (z2-z1)*(z2-z1))
    return dist

def compute_dists(path,features):
    for number in range(features):
	for number2 in xrange(number+1,features):
	    target = number
	    source = number2
	    count = 0
    	    input = open(path)
	    output = open('dist'+str(number)+'_'+str(number2)+'.txt', 'w')
	    for s in input.xreadlines():
	 #       print float(s.split(' ')[0]), s.split(' '), s.split(' ')[2]
		if (count % features) == target:
		    t_x = float(s.split(' ')[0])
		    t_y = float(s.split(' ')[1])
		    t_z = float(s.split(' ')[2])
		if (count % features) == source:
		    s_x = float(s.split(' ')[0])
		    s_y = float(s.split(' ')[1])
		    s_z = float(s.split(' ')[2])
		if count != 0 and (count % features) == 0:
		    dist = vec_dist (s_x, s_y, s_z, t_x, t_y, t_z)
		    #print "count: ",  count, "dist: ", dist
		    output.write(str(dist) + '\n')
		count = count + 1
		print "count: ",count
    	    input.close()
	    output.close()




if __name__ == "__main__":
    path = sys.argv[1]
    features =int(sys.argv[2])
    compute_dists(path,features)
