


*****************************
Folder training
*****************************
- Contains the code for training data provided by Zoltan.



"run_training.m"
----------------
This file contains the code to train the model and save the data.

Usually the data provided by Zoltan is divided into different objects. Each object has several views. Each view is divided into different parts using the segmentation algorithm by Zoltan (see RAM paper).

The code starts reading the point data corresponding to each object,view,part:

	read_data_training;

Then the features:

	read_features;

Then some preprocessing (filtering the data and features) and creating the vocabulary with k-means.

 

"run_test.m"
----------------
Main code for classifying scenes. It starts reading points and data (similar to training).
The main procedures of this code is:
		generate_votes;

Which generates the votes in 3D space. Until here is the most important. 

From here on the code just try to get the parts generating the votes and do some filtering with bounding boxes of objects but I think you should decide this post-processing depending on your data.


In general the code is messy, but the procedure is very simple. I suggest to read the paper about voting from Bastian Leibe from the references of the RAM paper. I just follow this idea.



******************
Data
******************
I left some data from Zoltan in 

http://fortune.is.kyushu-u.ac.jp/~omozos/2x16_views.zip 




