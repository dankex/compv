1. Unzip all files to the same folder.

2. To run the algorithm on the provided rectified stereo image pair, execute the following MATLAB command and see the output:
	imshow(StereoDisp('im_left.png', 'im_right.png', 10, 15, 9, 16))

3. You can also open the provided ground truth disparity map for the left image 'truedisp_left.png' and compare it with my output.

4. Also included is a (down-scaled) stereo frame extracted from a 3D video recorded in our lab using our Sony HDR-TD10 3D camcorder
('cam_left.jpeg' and 'cam_right.jpeg') and the output (left image) depth map ('cam_depth_left.png') after running our algorithm:
	imshow(StereoDisp('cam_left.jpeg', 'cam_right.jpeg', 15, 25, 7, 10))


The image files and parameter values mentioned in points 2 and 3 have all been taken from the Middlebury stereo vision dataset [1]:
http://vision.middlebury.edu/stereo/data/

The FastCMeans.m and LUT2label.m files have been taken from Anton Semechko's "Fast segmentation of N-dimensional grayscale images":
http://www.mathworks.in/matlabcentral/fileexchange/41967-fast-segmentation-of-n-dimensional-grayscale-images/content/FastCMeans.m

References:
[1]	D. Scharstein and R. Szeliski. A taxonomy and evaluation of dense two-frame stereo correspondence algorithms.
International Journal of Computer Vision, 47(1/2/3):7-42, April-June 2002.