Directory: StereoDisp
Website: http://www.mathworks.com/matlabcentral/fileexchange/46172-fast-stereo-matching-and-disparity-estimation-by-s-mukherjee-and-prof-g-r-m-reddy
Description: In a nutshell, I first convert the stereo image pair from R,G,B to L,a,b color space. Next, I perform intensity(L)-based segmentation of only left image pixels using a fast histogram-based K-Means implementation, then refine the segment boundaries using morphological filtering and connected components analysis. Then I determine the disparities of pixels constituting the refined boundaries using a block-based SAD approach, and lastly, fill in the (missing) disparities of pixels lying inside the refined segment boundaries (based on the refined boundaries' disparities already determined) using my simple and fast reconstruction method.

Directory: disparity-cv
Website: http://www.jayrambhia.com/blog/disparity-maps/
Description: Calculate disparity map with StereoBM and StereoSGBM. Some difference from ground truth but usable.

