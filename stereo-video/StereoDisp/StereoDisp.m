% Obtain disparity map of the left image of a rectified stereo image pair
function disp_map = StereoDisp(im_left, im_right, K, max_disp, win_size, disp_scale)
% ALGORITHM DESCRIPTION:
%    My algorithm is a hybrid approach (mixture of block and region based).
% In a nutshell, I first convert the stereo image pair from R,G,B to L,a,b
% color space. Next, I perform intensity(L)-based segmentation of only left
% image pixels using a fast histogram-based K-Means implementation, then
% refine the segment boundaries using morphological filtering and connected
% components analysis. Then I determine the disparities of pixels that make
% up the refined boundaries using a block-based SAD approach, and lastly,
% fill in the (missing) disparities of pixels lying inside refined segment
% boundaries based on boundary disparities, using my reconstruction method.
%
% INPUT ARGUMENTS:
%  -   im_left  : Left image of the rectified stereo image pair
%  -   im_right : Right image of the rectified stereo image pair
%  -          K : Number of intensity-based clusters for K-Means clustering
%  -   max_disp : Maximum disparity search range
%  -   win_size : Length of square block for cost aggregation (must be odd)
%  - disp_scale : Factor by which calculated disparities will be multiplied
%                 (value should be such that, max_disp * disp_scale <= 255)
%
%       CAUTION : If you don't specify values for any of the last four
%                 input arguments, I will assume some random values,
%                 and the disparity map will be created accordingly!
%
% OUTPUT:
%  -   disp_map : Generated disparity map; To view it, use imshow(disp_map)
%                 Nearer objects are represented by lighter shades of grey.
%
%          NOTE : As of now, my algorithm does not determine disparities of
%                 few rows and columns of pixels lying near the borders of
%                 the input image (black regions in output disparity map).
%                 You are encouraged to modify the algorithm to extrapolate
%                 already calculated disparities to those regions as well.
%
%      AUTHOR   : Subhayan Mukherjee (subhayan001@gmail.com)
%      DATE     : Apr.2014
%
% Basic error checking
if ~exist('im_left', 'var') || ~exist('im_right', 'var'), disp('Input stereo image pair not properly specified!'); end
%
% Default (random) input arguments
if ~exist('K', 'var'), K = 10; end
if ~exist('max_disp', 'var'), max_disp = 15; end
if ~exist('win_size', 'var'), win_size = 9; end
if ~exist('disp_scale', 'var'), disp_scale = floor(255/max_disp); end

% Read original left and right images
he_l = imread(im_left);
he_r = imread(im_right);

% Extract intensities (L) from original left and right images
cform = makecform('srgb2lab');
lab_he = applycform(he_l,cform);
Int_l = double(lab_he(:,:,1:1));
lab_he = applycform(he_r,cform);
Int_r = double(lab_he(:,:,1:1));

% Initialize remaining parameters
win_lim = double(idivide(uint8(win_size),2));

% Classify the Intensities in 'L' Space Using K-Means Clustering
L = FastCMeans(uint8(Int_l), K);

% Label every pixel in the right image using the results from K-Means
leftImage = L * double(idivide(255, uint8(K)));

% Find the dimensions of the left image
[nrLeft,ncLeft] = size(leftImage);

% Initialize the disparity map for the left image
disp_map = zeros(nrLeft, ncLeft);

% Determine segment boundaries of the left image
segb_left = zeros(nrLeft-1, ncLeft-1);
for i = 1: 1: nrLeft-1
    for j = 2: 1: ncLeft-1
        if leftImage(i, j) ~= leftImage(i, j+1) ...
                || leftImage(i, j) ~= leftImage(i+1, j+1) ...
                || leftImage(i, j) ~= leftImage(i+1, j) ...
                || leftImage(i, j) ~= leftImage(i+1, j-1)
            segb_left(i, j) = 1;
        end
    end
end

% Morphologically cleanse the segment boundary map
segb_left = bwmorph(bwmorph(segb_left, 'fill', Inf), 'remove', Inf);

% Remove small artifacts (bounded regions formed out of localized intensity
% variations) from the segment boundary map by connected component analysis

CC = bwconncomp(segb_left);
numPixels = cellfun(@numel,CC.PixelIdxList);
[~, art_idx] = sort(numPixels);

pix_cnt = sum(sum(segb_left));
stop_idx = 0;
art_cnt = numPixels(art_idx(1));
while art_cnt / pix_cnt * 100 < 4
    stop_idx = stop_idx + 1;
    art_cnt = art_cnt + numPixels(art_idx(stop_idx));
end    
stop_idx = stop_idx - 1;

for i = 1: 1: stop_idx
    segb_left(CC.PixelIdxList{art_idx(i)}) = 0;
end

% Calculate disparities of left image pixels lying on segment boundaries
for i = win_lim+1: 1: nrLeft-1 - win_lim
    for j = win_lim+1 + max_disp: 1: ncLeft-1 - win_lim - 1
        if segb_left(i, j) > 0
            disp_map(i, j) = PixelDisp(Int_r, i, win_lim, j, Int_l, max_disp, win_size) * disp_scale;
        end
    end
end

% Replicate segment boundary disparities row-wise
for i = win_lim+1: 1: nrLeft-1 - win_lim
    last_idx = win_lim+1 + max_disp;
    
    % Determine disparity of first pixel in row
    j = last_idx;
    disp_map(i, j) = PixelDisp(Int_r, i, win_lim, j, Int_l, max_disp, win_size) * disp_scale;
    
    last_disp = disp_map(i, last_idx);
    last_idx = last_idx + 1;
    for j = win_lim+2 + max_disp: 1: ncLeft-1 - win_lim - 1
        % Determine disparity of last pixel in row
        if j == ncLeft-1 - win_lim - 1
            disp_map(i, j) = PixelDisp(Int_r, i, win_lim, j, Int_l, max_disp, win_size) * disp_scale;
            continue;
        end
        % Determine disparity of other pixels
        if segb_left(i, j) > 0
            rep_size = j-last_idx;
            if last_idx > win_lim+2 + max_disp  % Check if this's the first detected segment boundary pixel in this row
                if disp_map(i, j) == last_disp    % Disparities of this boundary pixel and the last one detected match!
                    disp_map(i, last_idx : (j-1)) = repmat(last_disp, 1, rep_size); % Make disparities of all pixels
                end                                                                 % lying in between, identical
            end
            last_disp = disp_map(i, j);
            last_idx = j + 1;
        end
    end
end

% Determine disparities of all remaining pixels of first and last row
for i = [win_lim+1 nrLeft-1 - win_lim]
    for j = win_lim+2 + max_disp: 1: ncLeft-1 - win_lim - 2
        if disp_map(i, j) == 0
            disp_map(i, j) = PixelDisp(Int_r, i, win_lim, j, Int_l, max_disp, win_size) * disp_scale;
        end
    end
end

% Disparity map reconstruction based on already determined disparities

% Determine boundaries separating regions in disparity map which have been
% already populated from regions where disparities are yet to be determined
bin_perim = bwperim(disp_map);

n_rows = size(bin_perim, 1);
n_cols = size(bin_perim, 2);

per_top = zeros(n_rows, n_cols);
per_bottom = zeros(n_rows, n_cols);

copy_idx = 0;

% For every pixel in the disparity map whose disparity has not yet been
% determined, find its nearest pixel whose disparity has been already
% determined, and which lies ABOVE the said pixel in the same column
for j = 1: 1: n_cols
    for i = 1: 1: n_rows
        if bin_perim(i, j) == 0
            per_top(i, j) = copy_idx;
        else
            copy_idx = i;
        end
    end
end

% For every pixel in the disparity map whose disparity has not yet been
% determined, find its nearest pixel whose disparity has been already
% determined, and which lies BELOW the said pixel in the same column
for j = 1: 1: n_cols
    for i = n_rows: -1: 1
        if bin_perim(i, j) == 0
            per_bottom(i, j) = copy_idx;
        else
            copy_idx = i;
        end
    end
end

disp_map = disp_map / disp_scale;

for i = win_lim+1: 1: nrLeft-1 - win_lim
    for j = win_lim+2 + max_disp: 1: ncLeft-1 - win_lim - 1
        if disp_map(i, j) == 0
            % Minimum of nearest (determined) disparities along same column
            disp_map(i, j) = min(disp_map(per_top(i, j), j), disp_map(per_bottom(i, j), j));
        end
    end
end

disp_map = uint8(disp_map * disp_scale);