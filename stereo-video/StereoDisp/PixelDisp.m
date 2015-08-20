% Function to calculate disparity of a given pixel using SAD cost function
function disp = PixelDisp(Int_r, i, win_lim, j, Int_l, max_disp, win_size)
    B = Int_l((i-win_lim) : (i+win_lim), (j-win_lim) : (j+win_lim));
    candidates = Int_r((i-win_lim) : (i+win_lim), (j-max_disp-win_lim) : (j-1+win_lim));
    min_S = win_size * win_size * 255;
    disp = 1;
    for k = 1: 1: max_disp
        C = candidates(1 : win_size, k : (k+win_size-1));
        S = sum(sum(abs(B-C)));
        if S < min_S
            min_S = S;
            disp = max_disp + 1 - k;
        end
    end
end