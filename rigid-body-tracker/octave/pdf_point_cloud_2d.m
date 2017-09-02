% X: observed points
% x: point to test
function sumP = pdf_point_cloud_2d(X, x)
  sigma = 1;
  Z = 1 / sqrt(2 * pi * sigma ^ 2);
  S = 2 * sigma ^ 2;
  
  sumP = 0;
  for i = 1:size(X,1)
    mu = X(i, :);
    sumP = sumP + exp(-norm(x - mu) / S) / Z;
  endfor
  
  sumP = sumP / size(X,1);
endfunction