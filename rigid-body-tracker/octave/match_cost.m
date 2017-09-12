% ------- COST FUNC ------- %
function L = match_cost(x, y, z)
  % TODO transform Y by t=[x,y,z]
  % Project Y to camera
  global gX
  global gY
  global gR
  global gP

  X = gX;
  Y = gY;

  sizes = size(x);
  len = prod(sizes);

  xv = reshape(x, len, 1);
  yv = reshape(y, len, 1);
  zv = reshape(z, len, 1);

  L = zeros(len, 1);

  for i = 1:len
    if mod(i, 1000) == 1
      fprintf("cell %d/%d\n", i, len);
      fflush(stdout);
    endif
    T = gR;
    T(1,4) = x(i);
    T(2,4) = y(i);
    T(3,4) = z(i);

    Yt = (T * [Y ones(size(Y,1),1)]')';
    Xbt = (gP * Yt')';
    Xbt = Xbt ./ Xbt(:,3);

    prob = 1;
    logProb = 0;
    for q = size(Xbt, 1)
      r = Xbt(q, :);
      pr = pdf_point_cloud_2d(X, r);
      prob = prob * pr;
      logProb = logProb + log(pr);
    endfor
    
    global verify
    if verify
      error = norm(X - Xbt)
      prob
    endif
    
    loss = -logProb;
    L(i) = loss;
  endfor

  L = reshape(L, sizes);
endfunction