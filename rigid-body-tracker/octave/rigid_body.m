function X = rigid_body(id)
  switch (id)
    case 1
      % generate a circle
      N = 10;
      R = 2;
      t = (0:(1*pi/N):1*pi)';
      X = R * [cos(t) sin(t) 0 * t];
    case 2
      N = 60;
      R = 2;
      t = (0:(2*pi/N):2*pi)';
      X1 = R * [cos(t) sin(t) 0*t];
      X2 = R * [cos(t) 0*t sin(t)];
      X = [X1; X2];
  endswitch
endfunction