% Solve the transform on the object knowing projection of each point
% X is the original 3D coordinates
% Xb is observed 3D coordinates
% R is the known rotation matrix
% P is the camera matrix
function T = solve_unmatched(X, Xb, R, P)
  % Estimate the initial values for t (translation)
  % Estimate center (U0,V0) and diameter in image D0 = max(H0, W0)

  Umax = max(Xb(:,1));
  Umin = min(Xb(:,1));

  Vmax = max(Xb(:,2));
  Vmin = min(Xb(:,2));

  U0 = mean([Umin, Umax]);
  V0 = mean([Vmin, Vmax]);

  % Initial X0 Y0
  XX0 = inv([P; 0 0 0 1]) * [U0, V0, 0, 1]';
  X0 = XX0(1);
  Y0 = XX0(2);

  H0c = Umax - Umin;
  W0c = Vmax - Vmin;

  % Diameter in camera coordinates
  D0c = max(H0c, W0c);

  % World coords
  Xmax = max(X(:,1));
  Xmin = min(X(:,1));

  Ymax = max(X(:,2));
  Ymin = min(X(:,2));

  H0w = Ymax - Ymin;
  W0w = Xmax - Xmin;

  D0w = max(H0w, W0w);

  % Depth (TODO consider P)
  Z0 = D0w / D0c;

  fprintf("Initial values %f %f %f\n", X0, Y0, Z0);

  use_old_version = 0;
  if use_old_version
    % Optimization
    gBest = pso3func(Xb, X, R, P, [X0, Y0, Z0]);
  else      
    global gX = Xb;
    global gY = X;

    global gR = R;
    global gP = P;
    
    global verify
    if verify
      % verify the solution
      solution_cost = match_cost(X0, Y0, Z0)
      return
    endif

    gBest = pso3_v2(50, 100, [X0, Y0, Z0]);

    gBest
    gBest = gBest(size(gBest)(1), :);
  endif

  gBest

  % correct for P
  T = R;
  T(1,4) = gBest(1);
  T(2,4) = gBest(2);
  T(3,4) = gBest(3);
endfunction

