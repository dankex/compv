% Solve the transform on the object knowing projection of each point
% X is the original 3D coordinates
% Xb is observed 3D coordinates
% P is the camera matrix
function T = solve_matched(X, Xb, P)
  % solve the [R,T] matrices
  TOL = .001;
  [R, T] = ppnp(Xb, X, TOL);
  S = [[R T]; 0 0 0 1];

  % correct for P
  T = inv([P; 0 0 0 1]) * S;
endfunction