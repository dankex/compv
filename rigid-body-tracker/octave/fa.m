function fa(X)
  global gX = X
  fprintf("fa");
  fb();
endfunction

function fb()
  global gX
  fprintf("fb");
  gX
endfunction
