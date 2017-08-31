function M = get_camera()
  % pinhole
  f = 1;
  M = [1 0 0 0; 0 1 0 0; 0 0 1/f 0];
endfunction