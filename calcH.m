function H = calcH(pose, landmarks)
  H = [];
  x = pose(1); y = pose(2);
  q = x^2 + y^2;
  for k = 1:size(landmarks, 1)
    lx = landmarks(k, 1);
    ly = landmarks(k, 2);
    H = [H
      (x-lx)/sqrt(q)  (y-ly)/sqrt(q)  0
      (ly-y)/q        (x-lx)/q        -1];
  end
  
end