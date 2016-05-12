%converts x,y world coordinates to r, theta robot centered coordinates
function rth = w2r(xy, pose)
  p = pose(1:2, 1);
  f0 = xy' - p;
  f1 = [sqrt(sum(f0.*f0)); atan2(f0(2, :), f0(1, :)) - pose(3)];
  rth = f1;
end