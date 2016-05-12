function V = calcV(dst, th, diam)
  k = dst / (4*diam);
  a = cos(th)/2 - k * sin(th);
  b = sin(th)/2 + k * cos(th);
  c = cos(th)/2 + k * sin(th);
  d = sin(th)/2 - k * cos(th);
  V = [ a   c
        b   d
        1/(2*diam)  -1/(2*diam)];
end