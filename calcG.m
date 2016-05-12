function G = calcG(dst, th)
  G =  [1   0   (-dst) * sin(th)
        0   1   dst * cos(th)
        0   0   1];
end