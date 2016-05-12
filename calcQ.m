function Q = calcQ(Q1, n)
  Q = Q1;
  for k = 1:(n-1)
    Q = blkdiag(Q, Q1);
  end
end