function result = angleNormalize(x)
  result = mod(x + pi, 2*pi) - pi;
end