function Inova = calcInova(vsensor, vreal, pose)

  zsensor = w2r(vsensor, pose);
  zreal = w2r(vreal, pose);
  
  Inova = reshape(zsensor - zreal, [2*size(zsensor, 2) 1]);
end