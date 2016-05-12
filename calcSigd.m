function Sigd = calcSigd(Vels, dt)
  ks = 0.05;
  Sigd = diag(ks*Vels.right*dt, ks*Vels.left*dt);
end