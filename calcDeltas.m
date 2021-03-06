function [ds, dth] = calcDeltas(Vels, dt, diam)
  dsr = dt * Vels.right;
  dsl = dt * Vels.left;
  ds = (dsr + dsl) / 2;
  dth = angleNormalize(dsr - dsl) / (2*diam);
end