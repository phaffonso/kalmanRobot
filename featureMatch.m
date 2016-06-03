%Matches a single detected feature with one or more real features
function matchArr = featureMatch(detected, features, pose)
  %constants
  dth = 5*pi/180;
  dr = 60;
  
  d1 = w2r(detected, pose);
  f1 = w2r(features, pose);
  difr = f1(1, :) .- d1(1, :);
  difth = angleNormalize(f1(2, :) .- d1(2, :));
  %pseudo rectangle matching
  matchArr = (abs(difr) < dr) .* (abs(difth) < dth);
  %pseudo ellipsis matching
  %matchArr = dif(1, :).^2 ./ dr + dif(2, :).^2 ./ dth < 1;
  
end