%Matches a single detected feature with one or more known features
function matchArr = featureMatch(detected, features, pose)
  %constants
  dth = 2*pi/180; dr = 30;
  
  d1 = w2r(detected, pose);
  f1 = w2r(features, pose);
  dif = f1 .- d1;
  %pseudo rectangle
  matchArr = (abs(dif(1, :)) < dr) .* (abs(dif(2, :)) < dth);
  %pseudo ellipsis
  %matchArr = dif(1, :).^2 ./ dr + dif(2, :).^2 ./ dth < 1;
  
end