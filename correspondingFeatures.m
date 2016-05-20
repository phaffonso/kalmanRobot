function [valid corr] = correspondingFeatures(features, result, pose)

  detectedFeatures = [];
  valid = [];
  range = 2:2:(size(result, 1) - 1);
  for k = range
    matchArr = featureMatch(result(k, :), features, pose);
    [dummy, i] = max(matchArr);
    matches = sum(matchArr);
    if matches == 1
      detectedFeatures = [detectedFeatures ; i];
      valid = [valid; result(k, :)];
    else
      detectedFeatures = [detectedFeatures ;(-matches)];
    end
  end
  
  if(sum(detectedFeatures > 0) > 0)
    try
      corr = features(detectedFeatures(detectedFeatures > 0), :);
    catch
      detectedFeatures
      error('line20');
    end
  else
    corr = []
  end

end