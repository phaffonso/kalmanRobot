
load laserData.mat

features = [0 0
0 2280
920 2280
920 3200
4190 3200
4190 2365
4680 2365
4680 650
4030 0 ]';

angles = -90:1:90;
theta = (angles + pose.th) * (pi/180);

%figure(2)
%polar(theta, laserData);

rho = laserData;
points = [ rho.*cos(theta) + pose.x; rho.*sin(theta) + pose.y];

drawMap;
figure(1)
plot(points(1, :), points(2, :), 'o');

pose2 = pose;
pose2.th = pose.th * pi / 180;

result = (SplitAndMerge(rho, [-90 90 1], pose2))' ;

[sensor real] = correspondingFeatures(features', result, [pose.x;pose.y;pose2.th])

%detectedFeatures = [];
%for k = 2:2:(size(result, 2) - 1)
%  matchArr = featureMatch(result(:, k), features, [pose.x;pose.y;pose2.th]);
%  [dummy, i] = max(matchArr);
%  matches = sum(matchArr);
%  if matches == 1
%    detectedFeatures = [detectedFeatures i];
%  else
%    detectedFeatures = [detectedFeatures (-matches)];
%  end
%end

%detectedFeatures

%if((sum(detectedFeatures) <= 0) == 0)
%  vsensor = result(:, 2:2:(size(result, 2) - 1));
%  vreal = features(:, detectedFeatures);
%  Inova = calcInova(vsensor, vreal, [pose.x;pose.y;pose2.th]);
%  Inova
%end
