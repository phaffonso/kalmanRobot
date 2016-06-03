clear all;

%create group
http_init;
realrob = 0;
kalman_on = 1;
initial_pose_error = 0;
if(realrob)
  disp('using real robot');
  numLaser = '0';
  baseurl = 'http://10.1.3.130:4950/';
else
  disp('using simulator');
  numLaser = '1';
  baseurl = 'http://127.0.0.1:4950/';
end
gr.name = 'grupo2';
gr.resources{1} = '/motion/pose';
gr.resources{2} = '/motion/vel2';
gr.resources{3} = ['/perception/laser/' numLaser '/distances?range=-90:90:1'];
http_delete([baseurl 'group/' gr.name]);
g1 = http_post([baseurl 'group'], gr);
g1 = [baseurl 'group/' gr.name];
%constants
dt = 2;
sigx = 5;
sigy = 5;
sigth = 3*pi/180;
R = diag([sigx^2 sigy^2 sigth^2]);
sigld = 5;
siglth = 0.2*pi/180;
Q1 = diag([sigld^2 siglth^2]);
diam = 165;

%variables
deltaPose = zeros(3, 1);
Sig = zeros(3);

features = [0 0
0 2280
920 2280
920 3200
4190 3200
4190 2365
4680 2365
4680 650
4030 0 ];

%read sensor
data = http_get(g1);
Pose = data{1}.pose;
Pose.th = mod(Pose.th*pi/180,2*pi);
%PoseR = [Pose.x; Pose.y; Pose.th]
PoseR = [2340 1600 0];
if initial_pose_error
  PoseR = PoseR + [20 20 2*pi/180];
end
update{1}.pose.x = PoseR(1);
update{1}.pose.y = PoseR(2);
update{1}.pose.th = PoseR(3)*180/pi;
http_put(g1, update);
Vels = data{2}.vel2;

figure(1)
clf; hold on;
drawMap;
plot(PoseR(1), PoseR(2), 'o');

figure(2)
clf; hold on;
figure(3)
clf; hold on;
figure(4)
clf; hold on;
iter = 0;

tic;
key = [];
while (sum(size(key)) == 0)
  key = getKey;
  iter = iter + 1
  PoseR
  % Prediction step
  [ds dth] = calcDeltas(Vels, dt, diam);
  thm = Pose.th + dth/2;
  G = calcG(ds, thm);
  V = calcV(ds, thm, diam);
  Sigd = calcSigd(Vels, dt);
  Sigb = G * Sig * G' + V * Sigd * V' + R;
  %End of prediction step
  
  elapsedTime = toc
  sleep(dt - elapsedTime);
  tic;
  
  %Update step 
  %read sensor
  data = http_get(g1);
  Pose = data{1}.pose;
  Pose.th = mod(Pose.th*pi/180,2*pi);%degrees to rad
  PrevPose = PoseR;
  PoseR = [Pose.x; Pose.y; Pose.th];
  Vels = data{2}.vel2;
  
  %Feature detection
  rho = data{3}.distances;
  figure(1);
  result = (SplitAndMerge(rho, [-90 90 1], Pose))';
  [sensor real] = correspondingFeatures(features, result, PoseR);
  numFeatures = size(sensor, 1)
  
  %Calc inova
  if(numFeatures > 1)
    Inova = calcInova(sensor, real, PoseR);
    
    H = calcH(PoseR, real);
    Q = calcQ(Q1, numFeatures);
    K = Sigb * H' * inv(H * Sigb * H' + Q);
    
    if(kalman_on)
      deltaPose = K * Inova
      PoseR = PoseR + K * Inova;
    end

    update{1}.pose.x = deltaPose(1);
    update{1}.pose.y = deltaPose(2);
    update{1}.pose.th = deltaPose(3)*180/pi;
    PoseR
    angledif = abs(PrevPose(3) - PoseR(3))
    if(angledif  > 0.1)
      disp rotationDetected
    else
      http_post([baseurl 'motion/pose'], update{1}.pose);
      disp poseUpdated
    end
    
    Sig = (eye(3) - K*H)*Sigb;
  end
  
  plot(PoseR(1), PoseR(2), 'o');
  figure(2);
  plot([iter-1 iter], [PrevPose(1) PoseR(1)], '-');
  figure(3);
  plot([iter-1 iter], [PrevPose(2) PoseR(2)], '-');
  figure(4);
  plot([iter-1 iter], angleNormalize([PrevPose(3) PoseR(3)]), '-');
  
  fflush(stdout);
end

http_delete(g1);
