clear all;

%create group
http_init
numLaser = '1';
gr.name = 'grupo1';
gr.resources{1} = '/motion/pose';
gr.resources{2} = '/motion/vel2';
gr.resources{3} = ['/perception/laser/' numLaser '/distances?range=-90:90:1'];
http_delete(['http://127.0.0.1:4950/group/' gr.name]);
g1 = http_post('http://127.0.0.1:4950/group', gr);

%constants
dt = 2;
sigx = 5;
sigy = 5;
sigth = 3*pi/180;
R = diag([sigx^2 sigy^2 sigth^2]);
sigld = 10;
siglth = 0.2*pi/180;
Q1 = diag([sigld^2 siglth^2]);
diam = 165;
%variables
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
%err = 10;
%PoseR = [Pose.x; Pose.y; Pose.th]
PoseR = [2340 1600 0];
update{1}.pose.x = PoseR(1);
update{1}.pose.y = PoseR(2);
update{1}.pose.th = PoseR(3)*180/pi;
http_put(g1, update);
Vels = data{2}.vel2;

figure(1)
clf; hold on;
drawMap;
plot(Pose.x, Pose.y, 'o');

figure(2)
clf; hold on;
figure(3)
clf; hold on;
figure(4)
clf; hold on;
iter = 0;

while 1
  iter = iter + 1
  % Prediction step
  [ds dth] = calcDeltas(Vels, dt, diam)
  thm = Pose.th + dth/2;
  G = calcG(ds, thm);
  V = calcV(ds, thm, diam);
  Sigd = calcSigd(Vels, dt);
  Sigb = G * Sig * G' + V * Sigd * V' + R;
  %End of prediction step
  
  sleep(dt);
  
  %Update step 
  %read sensor
  data = http_get(g1);
  Pose = data{1}.pose;
  Pose.th = mod(Pose.th*pi/180,2*pi);%degrees to rad
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
    PrevPose = PoseR;
    PoseR = PoseR + K * Inova;
    
    plot(PoseR(1), PoseR(2), 'o');
    figure(2);
    plot([iter-1 iter], [PrevPose(1) PoseR(1)], '-');
    figure(3);
    plot([iter-1 iter], [PrevPose(2) PoseR(2)], '-');
    figure(4);
    plot([iter-1 iter], angleNormalize([PrevPose(3) PoseR(3)]), '-');

    update{1}.pose.x = PoseR(1);
    update{1}.pose.y = PoseR(2);
    update{1}.pose.th = PoseR(3)*180/pi;
    http_post(g1, update);
    
    Sig = (eye(3) - K*H)*Sigb;
  end
  
  fflush(stdout);
end

http_delete(g1);
