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
sigld = 0.5;
siglth = 0.1*pi/180;
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
%PoseR = [Pose.x + err; Pose.y + err; Pose.th];
PoseR = [2350 1610 0];
%PoseR = [2000 1500 2];
update{1}.pose.x = PoseR(1);
update{1}.pose.y = PoseR(2);
update{1}.pose.th = PoseR(3);
http_put(g1, update);
Vels = data{2}.vel2;

drawMap;
figure(1)
plot(Pose.y, Pose.x, 'o');

while 1
  % Prediction step
  [ds dth] = calcDeltas(Vels, dt, diam);
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
  Pose.th = mod(Pose.th*pi/180,2*pi);
  PoseR = [Pose.x; Pose.y; Pose.th];
  Vels = data{2}.vel2;
  
  %Feature detection
  rho = data{3}.distances;
  result = (SplitAndMerge(rho, [-90 90 1], Pose))';
  [sensor real] = correspondingFeatures(features, result, PoseR);
  numFeatures = size(sensor, 1)
  
  %Calc inova
  if(numFeatures > 1)
    Inova = calcInova(sensor, real, PoseR);
    
    H = calcH(PoseR, real);
    Q = calcQ(Q1, numFeatures);
    K = Sigb * H' * inv(H * Sigb * H' + Q);
    PoseR = PoseR + K * Inova
    plot(PoseR(2), PoseR(1), 'o');
    

    update{1}.pose.x = PoseR(1);
    update{1}.pose.y = PoseR(2);
    update{1}.pose.th = PoseR(3);
    http_put(g1, update);
    
    Sig = (eye(3) - K*H)*Sigb;
  end
  
  fflush(stdout);
end

http_delete(g1);
