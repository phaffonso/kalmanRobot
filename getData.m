
interval = 1;

http_init;
%http_init('SID_8628');

%numSamples = 100;
numLaser = '1';
%numLaser = '0';
address = 'http://127.0.0.1:4950/';
%address = 'http://143.106.148.171:9090/resource/RobotFEEC2/';
urlLaser = [address 'perception/laser/' numLaser '/distances?range=-180:180:1'];
urlPose = [address 'motion/pose'];

laserData = http_get(urlLaser);
pose = http_get(urlPose);

save laserData.mat laserData pose