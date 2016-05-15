%figure 1;
%hold on;
%Configuracao do grafico
d = 50;
axis([0-d 4680+d 0-d 3200+d], "equal");

%Desenha os obstaculos no grafico
lines = [0	2280	920	2280
920	2280	920	3200
4190	2365	4680	2365
4190	2365	4190	3200
4030	0	4680	650
0 2280 0 0
0 0 4030 0
920 3200 4190 3200
4680 650 4680 2365];
for i = 1:size(lines, 1)
  plot([lines(i, 1) lines(i, 3)], [lines(i, 2) lines(i, 4)], '-r');
end