warning off all

% Vzrocni cas simulacije v Simulinku.
dT = 1/500;

% Zacetna lega robota za vse gibe.
q0 = [0 0 0 0 90 0]'*pi/180;

% Ojacenje za izracun inverzne kinematike.
fbGain = 20;

system('MotomanVodenje.exe &')

% Odpiranje predloge
open 'rtMotomanVR.slx';

