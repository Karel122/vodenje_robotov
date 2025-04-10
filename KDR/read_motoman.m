function [x, ft, q] = read_motoman()
%[x, ft, q] = read_motoman() vrne
% x - lega robota v zunanjih koordinatah 6x1 z ZYX eulerjevimi koti
% ft - vektor sil in navorov 6x1, ki jih izmeri senzor v lastnem KS
% q - vrednost sklepnih spremenljivk

q = [1.047197551196598, -0.523598775598299, 0.523598775598299, 0, 1.570796326794897, 0];
ft = [0 0 20 0 3.4 0]';
x = [0.119, 0.206114046100696, 0.352967875173176, -2.094395102393196, 0, 3.141592653589793];