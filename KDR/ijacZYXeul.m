function invJ = ijacZYXeul(q)

	% q - Vrednost kotov v sklepih robota (vhod).
	% invJ - Inverzna analiticna Jacobijeva matrika, kjer je orientacija
	%        predstavljena z ZYX Eulerjevimi koti (izhod).

	% Izracun geometrijske Jacobijeve matrike (Jg).
		Jg = jacobi0(q);                                              %%% STUDENT %%%

	% Izracun lege vrha robota z ZYX Eulerjevimi koti za opis orientacije.
		X = q2ZYXeul(q);                                              %%% STUDENT %%%
		
	% Iz vektorja X izluscimo ustrezne komponente Eulerjevih kotov.
		fi      = X(4);                                               %%% STUDENT %%%
		theta   = X(5);                                               %%% STUDENT %%%
		psi     = X(6);                                               %%% STUDENT %%%

	% Izracun transformacijske matrike za pretvorbo iz geometrijske v
	% analiticno Jacobijevo matriko.
		Tr = [eye(3,3) zeros(3,3);
              zeros(3,3) [0 -sin(fi) cos(fi)*cos(theta);
                          0  cos(fi) sin(fi)*cos(theta);
                          1  0         -sin(theta)]];                                        %%% STUDENT %%%
        
	% Izracun inverzne analiticne Jacobijeve matrike.
    % inverzni Jacobi [6x5]-->psevdo inverz: dq=[J'*J]^-1*J'
% 		invJ = [(inv(Tr)*Jg).'*(inv(Tr)*Jg)]^-1*(inv(Tr)*Jg).';
        invJ = inv(inv(Tr)*Jg);%%% STUDENT %%%