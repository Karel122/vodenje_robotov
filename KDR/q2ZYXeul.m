 function X = q2ZYXeul(q)

	% q - Vektor kotnih spremenljivk robota (vhod).
	% x - Lega vrha robota XYZ in orientacija z ZYX Eulerjevimi koti (izhod).

	% Matrika lege vrha robota.
		T6= dirkinT6(dirkinA(q));                                              %%% STUDENT %%%

	% Izluscimo ustrezne komponente rotacijske matrike.
		r21 = T6(2,1);                                             %%% STUDENT %%%
		r31 = T6(3,1);                                             %%% STUDENT %%%
		r32 = T6(3,2);                                             %%% STUDENT %%%
		r33 = T6(3,3);                                             %%% STUDENT %%%
		r11 = T6(1,1);                                             %%% STUDENT %%%

    % Izracunamo ustrezne Eulerjeve kote.
        fi = atan2(r21,r11);                                              %%% STUDENT %%%
        theta = atan2(-r31,sqrt(r32^2+r33^2));                                           %%% STUDENT %%%
        psi = atan2(r32,r33);                                             %%% STUDENT %%%
        
	% Orientacija vrha predstavljena z vektorjem treh ZYX Eulerjevih kotov.
		ea = [fi; theta; psi];                                              %%% STUDENT %%%

	% Lega vrha robota v smislu polozaja XYZ in orientacije predstavljene z
	% ZYX Eulerjevimi koti.
		X = [T6(1,4);T6(2,4);T6(3,4);ea];                                               %%% STUDENT %%%


