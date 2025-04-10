function J = jacobi0(q)

	% q - Vrednost kotov v sklepih robota (vhod).
	% J - Geometrijska Jacobijeva matrika (izhod).

	% Izracun transformacijskih matrik A.
		A = dirkinA(q);                                              %%% STUDENT %%%

	% Transformacijska matrika lege vrha T5.
		T6 = dirkinT6(A);                                             %%% STUDENT %%%

	% Inicializacija prazne Jacobijeve matrike.
		Jp = zeros(3,6);    % Pozicijska podmatrika.
		Jo = zeros(3,6);    % Orientacijska podmatrika.

	% Inicializacija spremenljivk.
		z_0 = [0,0,1]';     % Vektor v smeri osi z_0.
		p_0 = [0,0,0]';     % Vektor polozaja prvega sklepa glede na ref. k.s.
		Tn = eye(4);        % Zacetna vrednost delne matrike Tn; n = 1,2...

	% 1. STOLPEC JACOBIJEVE PODMATRIKE
		% Izracun 1. stolpca pozicijske Jacobijeve podmatrike.
        
		Jp(:,1) = cross(z_0,T6(1:3,4)-p_0);                                        %%% STUDENT %%%
		% Izracun 1. stolpca rotacijske Jacobijeve podmatrike.
		Jo(:,1) = z_0;                                        %%% STUDENT %%%

	% 2. STOLPEC JACOBIJEVE PODMATRIKE
		% Izracun matrike Tn, ki vsebuje informacijo Zj-1 in Pj-1.
		Tn = A(:,:,1);                                             %%% STUDENT %%%
		% Izracun 2. stolpca pozicijske Jacobijeve podmatrike.
		Jp(:,2) = cross(Tn(1:3,3),T6(1:3,4)-Tn(1:3,4));                                        %%% STUDENT %%%
		% Izracun 2. stolpca rotacijske Jacobijeve podmatrike.
		Jo(:,2) = Tn(1:3,3);                                        %%% STUDENT %%%

	% 3. STOLPEC JACOBIJEVE PODMATRIKE
		% Izracun matrike Tn, ki vsebuje informacijo Zj-1 in Pj-1.
		Tn = A(:,:,1)*A(:,:,2);                                             %%% STUDENT %%%
		% Izracun 3. stolpca pozicijske Jacobijeve podmatrike.
		Jp(:,3) = cross(Tn(1:3,3),T6(1:3,4)-Tn(1:3,4));                                        %%% STUDENT %%%
		% Izracun 3. stolpca rotacijske Jacobijeve podmatrike.
		Jo(:,3) = Tn(1:3,3);                                        %%% STUDENT %%%
		
	% 4. STOLPEC JACOBIJEVE PODMATRIKE
		% Izracun matrike Tn, ki vsebuje informacijo Zj-1 in Pj-1.
		Tn = Tn*A(:,:,3);                                             %%% STUDENT %%%
		% Izracun 4. stolpca pozicijske Jacobijeve podmatrike.
		Jp(:,4) = cross(Tn(1:3,3),T6(1:3,4)-Tn(1:3,4));                                        %%% STUDENT %%%
		% Izracun 4. stolpca rotacijske Jacobijeve podmatrike.
		Jo(:,4) = Tn(1:3,3);                                        %%% STUDENT %%%
		
	% 5. STOLPEC JACOBIJEVE PODMATRIKE
		% Izracun matrike Tn, ki vsebuje informacijo Zj-1 in Pj-1.
		Tn = Tn*A(:,:,4);                                             %%% STUDENT %%%
		% Izracun 5. stolpca pozicijske Jacobijeve podmatrike.
		Jp(:,5) = cross(Tn(1:3,3),T6(1:3,4)-Tn(1:3,4));                                        %%% STUDENT %%%
		% Izracun 5. stolpca rotacijske Jacobijeve podmatrike.
		Jo(:,5) = Tn(1:3,3);                                        %%% STUDENT %%%
        
    % 6. STOLPEC JACOBIJEVE PODMATRIKE
    % Izracun matrike Tn, ki vsebuje informacijo Zj-1 in Pj-1.
        Tn = Tn*A(:,:,5);                                             %%% STUDENT %%%
        % Izracun 6. stolpca pozicijske Jacobijeve podmatrike.
        Jp(:,6) = cross(Tn(1:3,3),T6(1:3,4)-Tn(1:3,4));                                        %%% STUDENT %%%
        % Izracun 6. stolpca rotacijske Jacobijeve podmatrike.
        Jo(:,6) = Tn(1:3,3);                                        %%% STUDENT %%%

		
	% Celotna Jacobijeva matrika je sestavljena iz obeh podmatrik.
		J = [Jp; Jo];