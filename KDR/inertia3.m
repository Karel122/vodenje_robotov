function B = inertia3(q)

    % Funkcija izracuna vztrajnostno matriko za robot s tremi segmenti.
    % Vhod je vektor kotov v sklepih q.

    % Inicializacijs vztrajnostne matrike.
        B = zeros(3,3);

    % Upostevamo pogoj za pospesek, ki deluje na bazo robota.
        ddp0 = ;                                            %%% STUDENT %%%

    % Upostevamo pogoj za kotne pospeske v sklepih.
        dq = ;                                              %%% STUDENT %%%

    
    % Izracunamo prvi stolpec vztrajnostne matrike B, pri cemer uporabimo
    % funkcijo za izracun inverzne dinamike robota za izracun navorov, 
    % ki predstavljajo stolpec matrike vztrajnosti.
        ddq = ;                                             %%% STUDENT %%%
        B(:,1) = ;                                           %%% STUDENT %%%

    % Izracunamo drugi stolpec vztrajnostne matrike B.
        ddq = ;                                             %%% STUDENT %%%
        B(:,2) = ;                                          %%% STUDENT %%%
        
    % Izracunamo tretji stolpec vztrajnostne matrike B.
        ddq = ;                                             %%% STUDENT %%%
        B(:,3) = ;                                          %%% STUDENT %%%
end