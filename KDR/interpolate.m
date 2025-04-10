function [time, p] = interpolate(t_c, t_f, a_c, dT, p_i, p_f)

    % t_c - Cas pospesevanja in zaviranja.
    % t_f - Celoten cas gibanja.
    % a_c - Pospesek/pojemek.
    % dT  - Vzrocni cas interpolatorja.
    % p_i - Zacetni polozaj.
    % p_f - Koncni polozaj.

    % Definiramo vektor casa za posamezne odseke interpolacije.
        t1 = 0 :dT:t_c;												%%% STUDENT %%%
        t2 = (t_c+dT):dT:(t_f-t_c);								                %%% STUDENT %%%
        t3 = (t_f-t_c+ dT):dT:t_f;						                        %%% STUDENT %%%

    % Izracun interpolirane poti za pospesevanje, enakomerno 
    % hitrost in zaviranje.
        p1 = p_i + 1/2*a_c*(t1.^2);						                        %%% STUDENT %%%
        p2 = p_i + a_c*t_c*(t2-t_c/2) ;					                            %%% STUDENT %%%
        p3 = p_f-1/2*a_c*((t_f-t3).^2);												%%% STUDENT %%%
        
    % Sestavimo vektorje v samostojni vektor.
        time = [t1,t2,t3];
        p = [p1,p2,p3];


