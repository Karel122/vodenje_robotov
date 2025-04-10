    % Zbrisemo stare podatke.
    clear all; close all; clc;
    
    % Vzrocni cas.
        dT = 1/4000;

    % Zacetna lega robota.
        q0 = [0 0 0 0 90 0]'*pi/180;

    % Izracun lege vrha robota pri vrednostih kotnih spremenljivk q0.
        p0 = q2ZYXeul(q0); 
        pY0 = round(p0(2)*100000)/100000;
        pZ0 = round(p0(3)*100000)/100000;

    % Sprememba koncne pozicije vrha robota (v metrih).
        dpY = 0.15;
        dpZ = -0.10;

    % Zeljena hitrost gibanja (m/s).
        v_c = 0.0500000;

    % Pospesek in pojemek giba (m/s2).
        a_c = 0.250000;

    % Odlociti se je potrebno kateri premik je daljsi.
        if(dpY >= dpZ)
            p_i_Dolg = pY0;
            p_f_Dolg = pY0 + dpY;
            p_i_Kratek = pZ0;
            p_f_Kratek = pZ0 + dpZ;            
        else
            p_i_Dolg = pZ0;
            p_f_Dolg = pZ0 + dpZ;
            p_i_Kratek = pY0;
            p_f_Kratek = pY0 + dpY;            
        end
        
    % *** IZRACUN ZA DALSI GIB ***
    % Izracun casa pospesevanja.
        t_c = v_c/a_c;                                       %%% STUDENTI %%%

    % Izracun polozaja ob koncu pospesevanja.
        p_c = 1/2*a_c*(t_c^2);                                       %%% STUDENTI %%%
    
    %  Izracun dolzine giba pri konstantni hitrosti.
        d_l = (p_f_Dolg-p_i_Dolg)-2*p_c;                                       %%% STUDENTI %%%
  
    % Ce je dolzina linearnega dela negativna, je pospesek prevelik.
        if(d_l < 0 )
            sprintf('Pospesek je prevelik!')
            %break
        end
    
    % Izracun casa giba pri konstantni hitrosti.
        t_l = d_l/v_c  ;                                       %%% STUDENTI %%%
    
    % Celoten cas giba - cas pospesevanje je enak casu zaviranja.
        t_f = t_l + 2*t_c  ;                                       %%% STUDENTI %%%
    
    % Izracun interpolirane poti za koordinato daljse poti.
        [time1,p1] = interpolate(t_c,t_f,a_c,dT,p_i_Dolg,p_f_Dolg);                               %%% STUDENTI %%%

    % IZRACUN ZA KRAJSI GIB        
    % Izracun pospeska za koordinato kratke poti.
        a_c =    (p_f_Kratek-p_i_Kratek)/(t_c^2+t_c*(t_f-2*t_c));                                   %%% STUDENTI %%%

    % Izracun interpolirane poti za koordinato kratke poti.
        [time2,p2] = interpolate(t_c,t_f,a_c,dT,p_i_Kratek,p_f_Kratek);                               %%% STUDENTI %%%
    
    % Izris interpolirane poti, hitrosti in pospeska za eno smer
        % Pot
        figure
        subplot(3,1,1)
        plot(time1,p1,'.-b');
        title([sprintf('Izracunan premik: %.3f m',abs(p1(1)-p1(end)))])
        xlabel('t (s)');
        ylabel('p1 (mm)');
        grid on
    
        % Hitrost
        subplot(3,1,2)
        s1 = diff(p1)/dT;
        plot(time1(1:end-1),s1,'.-b');
        title([sprintf('Izracunana maksimalna hitrost: %.3f m/s',max(s1))])
        xlabel('t (s)');
        ylabel('v1 (mm/s)');
        grid on

        % Pospesek
        subplot(3,1,3)
        a1 = diff(s1)/dT;
        plot(time1(1:end-2),a1,'.-b');
        title([sprintf('Izracunan pospesek: %.3f m/s2',max(a1))])    
        xlabel('t (s)');
        ylabel('a1 (mm/s^2)');
        grid on

    % Izris interpolirane poti, hitrosti in pospeska za drugo smer
        % Pot
        figure
        subplot(3,1,1)
        plot(time2,p2,'.-b');
        title([sprintf('Izracunan premik: %.3f m',abs(p2(1)-p2(end)))])
        xlabel('t (s)');
        ylabel('p1 (mm)');
        grid on
    
        % Hitrost
        subplot(3,1,2)
        s2 = diff(p2)/dT;
        plot(time2(1:end-1),s2,'.-b');
        title([sprintf('Izracunana maksimalna hitrost: %.3f m/s',max(s2))])
        xlabel('t (s)');
        ylabel('v2 (mm/s)');
        grid on

        % Pospesek
        subplot(3,1,3)
        a2 = diff(s2)/dT;
        plot(time2(1:end-2),a2,'.-b');
        title([sprintf('Izracunan pospesek: %.3f m/s2',max(a2))])
        xlabel('t (s)');
        ylabel('a2 (mm/s^2)');
        grid on
    
    % Izris interpolirane poti v obeh koordinatah
        figure    
        plot(p1,p2,'.-b');
        title('Trajektorija v YZ ravnini');
        xlabel('Y (mm)');
        ylabel('Z (mm)');
        axis equal,
        grid on
 
    % Sestavimo matriko odvisnosti poti od casa
        fbGain = 20;
        xTrajec = [time1',ones(length(time1),1)*0.3930,p1',p2',ones(length(time1),1)*3.1416,ones(length(time1),1)*0,ones(length(time1),1)*3.1416];        
        
        