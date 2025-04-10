function tau = torque(ddq, dq, q, ddp0)
    % Funkcija izracuna inverzni dinamicni model za manipulator s tremi 
    % rotacijskimi prostostnimi stopnjami. 
    % Vhodni vektorji so polozaji v sklepih q, sklepne hitrosti dq, 
    % sklepni pospeski ddq in vektor ddp0, ki doloca pospesek baze robota
    % Izhod funkcije je vektor sklepnih navorov tau.

    % Mase segmentov robota.
        m = [0 1.318 0.821];                
    % Vektorji od sklepa i-1 do sklepa i.
        r = [[0.033 0 0.147]' [0.155 0 0]' [0.135 0 0]'];
    % Vektorji tezisc segmentov glede na lokalni koordinatni sistem.
        rC = [[0.01516 0.00359 0.03105]' [0.11397 0.0150 -0.01903]' [0.10441 0.00013 0.02022]'];
    % Vztrajnostne matrike segmentov.
        I = zeros(3,3,3);    
        I(:,:,1) = [0.0029525 0 0; 0 0.0060091 0; 0 0 0.0058821];
        I(:,:,2) = [0.0031145 0 0; 0 0.0005843 0; 0 0 0.0031631]; 
        I(:,:,3) = [0.00041967 0 0; 0 0.00172767 0; 0 0 0.0018468];
    % Parametri viskoznega trenja za posamezne sklepe.
        fv = [2 1 1];
    % Struktura z vsemi robotskimi parametri (npr. masa prvega segmenta 
    % je rob.m(1)).
        rob = struct('m', m, 'rC', rC, 'r', r, 'I', I, 'fv', fv);

    % Izracun transformacijskih matrik za prve tri segmente.
        d1=0.147;
        a1=0.033;
        a2=0.155;
        a3=0.135; 
        %AA=dirkin(q)
        A = zeros(4,4,3);
        A(:,:,1) = hdh(q(1),d1,a1,-pi/2);                                      %%% STUDENT %%%
        A(:,:,2) = hdh(q(2)-pi/2,0,a2,0);                                     %%% STUDENT %%%
        A(:,:,3) = hdh(q(3),0,a3,0);                                     %%% STUDENT %%%

    % Vektor v smeri osi z0.
        z0 = [0 0 1]';
    % Inicializacija vektorja navora.
        tau = zeros(3,1);

    % Dolocitev rotacijskih matrik kot podmatrik matrik A.
        R = zeros(3,3,3);
        R(:,:,1) = A(1:3,1:3,1);                                       %%% STUDENT %%%
        R(:,:,2) = A(1:3,1:3,2);                                        %%% STUDENT %%%
        R(:,:,3) = A(1:3,1:3,3);                                        %%% STUDENT %%%

    % VSE INICIALIZACIJE V NADALJEVANJU SO IZVEDENE ZA VSE TRI SKLEPE IN
    % SEGMENTE ROBOTA, ZATO SO UPORABLJENE MATRIKE DIMENZIJ 3x3, KJER VSAK
    % STOLPEC PREDSTAVLJA VEKTOR, KI OPISUJE EN SEGMENT ALI SKLEP ROBOTA.

    % Inicializacija vektorjev kotnih hitrosti segmentov.
        omega = zeros(3,3);
    % Inicializacija vektorjev kotnih pospeskov segmentov.
        domega = zeros(3,3);
    % Inicializacija vektorjev transl. pospeskov koordinatnih sistemov
    % segmentov.
        ddp = zeros(3,3);
    % Inicializacija vektorjev transl. pospeskov tezisc segmentov.
        ddpC = zeros(3,3);
    % Inicializacija vektorjev sile v sklepih.
        f = zeros(3,3);
    % Inicializacija vektorjev navora v sklepih.
        mi = zeros(3,3);

    % Izracun kinematicnih velicin.
    for ii = 1:3
        if (ii == 1) % Robni pogoj za prvi sklep.
            % Kotna hitrost segmenta.
                omega(:,ii) = R(:,:,ii)'*(dq(ii)*z0);                           %%% STUDENT %%%
            % Kotni pospesek segmenta.        
                domega(:,ii) = R(:,:,ii)'*(ddq(ii)*z0);                          %%% STUDENT %%%
            % Translacijski pospesek segmenta.
                ddp(:,ii) =R(:,:,ii)'*ddp0+cross(domega(:,ii),r(:,ii))+cross(omega(:,ii),cross(omega(:,ii),r(:,ii)));                                %%% STUDENT %%%        
        else
            % Kotna hitrost segmenta.
                omega(:,ii) = R(:,:,ii)'*(omega(:,ii-1)+dq(ii)*z0);                             %%% STUDENT %%%
            % Kotni pospesek segmenta.                
                domega(:,ii) = R(:,:,ii)'*(domega(:,ii-1)+ddq(ii)*z0+cross(dq(ii)*omega(:,ii-1),z0));                            %%% STUDENT %%%
            % Translacijski pospesek segmenta.
                ddp(:,ii) = R(:,:,ii)'*ddp(:,ii-1)+cross(domega(:,ii),r(:,ii))+cross(omega(:,ii),cross(omega(:,ii),r(:,ii)));                              %%% STUDENT %%%        
        end
        % Translacijski pospesek tezisca segmenta.
            ddpC(:,ii) = ddp(:,ii)+cross(domega(:,ii),rC(:,ii))+cross(omega(:,ii),cross(omega(:,ii),rC(:,ii)));                                  %%% STUDENT %%%
    end

    % Izracun dinamicnih velicin.
    for ii = 3:-1:1
        if (ii==3) % Robni pogoj za silo na vrhu.
            % Sila, ki deluje na sklep.
                f(:,ii) = m(ii)*ddpC(:,ii);                                  %%% STUDENT %%%
            % Navor, ki deluje na sklep.
                mi(:,ii) = -cross(f(:,ii),(r(:,ii)+rC(:,ii)))+I(:,:,ii)*domega(:,ii)+cross(omega(:,ii),I(:,:,ii)*omega(:,ii));                                %%% STUDENT %%%
        else
            % Sila, ki deluje na sklep.
                f(:,ii) = R(:,:,ii+1)*f(:,ii+1)+m(ii)*ddpC(:,ii);                                 %%% STUDENT %%%
            % Navor, ki deluje na sklep.
                mi(:,ii) = -cross(f(:,ii),(r(:,ii)+rC(:,ii)))+R(:,:,ii+1)*mi(:,ii+1)+cross(R(:,:,ii+1)*f(:,ii+1),rC(:,ii))+I(:,:,ii)*domega(:,ii)+cross(omega(:,ii),I(:,:,ii)*omega(:,ii));                                  %%% STUDENT %%%        
        end
        % Navor, ki deluje v osi sklepa (obremenjuje motor).
            tau(ii) = mi(:,ii)'*R(:,:,ii)'*z0+fv(ii)*dq(ii);                                     %%% STUDENT %%%
    end