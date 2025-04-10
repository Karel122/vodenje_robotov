function A = dirkinA(q)

    % q - Vektor kotnih spremenljivk za 6 sklepov (vhod).
    % A(:,:,1) ... A(:,:,6) - Lista matrik A (izhod).

     % Parameteri robota Motoman MH5
        a1 = 0.088;
        a2 = -0.310;
        a3 = -0.040;
        d1 = 0.131;
        d4 = 0.305;
        d6 = 0.0865;


    % Definiranje matrik A (5 prostostnih stopenj).
        A = zeros(4,4,6);
        A(:,:,1) = hdh(q(1), d1, a1, -pi/2);                                   
        A(:,:,2) = hdh(q(2)+pi/2, 0, a2, 0);                                   
        A(:,:,3) = hdh(q(3), 0, a3, pi/2);                                   
        A(:,:,4) = hdh(q(4), d4, 0, -pi/2);                                   
        A(:,:,5) = hdh(q(5), 0, 0, pi/2);                                   
        A(:,:,6) = hdh(q(6), d6, 0, 0) ;                                   
                                          
