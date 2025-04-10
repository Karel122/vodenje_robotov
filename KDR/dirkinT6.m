function T6 = dirkinT6(A)


    % MOTOMAN MH5
    % A(:,:,1) ... A(:,:,6) - Lista matrik A (vhod).
    % T6 - Matrika lege vrha robota (izhod).

   T6 = A(:,:,1) * A(:,:,2) * A(:,:,3) * A(:,:,4) * A(:,:,5) * A(:,:,6);                                                

