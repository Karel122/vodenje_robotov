%% Create env
vpisna_stevilka = 649901670;
rng(vpisna_stevilka) 
n = 4;

klet = -1*ones(n,n);

for i=1:n
    for j=1:n
        if (rand() < 0.25)
            klet(i,j) = -n;
        end
    end
end
klet(1,1) = -1;
klet(1,2) = -1;
klet(2,1) = -1;
klet(2,2) = -1;
klet(n-1,n-1) = -1;
klet(n,n-1) = -1;
klet(n-1,n) = -1;
klet(n,n) = n;

% Render environment
disp(klet)

fh = figure;
imagesc(klet);
colormap(copper);

for i=1:n
    for j=1:n
        
        if (i==1) && (j == 1)
            text(1,1,{'1','START'},'HorizontalAlignment','center');
        elseif (i==n) && (j==n)
            text(n,n,{num2str(n*n),'GOAL'},'HorizontalAlignment','center')
        else
            text(j,i,num2str(i+n*(j-1)),'HorizontalAlignment','center')
        end
    end
end

axis off

%%
%Vaša koda


%%
% Vizualizacija rešitve
indexQ = int32([(1:(n*n))]');
visQ = table(indexQ,Q)

num_steps = vizualizacija_Q4(Q, klet);
num_steps = visualization_Q_arrows4(Q, klet);

