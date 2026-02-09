function Rd = Rd_Matrix_rigid(bodies,Rj)
I_til = [0 -1;
    1 0];

for i = 1:length( bodies)
    n =  bodies(i).bran;
    rj = Rj(:,n(end));%    rj = Rj(:,n(i));
    Rdi = [-I_til*rj
               1];   
    Rd(3*(i-1)+1:3*i,i) = Rdi;
    
end