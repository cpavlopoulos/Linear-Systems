clear all; clc;
%creating matrices A,B,C,D
A = [-1.7 (-2.13)*10^(-4) 0; 696 2.9 2.4; 0 6.5 -19.5];
B = [0; 0; -0.16];
C = [0 1 0];
D = 0;

%%%%%%%
%%%A%%%
%%%%%%%

%printing eigenvalues with eig()
eigenvalues = eig(A)

%creating the eigenvectors
[V, D] = eig(A);

%each collumn of matrix V is an eigenvector 4
eigenvector1=V(:,1)
eigenvector2=V(:,2)
eigenvector3=V(:,3)

%%%%%%%
%%%B%%%
%%%%%%%

T = 0:0.01:4;           %time space
U = ones(size(T));      %unit step input (since time is  positive it works like a unit step)
sys = ss(A,B,C,0);      %state-space model
[y,T] = lsim(sys,U,T);  %signal creation without x0 since we have no initial conditions
figure(1)
plot(T,y) 
title('Zero Initial Conditions and Unit Step Input')

%%%%%%%
%%%C%%%
%%%%%%%

t = 0:0.01:4;           %time space
u = zeros(size(t));     %zero input
x0 = [0.1 0 0];         %initial conditions
sys = ss(A,B,C,0);      %state space model
[y,t,x] = lsim(sys,u,t,x0); %signal creation
figure(2)
plot(t,y) 
title('Initial concentration sligthly higher that EQ point')



