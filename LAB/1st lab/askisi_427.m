clear all; clc;
%creating matrices A,B,C,D
A = [0 11 0 0; 1 0 0 0; 0 -1 0 0; 0 0 1 0];
B = [-1; 0; 1; 0];
C = [ 1 0 0 0 ];
D = 0;

%%%%%%%
%%%A%%%
%%%%%%%

%printing eigenvalues with eig()
eigenvalues = eig(A)

%creating the eigenvectors
[V, D] = eig(A)

%each collumn of matrix V is an eigenvector 4
eigenvector1=V(:,1)
eigenvector2=V(:,2)
eigenvector3=V(:,3)
eigenvector4=V(:,4)

%%%%%%%
%%%B%%%
%%%%%%%

t = 0:0.01:5;           %time space
u = zeros(size(t));     %zero input
x0 = [0.01 0 0 0];      %initial conditions
sys = ss(A,B,C,0);      %state space model
[y,t,x] = lsim(sys,u,t,x0); %signal creation
figure(1)
plot(t,y) 
title('Non-Zero Initial Conditions and No Input')


T = 0:0.01:5;           %time space
U = ones(size(T));      %unit step input (since time is  positive it works like a unit step)
sys = ss(A,B,C,0);      %state-space model
[y,T] = lsim(sys,U,T);%signal creation without x0 since we have no initial conditions
figure(2)
plot(T,y) 
title('Zero Initial Conditions and Unit Step Input')


