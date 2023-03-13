clear all; clc;
%creating matrices A,B,C,D
A = [0 1; -1000 -40];   %-k/J = -1000  -c/J = -40
B = [0; 5]              %Kt/J = 5 
C = [1 0];              %x1=thita -> y=exit=position of head=thita -> y=x1
D =0;

%%%%%%%
%%%A%%%
%%%%%%%

%printing eigenvalues with eig()
eigenvalues = eig(A)

%a homogenous linear system X'=AX has a zero solution
%the stability of this solution is determined by theorems
%on our case if the real parts of the matrix's eigenvalues are 
%negative then our system is asymptotically stable 

%creating our system in order to show it's asymptotically stable
%it's close on zero but with a data cursor we can see it doesn't get to 0
sys = ss(A,B,C,0);      %state space model
t= 0:0.1:1;             %time sample 
u = zeros(size(t));     %u = 0
x0 = [0.01 0];          %initial conditions
[y,t,x] = lsim(sys,u,t,x0); %signal creation
plot(t,y)
grid on
title('Zero input and trivial solution x=0')

%%%%%%%
%%%B%%%
%%%%%%%

T = 0:0.01:1;           %time sample 
U = ones(size(T));      %unit step (since time is  positive it works like a unit step)
sys = ss(A,B,C,0);      %state-space model
[y,T,x] = lsim(sys,U,T);%signal without x0 since we dont have initial conditions
figure(2)
plot(T,y) 
title('Zero Initial Conditions and Unit Step Input')

