clear all; clc;
%creating matrices A,B,C,D
A = [-0.0507 -3.861 0 -32.2; -0.00117 -0.5164 1 0; 
    -0.000129 1.4168 -0.4932 0; 0 0 1 0]
B = [0; -0.0717; -1.645; 0]
C = [0 0 1 1]
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


t = 0:0.01:10;          %time space
sys = ss(A,B,C,0);      %state space model
x0=[0.01 0 0 0];
u=zeros(size(t));
[Y,t,x] = lsim(sys,u,t,x0); %signal creation
figure(1)
plot(t,Y) 
title('Expressing system')

%%%%%%%
%%%B%%%
%%%%%%%

t = 0:0.01:20;          %time space
sys = ss(A,B,C,0);      %state space model
figure(2)
step(-1*sys,t)          %adding -1 elevator deflection as a feedback on system
title('Zero input/ Elevator deflection -1 /Zero initial conditions')

%%%%%%%
%%%C%%%
%%%%%%%

t = 0:0.01:20;           %time space
u = -1.*ones(size(t));   %negative unit input
sys = ss(A,B,C,0);       %state space model
[Y,t,x] = lsim(sys,u,t); %signal creation
figure(3)
plot(t,Y) 
title('Negative unit step input/Zero initial conditions')





