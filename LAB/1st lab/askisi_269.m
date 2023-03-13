clear all; clc;
%creating matrices A,B,C,D
A = [0 1 0 0; -0.1910 -0.0536 0.0910 0.0036;
    0 0 0 1; 0.0910 0.0036 -0.1910 -0.0536];

B = [0 0; 1 0; 0 0; 0 -1];

C = [1 0 0 0; 0 0 1 0];

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

t=0:0.01:250;       %time space
u = zeros(size(t)); %zero input (f1,f2 = 0)
u = repmat(u,2,1);  %u needs to be a 2x1 matrix
x0=[0.01 0 0 0];    %initial conditions
sys=ss(A,B,C,0);    %state space model
[y,t,x] = lsim(sys,u,t,x0); %signal
figure(1)
plot(t,y)           %plot of x(t)
title('Non-Zero Initial Conditions and No Inputs')

%%%%%%%
%%%B%%%
%%%%%%%

t=0:0.01:250;       %time space
u = zeros(size(t)); %zero input (f1,f2 = 0)
u = repmat(u,2,1);  %u needs to be a 2x1 matrix
X0=[1 0 -0.5 0];    %initial conditions x(0)
sys=ss(A,B,C,0);    %state space model
[y,t,x] = lsim(sys,u,t,X0); %signal
figure(2)
plot(t,y)           %plot of x(t)
title('Specific Initial Conditions and No Inputs')

%%%%%%%
%%%D%%%
%%%%%%%


sys=ss(A,B,C,0);    %state space model
figure(3)
[y,t,x] = impulse(sys); %zero initial conditions so just impulse response
plot(t,y(:,1,1),'r')    %plot of output1 from f1(t)=d(t)
hold on
plot(t,y(:,1,2),'b')    %plot of output2 from f1(t)=d(t)
hold off
legend('Out1', 'Out2');
title('Zero Initial Conditions and Unit Impulse in First Input')


