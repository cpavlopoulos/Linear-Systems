clear all; clc;
%creating matrices A,B,C,D
A = [-0.0507 -3.861 0 -32.2; -0.00117 -0.5164 1 0; 
    -0.000129 1.4168 -0.4932 0; 0 0 1 0];
B = [0; -0.0717; -1.645; 0];
C = [0 0 1 1];
D = 0;
sys = ss(A,B,C,0);

co = ctrb(A,B);

Controllability = rank(co)

%% A
t = 0:0.01:500;          %time space
x0=[0.01 0 0 0];
r=zeros(size(t));

p1 = -1.25 + 2.2651i;
p2 = -1.25 - 2.2651i;
p3 = -0.01 + 0.095i;
p4 = -0.01 - 0.095i;

K = place(A,B,[p1 p2 p3 p4])
sys_cl = ss(A-B*K,B,C,0);

figure(1)
lsim(sys_cl,r,t,x0);
title('Closed loop system with pole placement');

Nbar=rscale(sys,K)

%% B
C = [0 0 1 0];

op1=0;
op2=-0.421;
op3=-0.587;
op4=-1;

L=place(A',C',[op1 op2 op3 op4])'

At = [ A-B*K             B*K
       zeros(size(A))    A-L*C ]

Bt = [    B*Nbar
       zeros(size(B)) ]

Ct = [ C    zeros(size(C)) ]

sys = ss(At,Bt,Ct,0);

figure(2)
lsim(sys,zeros(size(t)),t,[x0 x0]);
title('Linear Simulation Results (with observer)')

%% C

s = tf('s');
sia=(s*eye(8,8))-(At);
F=inv(sia);
tf=Ct*F*Bt

%% D
p=100;
Q=(p*C')*C
R=1;
[K]=lqr(A,B,Q,R)

sys_lqr=ss(A-B*K, B, C, D);

figure(3)
step(sys_lqr)
title('Closed-Loop Step Response: LQR');

