clear all; close all; clc;

%creating matrices A,B,C,D
A = [-0.0507 -3.861 0 -32.2; -0.00117 -0.5164 1 0; 
    -0.000129 1.4168 -0.4932 0; 0 0 1 0]
B = [0; -0.0717; -1.645; 0]
C = [0 0 1 1]
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

Nbar=rscale(sys,K)

figure()
lsim(sys_cl,r,t,x0);
title('Closed loop system with pole placement');

%% B

x0_a=[0 0.1 0 0];
r=zeros(size(t));
figure()
lsim(sys_cl,r,t,x0_a);
title('Closed loop system with x0 [0 0.1 0 0]');

x0_b=[0 -0.1 0 0];
r=zeros(size(t));
figure()
lsim(sys_cl,r,t,x0_b);
title('Closed loop system with x0 [0 -0.1 0 0]');

%% Ca

t = 0:0.01:500;          %time space
figure()
step(-1*sys_cl,t)          %adding -1 elevator deflection as a feedback on system
title('Closed loop system with elevator deflection -1 input')

%% Cb

t = 0:0.01:500;           %time space
u2 = -1.*ones(size(t));   %negative unit input
[Y,t,x] = lsim(sys_cl,u2,t); %signal creation
figure()
plot(t,Y) 
title('Closed loop system with negative unit step input')

%% D

C = [0 0 1 0]

%desired poles
op1=-0.1;
op2=-0.421;
op3=-0.587;
op4=-1;

%calculation of L
L=place(A',C',[op1 op2 op3 op4])'

At = [ A-B*K             B*K
       zeros(size(A))    A-L*C ]

Bt = [    B*Nbar
       zeros(size(B)) ]

Ct = [ C    zeros(size(C)) ]

sys = ss(At,Bt,Ct,0);

figure()
lsim(sys,zeros(size(t)),t,[x0 x0]);
title('Full order state observer with poles at -0.1, -0.421, -0.587, -1')

%% E

poles=eig(A-L*C) 
x0_bar = [0.2 -0.1 0.1 -0.1];
e_a=x0_a-x0_bar
e_b=x0_b-x0_bar

t = 0:0.01:100;          %time space
r=zeros(size(t));

et_a=(A-L*C)*e_a'
et_b=(A-L*C)*e_b'

figure()
lsim(sys,r,t,[et_a' et_a']);
title('Error for X0 [0, 0.1, 0, 0]');

figure()
lsim(sys,r,t,[et_b' et_b']);
title('Error for X0 [0, -0.1, 0, 0]');

%% Z

P =[-0.6 -0.7 -0.8 -0.9];

L=place(A',C',P)'

At2 = [ A-B*K             B*K
       zeros(size(A))    A-L*C ]

Bt2 = [    B*Nbar
       zeros(size(B)) ]

Ct2 = [ C    zeros(size(C)) ]

sys2 = ss(At2,Bt2,Ct2,0);

figure()
lsim(sys2,zeros(size(t)),t,[e_a e_a]);
title('Question Z for X0 [0, 0.1, 0, 0]')

figure()
lsim(sys2,zeros(size(t)),t,[e_b e_b]);
title('Question Z for X0 [0, -0.1, 0, 0]')

%% H
s = tf('s');
sia=(s*eye(8,8))-(At);
F=inv(sia);
tf=Ct*F*Bt


%% Th

p=100;
Q=(p*C')*C
R=1;
[K]=lqr(A,B,Q,R)

sys_lqr=ss(A-B*K, B, C, D);

figure()
step(sys_lqr)
title('Closed-Loop Step Response: LQR');

P=pole(sys_lqr)