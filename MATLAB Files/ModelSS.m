m = 0.768;
g = 9.81;
l = 0.1524;
I = 0.0177;
omega = 209.44;
m_flywheel = 0.334;
r = 0.06;
I_flywheel = 0.5*m*r^2;

A = [0 1; -(1/I)*(m*g*l) 0];
B = [0 0; 0 1/I];
C = [1 0];
D = [0 0];

sys = ss(A,B,C,D);
val = eig(A);

G = tf([omega*I_flywheel 0],[I 0 m*g*l]);
pzmap(G)