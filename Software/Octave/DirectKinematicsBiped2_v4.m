
pkg load symbolic;
syms L1 L2 L3 L4;
syms alf0 alf1 alf2 alf3 alf4 alf5;
syms a0 a1 a2 a3 a4 a5 a6;
syms d1 d2 d3 d4 d5 d6;
syms t1 t2 t3 t4 t5 t6;
syms T01 T12 T23 T34 T45 T56 T06;
syms T01_inv T12_inv T23_inv T34_inv T45_inv T56_inv T06_inv;
syms T03_inv;
% Target Pose for {6} with respecxt {0}
syms alp bet gam;



syms Px Py Pz; % target coordinate of the origin of {6} with respect {0}
syms T06_Targ; % target pose for {6} with respect {0}
T06_Targ=[[cos(gam)*cos(bet), cos(gam)*sin(bet)*sin(alp)-sin(gam)*cos(alp), cos(gam)*sin(bet)*cos(alp)+sin(gam)*sin(alp), Px];[sin(gam)*cos(bet),sin(gam)*sin(bet)*sin(alp)+cos(gam)*cos(alp),sin(gam)*sin(bet)*cos(alp)-cos(gam)*sin(alp),Py];[-sin(bet), cos(bet)*sin(alp), cos(bet)*cos(alp), Pz];[0, 0, 0, 1]];
t1=-59*3.14/180
t2=24*3.14/180
t3=180*3.14/180
t4=45*3.14/180
t5=89*3.14/180
t6=-121*3.14/180
alf0=0
alf1=0
alf2=0
alf3=0
alf4=0
alf5=0
L1=200
L2=155
d2=0
d3=0
d4=0
d5=0
d6=0

T01=[[cos(t1), -sin(t1), 0, 0]; [sin(t1)*cos(alf0), cos(t1)*cos(alf0), -sin(alf0), L2]; [sin(t1)*sin(alf0), cos(t1)*sin(alf0), cos(alf0), -L1];[0, 0, 0, 1]]
%T01=T01(t1)=[[cos(t1), -sin(t1), 0, a0]; [sin(t1)*cos(alf0), cos(t1)*cos(alf0), -sin(alf0), -sin(alf0)*d1]; [sin(t1)*sin(alf0), cos(t1)*sin(alf0), cos(alf0), cos(alf0)*d1];[0, 0, 0, 1]]
T12=[[cos(t2), -sin(t2), 0, a1]; [sin(t2)*cos(alf1), cos(t2)*cos(alf1), -sin(alf1), -sin(alf1)*d2]; [sin(t2)*sin(alf1), cos(t2)*sin(alf1), cos(alf1), cos(alf1)*d2];[0, 0, 0, 1]]
T23=[[cos(t3), -sin(t3), 0, a2]; [sin(t3)*cos(alf2), cos(t3)*cos(alf2), -sin(alf2), -sin(alf2)*d3]; [sin(t3)*sin(alf2), cos(t3)*sin(alf2), cos(alf2), cos(alf2)*d3];[0, 0, 0, 1]]
T34=[[cos(t4), -sin(t4), 0, a3]; [sin(t4)*cos(alf3), cos(t4)*cos(alf3), -sin(alf3), -sin(alf3)*d4]; [sin(t4)*sin(alf3), cos(t4)*sin(alf3), cos(alf3), cos(alf3)*d4];[0, 0, 0, 1]]
T45=[[cos(t5), -sin(t5), 0, a4]; [sin(t5)*cos(alf4), cos(t5)*cos(alf4), -sin(alf4), -sin(alf4)*d5]; [sin(t5)*sin(alf4), cos(t5)*sin(alf4), cos(alf4), cos(alf4)*d5];[0, 0, 0, 1]]
T56=[[cos(t6), -sin(t6), 0, a5]; [sin(t6)*cos(alf5), cos(t6)*cos(alf5), -sin(alf5), -sin(alf5)*d6]; [sin(t6)*sin(alf5), cos(t6)*sin(alf5), cos(alf5), cos(alf5)*d6];[0, 0, 0, 1]]
%T36=T34*T45*T56
%Substitute DH parameters
Tt1=[-141.0408;234.6255;-803.4953;1]
T01=subs(T01,{t1,alf0,a0,d1},{0,0,L2,-L1})
T01*Tt1
T12=subs(T12,{t2,alf1,a1,d2,t2},{0,-pi/2,0,0,t2+pi/2})
T23=subs(T23,{t3,alf2,a2,d3},{0,pi/2,0,0})
T34=subs(T34,{t4,alf3,a3,d4},{0,-pi/2,L3,0})
T45=subs(T45,{t5,alf4,a4,d5},{0,0,L4,0})
T56=subs(T56,{t6,alf5,a5,d6},{0,pi/2,0,0})
T06=T01*T12*T23*T34*T45*T56
% Inverse matrix calculation with simplification
T01_inv=simplify(inv(T01));
T12_inv=simplify(inv(T12));
T23_inv=simplify(inv(T23));
T34_inv=simplify(inv(T34));
T45_inv=simplify(inv(T45));
T56_inv=simplify(inv(T56));
T30=T23_inv*T12_inv*T01_inv
T36=T34*T45*T56
Left=T36*T06_Targ
Right=T30
%Left side:
%This takes time 
%T03_inv=simplify(inv(T23_inv*T12_inv*T01_inv));
%---------------- If we want to start solving for the positin of the origin of {4} with respect to {0}
%T04=T01*T12*T23*T34;T30=T23_inv*T12_inv*T01_inv
T36=T34*T45*T56
T60_targ_2=simplify(inv(T06_Targ))

Left=T36*T60_targ_2
Right=T30
%T14= T12*T23*T34;
%T24= T23*T34;
% The target position for the the origin of {4} with respect to {0}
%PT_left=T01_inv*[Px;Py;Pz;1] % should be equal to  %
%PT_right=T14*[;0;0;1]
% Let us try
%PT_left=simplify(T12_inv*(T01_inv*[Px;Py;Pz;1])) % should be equal to  
%PT_right=T24*[L4;0;0;1]
%----------------
%T60=simplify(T56_inv*T45_inv*T34_inv*T23_inv*T12_inv*T01_inv);
%T60_Targ=inv(T06_Targ);
%T63=simplify(T56_inv*T45_inv*T34_inv);
% We can find a particular numerical solution for the FK
% I tried different angles and the solution was always correct both in orientation of {6} and position of 6ORG with respect to {0} 
%T06=subs(T06,{t1,t2,t3,t4,t5,t6,L1,L2,L3,L4},{0,-pi/2,0,0,0,0,0.2,0.154,0.3185,0.3185})
Right
Right(3,3)
simpify(Right)
simplify(Right)
Left
Right(3,3)
Left(3,3
Left(3,3)
Left(3,3)
Left(3,1)
Left(3,2)
Left(1,3)
Left(2,3)
Left(3,2)
