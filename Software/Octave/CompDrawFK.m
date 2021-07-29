function CompDrawFK=CompDrawFK()


pkg load symbolic;
clear all;
close all;
clc;
warning("off", "all");
syms L1 L2 L3 L4;
syms alf0 alf1 alf2 alf3 alf4 alf5;
syms a0 a1 a2 a3 a4 a5 a6;
syms d1 d2 d3 d4 d5 d6;
syms t1 t2 t3 t4 t5 t6;
% Target Pose for {6} with respecxt {0}
syms alp bet gam;
syms Px Py Pz; % target coordinate of the origin of {6} with respect {0}
syms T06_Targ; % target pose for {6} with respect {0}
syms T01 T12 T23 T34 T45 T56 T03 T04 T05 T06;
%T06_Targ=[[cos(gam)*cos(bet), cos(gam)*sin(bet)*sin(alp)-sin(gam)*cos(alp), cos(gam)*sin(bet)*cos(alp)+sin(gam)*sin(alp), Px];[sin(gam)*cos(bet),sin(gam)*sin(bet)*sin(alp)+cos(gam)*cos(alp),sin(gam)*sin(bet)*cos(alp)-cos(gam)*sin(alp),Py];[-sin(bet), cos(bet)*sin(alp), cos(bet)*cos(alp), Pz];[0, 0, 0, 1]];
 
T01=T01(t1)=[[cos(t1), -sin(t1), 0, a0]; [sin(t1)*cos(alf0), cos(t1)*cos(alf0), -sin(alf0), -sin(alf0)*d1]; [sin(t1)*sin(alf0), cos(t1)*sin(alf0), cos(alf0), cos(alf0)*d1];[0, 0, 0, 1]];
T12=T12(t2)=[[cos(t2), -sin(t2), 0, a1]; [sin(t2)*cos(alf1), cos(t2)*cos(alf1), -sin(alf1), -sin(alf1)*d2]; [sin(t2)*sin(alf1), cos(t2)*sin(alf1), cos(alf1), cos(alf1)*d2];[0, 0, 0, 1]];
T23=T23(t3)=[[cos(t3), -sin(t3), 0, a2]; [sin(t3)*cos(alf2), cos(t3)*cos(alf2), -sin(alf2), -sin(alf2)*d3]; [sin(t3)*sin(alf2), cos(t3)*sin(alf2), cos(alf2), cos(alf2)*d3];[0, 0, 0, 1]];
T34=T34(t4)=[[cos(t4), -sin(t4), 0, a3]; [sin(t4)*cos(alf3), cos(t4)*cos(alf3), -sin(alf3), -sin(alf3)*d4]; [sin(t4)*sin(alf3), cos(t4)*sin(alf3), cos(alf3), cos(alf3)*d4];[0, 0, 0, 1]];
T45=T45(t5)=[[cos(t5), -sin(t5), 0, a4]; [sin(t5)*cos(alf4), cos(t5)*cos(alf4), -sin(alf4), -sin(alf4)*d5]; [sin(t5)*sin(alf4), cos(t5)*sin(alf4), cos(alf4), cos(alf4)*d5];[0, 0, 0, 1]];
T56=T56(t6)=[[cos(t6), -sin(t6), 0, a5]; [sin(t6)*cos(alf5), cos(t6)*cos(alf5), -sin(alf5), -sin(alf5)*d6]; [sin(t6)*sin(alf5), cos(t6)*sin(alf5), cos(alf5), cos(alf5)*d6];[0, 0, 0, 1]];

% Foot Frame {F} 
L5=50; % in mm
L6=100;
T6F=[[cos(-pi/2), 0, -sin(-pi/2), 0]',[0, 1, 0, 0]',[sin(-pi/2), 0, cos(-pi/2), 0]',[L5, 0, L6, 1]'];


T01=subs(T01,{alf0,a0,d1},{0,0,0});
T12=subs(T12,{alf1,a1,d2},{-pi/2,0,0});
T23=subs(T23,{alf2,a2,d3},{pi/2,0,0});
T34=subs(T34,{alf3,a3,d4},{-pi/2,318,0});
T45=subs(T45,{alf4,a4,d5},{0,318,0});
T56=subs(T56,{alf5,a5,d6},{pi/2,0,0});

%------------------------------------------------------------------------------------
% Angle Substitution, if all zero the frames will be represented in the home position

T01=subs(T01,{t1},{0});
T12=subs(T12,{t2},{pi/2}); % this should additional have pi/2
T23=subs(T23,{t3},{0});
T34=subs(T34,{t4},{0});
T45=subs(T45,{t5},{0});
T56=subs(T56,{t6},{0});

%------------------------------------------------------------------------------------


T02=T01*T12;
T03=T02*T23;
T04=T03*T34;
T05=T04*T45;
T06=T05*T56;

T0F=T06*T6F;

figure;      
hold on;    
grid on;
view(3);
lim=9;
xlim([-lim lim]);
ylim([-lim lim]);
zlim([-lim lim]);
orange=[0.9100    0.4100    0.1700];

% Scaling the frame axis and the dimensions of the robot 
scale1=4;
scale2=2.5;
scale3=1.5;
scale4=1;

% Scaling the link lenghs 
scaleD=80;

orange=[0.9100    0.4100    0.1700];

FontScale=11;


% Frame Zero {0}          
quiver3(0,0,0,scale1,0,0,'r','LineWidth',1);
quiver3(0,0,0,0,scale1,0,'r','LineWidth',1);
quiver3(0,0,0,0,0,scale1,'r','LineWidth',1);
text(scale1,0,0,'X_{0}','Color','red','FontSize',FontScale);
text(0,scale1,0,'Y_{0}','Color','red','FontSize',FontScale);
text(0,0,scale1,'Z_{0}','Color','red','FontSize',FontScale);

% Frame One {1}
quiver3(0,0,0,double(T01(1,1))*scale2,double(T01(2,1))*scale2,double(T01(3,1))*scale2,'b','LineWidth',2);
quiver3(0,0,0,double(T01(1,2))*scale2,double(T01(2,2))*scale2,double(T01(3,2))*scale2,'b','LineWidth',2);
quiver3(0,0,0,double(T01(1,3))*scale2,double(T01(2,3))*scale2,double(T01(3,3))*scale2,'b','LineWidth',2);
text(double(T01(1,1)*scale2),double(T01(2,1)*scale2),double(T01(3,1)*scale2),'X_{1}','Color','blue','FontSize',FontScale);
text(double(T01(1,2)*scale2),double(T01(2,2)*scale2),double(T01(3,2)*scale2),'Y_{1}','Color','blue','FontSize',FontScale);
text(double(T01(1,3)*scale2),double(T01(2,3)*scale2),double(T01(3,3)*scale2),'Z_{1}','Color','blue','FontSize',FontScale);

% Frame Two {2}
quiver3(0,0,0,double(T02(1,1)*scale3),double(T02(2,1)*scale3),double(T02(3,1)*scale3),'g','LineWidth',2);
quiver3(0,0,0,double(T02(1,2)*scale3),double(T02(2,2)*scale3),double(T02(3,2)*scale3),'g','LineWidth',2);
quiver3(0,0,0,double(T02(1,3)*scale3),double(T02(2,3)*scale3),double(T02(3,3)*scale3),'g','LineWidth',2);
text(double(T02(1,1)*scale3),double(T02(2,1)*scale3),double(T02(3,1)*scale3),'X_{2}','Color','green','FontSize',FontScale);
text(double(T02(1,2)*scale3),double(T02(2,2)*scale3),double(T02(3,2)*scale3),'Y_{2}','Color','green','FontSize',FontScale);
text(double(T02(1,3)*scale3),double(T02(2,3)*scale3),double(T02(3,3)*scale3),'Z_{2}','Color','green','FontSize',FontScale);


% Frame Three {3}
quiver3(0,0,0,double(T03(1,1)*scale4),double(T03(2,1)*scale4),double(T03(3,1)*scale4),'color',orange,'LineWidth',2);
quiver3(0,0,0,double(T03(1,2)*scale4),double(T03(2,2)*scale4),double(T03(3,2)*scale4),'color',orange,'LineWidth',2);
quiver3(0,0,0,double(T03(1,3)*scale4),double(T03(2,3)*scale4),double(T03(3,3)*scale4),'color',orange,'LineWidth',2);
text(double(T03(1,1)*scale4),double(T03(2,1)*scale4),double(T03(3,1)*scale4),'X_{3}','color',orange,'FontSize',FontScale);
text(double(T03(1,2)*scale4),double(T03(2,2)*scale4),double(T03(3,2)*scale4),'Y_{3}','color',orange,'FontSize',FontScale);
text(double(T03(1,3)*scale4),double(T03(2,3)*scale4),double(T03(3,3)*scale4),'Z_{3}','color',orange,'FontSize',FontScale);

% Link 3
plot3([0 double(T04(1,4))/scaleD],[0 double(T04(2,4))/scaleD],[0 double(T04(3,4))/scaleD],'--k');
  
% Frame four {4}
quiver3(double(T04(1,4))/scaleD,double(T04(2,4))/scaleD,double(T04(3,4))/scaleD,double(T04(1,1))*scale3,double(T04(2,1))*scale3,double(T04(3,1))*scale3,'color',orange,'LineWidth',2);
quiver3(double(T04(1,4))/scaleD,double(T04(2,4))/scaleD,double(T04(3,4))/scaleD,double(T04(1,2))*scale3,double(T04(2,2))*scale3,double(T04(3,2))*scale3,'color',orange,'LineWidth',2);
quiver3(double(T04(1,4))/scaleD,double(T04(2,4))/scaleD,double(T04(3,4))/scaleD,double(T04(1,3))*scale3,double(T04(2,3))*scale3,double(T04(3,3))*scale3,'color',orange,'LineWidth',2);
text((double(T04(1,4))/scaleD+double(T04(1,1))*scale3),(double(T04(2,4))/scaleD+double(T04(2,1))*scale3),(double(T04(3,4))/scaleD+double(T04(3,1))*scale3),'X_{4}','color',orange,'FontSize',FontScale);
text((double(T04(1,4))/scaleD+double(T04(1,2))*scale3),(double(T04(2,4))/scaleD+double(T04(2,2))*scale3),(double(T04(3,4))/scaleD+double(T04(3,2))*scale3),'Y_{4}','color',orange,'FontSize',FontScale);
text((double(T04(1,4))/scaleD+double(T04(1,3))*scale3),(double(T04(2,4))/scaleD+double(T04(2,3))*scale3),(double(T04(3,4))/scaleD+double(T04(3,3))*scale3),'Z_{4}','color',orange,'FontSize',FontScale);
  
% Link 4
plot3([double(T04(1,4))/scaleD double(T05(1,4))/scaleD],[double(T04(2,4))/scaleD double(T05(2,4))/scaleD],[double(T04(3,4))/scaleD double(T05(3,4))/scaleD],'--k');
  
% Frame Five  {5}
quiver3(double(T05(1,4))/scaleD,double(T05(2,4))/scaleD,double(T05(3,4))/scaleD, double(T05(1,1))*scale3,double(T05(2,1))*scale2,double(T05(3,1))*scale3,'color',orange,'LineWidth',2);
quiver3(double(T05(1,4))/scaleD,double(T05(2,4))/scaleD,double(T05(3,4))/scaleD, double(T05(1,2))*scale3,double(T05(2,2))*scale2,double(T05(3,2))*scale3,'color',orange,'LineWidth',2);
quiver3(double(T05(1,4))/scaleD,double(T05(2,4))/scaleD,double(T05(3,4))/scaleD, double(T05(1,3))*scale3,double(T05(2,3))*scale2,double(T05(3,3))*scale3,'color',orange,'LineWidth',2);
text((double(T05(1,4))/scaleD+double(T05(1,1))*scale2),(double(T05(2,4))/scaleD+double(T05(2,1))*scale3),(double(T05(3,4))/scaleD+double(T05(3,1))*scale3),'X_{5}','color',orange,'FontSize',FontScale);
text((double(T05(1,4))/scaleD+double(T05(1,2))*scale2),(double(T05(2,4))/scaleD+double(T05(2,2))*scale3),(double(T05(3,4))/scaleD+double(T05(3,2))*scale3),'Y_{5}','color',orange,'FontSize',FontScale);
text((double(T05(1,4))/scaleD+double(T05(1,3))*scale2),(double(T05(2,4))/scaleD+double(T05(2,3))*scale3),(double(T05(3,4))/scaleD+double(T05(3,3))*scale3),'Z_{5}','color',orange,'FontSize',FontScale);

% Frame six {6}
quiver3(double(T06(1,4))/scaleD,double(T06(2,4))/scaleD,double(T06(3,4))/scaleD, double(T06(1,1))*scale4,double(T06(2,1))*scale3,double(T06(3,1))*scale4,'g','LineWidth',2);
quiver3(double(T06(1,4))/scaleD,double(T06(2,4))/scaleD,double(T06(3,4))/scaleD, double(T06(1,2))*scale4,double(T06(2,2))*scale3,double(T06(3,2))*scale4,'g','LineWidth',2);
quiver3(double(T06(1,4))/scaleD,double(T06(2,4))/scaleD,double(T06(3,4))/scaleD, double(T06(1,3))*scale4,double(T06(2,3))*scale3,double(T06(3,3))*scale4,'g','LineWidth',2);
text((double(T06(1,4))/scaleD+double(T06(1,1))*scale3),(double(T06(2,4))/scaleD+double(T06(2,1))*scale4),(double(T06(3,4))/scaleD+double(T06(3,1))*scale4),'X_{6}','Color','green','FontSize',FontScale);
text((double(T06(1,4))/scaleD+double(T06(1,2))*scale3),(double(T06(2,4))/scaleD+double(T06(2,2))*scale4),(double(T06(3,4))/scaleD+double(T06(3,2))*scale4),'Y_{6}','Color','green','FontSize',FontScale);
text((double(T06(1,4))/scaleD+double(T06(1,3))*scale3),(double(T06(2,4))/scaleD+double(T06(2,3))*scale4),(double(T06(3,4))/scaleD+double(T06(3,3))*scale4),'Z_{6}','Color','green','FontSize',FontScale);

% Link 6
plot3([double(T06(1,4))/scaleD double(T0F(1,4))/scaleD],[double(T06(2,4))/scaleD double(T0F(2,4))/scaleD],[double(T06(3,4))/scaleD double(T0F(3,4))/scaleD],'--k');
  

% Frame Foot {F}
quiver3(double(T0F(1,4))/scaleD,double(T0F(2,4))/scaleD,double(T0F(3,4))/scaleD, double(T0F(1,1))*scale3,double(T0F(2,1))*scale3,double(T0F(3,1))*scale3,'r','LineWidth',2);
quiver3(double(T0F(1,4))/scaleD,double(T0F(2,4))/scaleD,double(T0F(3,4))/scaleD, double(T0F(1,2))*scale3,double(T0F(2,2))*scale3,double(T0F(3,2))*scale3,'r','LineWidth',2);
quiver3(double(T0F(1,4))/scaleD,double(T0F(2,4))/scaleD,double(T0F(3,4))/scaleD, double(T0F(1,3))*scale3,double(T0F(2,3))*scale3,double(T0F(3,3))*scale3,'r','LineWidth',2);
text((double(T0F(1,4))/scaleD+double(T0F(1,1))*scale3),(double(T0F(2,4))/scaleD+double(T0F(2,1))*scale3),(double(T0F(3,4))/scaleD+double(T0F(3,1))*scale3),'X_{F}','Color','red','FontSize',FontScale);
text((double(T0F(1,4))/scaleD+double(T0F(1,2))*scale3),(double(T0F(2,4))/scaleD+double(T0F(2,2))*scale3),(double(T0F(3,4))/scaleD+double(T0F(3,2))*scale3),'Y_{F}','Color','red','FontSize',FontScale);
text((double(T0F(1,4))/scaleD+double(T0F(1,3))*scale3),(double(T0F(2,4))/scaleD+double(T0F(2,3))*scale3),(double(T0F(3,4))/scaleD+double(T0F(3,3))*scale3),'Z_{F}','Color','red','FontSize',FontScale);

endfunction