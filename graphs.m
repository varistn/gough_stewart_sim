clc;clear;close;
%KINEMATICS
%Characteristics of the manipulator
alpha=[-90,30,150]*pi/180;  % Definition of base platform
L=1.5;                      % base length (centre to joint)
LL=1;                       % mobile length (centre to joint)
P=1;                        % Link first length prismatic lower
PP =1;                      % Length second Length

%%%%%%%%%%%%%%%%%%%%
%INVERSE KINEMATICS%
%%%%%%%%%%%%%%%%%%%%
x0 = 0; y0 = 0; z0 = 0;
pixelConv = 1000;

%Point 1   2     3     4    5     6     7     8     9    10   11
P_ee=[0    0    350   700  350    0    -350  -700  -500  -350 0;    %x
      0    0    -100  -200 300    500  250   0     -150  100  0;    %y
      2500 2500 2750  3000 2750   2500 2750  3000  3000  2500 2500; %z
      0    0    -15   -30  10     40   10    -20   -30   -20  0;    %alpha
      0    0    15    30   15     0    0     0     5     10   0;    %beta
      0    30   5     -20  -10    0    10    20    20    15   0];   %gamma
% n-1 element, where n is the number of columns of P_ee
tf=[.5 .5 .5 .5 .5 .5 .5 .5 .5 .5];
%time(0:0.05:5.45);
% Original Points
% P_ee=[0      700     0     -700;    %x
%       0      -200    500   0;       %y
%       2500   3000    2500  3000;    %z
%       0      -30     40    -20;     %alpha
%       0       30     0     0;       %beta
%       0       -20    0     20];     %gamma
% tf=[.5 .5 .5];

% TRAJECTORY GENERATION. Trajectory generation of all the joints
% 0.01 is too long, 0.05 is not as smooth?
dt=0.05; %stepsize
[position,velocity,acceleration,time]=via_points_match_VA(P_ee, tf, dt, 'prescribed',[0,0]);
%comet3(position(1,:) , position(2,:) , position(3,:) )
%plot(time,velocity)
%plot(time,acceleration)



for j =1:length(position(1,:))
    
    % Extracts the positions/ orientations from interpolated points
    xee = position(1,j);
    yee = position(2,j);
    zee = position(3,j);
    alpha = position(4,j);
    beta = position(5,j);
    gamma = position(6,j);

    R0_ee=[    cosd(beta)*cosd(gamma)                                 ,       -sind(beta)      , cosd(beta)*sind(gamma);...
       cosd(alpha)*sind(beta)*cosd(gamma)+sind(alpha)*sind(gamma) , cosd(alpha)*cosd(beta) , cosd(alpha)*sind(beta)*sind(gamma)-sind(alpha)*cosd(gamma);...
       sind(alpha)*sind(beta)*cosd(gamma)-cosd(alpha)*sind(gamma) , sind(alpha)*cosd(beta) , sind(alpha)*sind(beta)*sind(gamma)+cosd(alpha)*cosd(gamma)];

%WRITE HERE THE INVERSE KINEMATICS FOR ALL THE THREE BRANCHES, SOLVE FOR
%THE THREE ANGLES

    % Specify Constant
    numberofChain = 6;
    platformLength = 1*pixelConv; %[m]
    baseLength = 1.5*pixelConv; %[m]

%     % Arrays Preallocation 
%     d = cell(numberofChain,1);
%     d_length = zeros(numberofChain,1);
%     dDH = zeros(numberofChain,1);
%     theta1 = zeros(numberofChain,1);
%     theta2 = zeros(numberofChain,1);
%     Pp0_p = cell(numberofChain,1);
%     Pb0_b = cell(numberofChain,1);
%     Pb0_pi = zeros(3,numberofChain);

    % Position vector from base to platform
    Pb0_p0 = [xee;yee;zee]; %[x ; y ; z]

    % thetab is the angle from the position vector of each 
    % base (universal) joints to x base coordinate
    % thetap is the angle from the position vector of each 
    % platform (spherical) joints x platform coordinate
    thetab(1) = -135; thetap(1) = -105;
    thetab(2) = -45; thetap(2) = -75;
    thetab(3) = -15; thetap(3) = 15;
    thetab(4) = 75; thetap(4) = 45;
    thetab(5) = 105; thetap(5) = 135;
    thetab(6) = -165; thetap(6) = 165;

    for i = 1:numberofChain
        Pp0_p{i} = [platformLength*cosd(thetap(i));platformLength*sind(thetap(i));0];
        Pb0_b{i} = [baseLength*cosd(thetab(i));baseLength*sind(thetab(i));0];
        % d is the prismatic joint vector wrt the base
        d{i} = Pb0_p0 + R0_ee*Pp0_p{i} - Pb0_b{i};
        d_length(i) = norm(d{i});
    end

    beta = [-45;45;75;165;-165;-75]; % beta is the angle from x'0 to global x0

    % Revolute angle theta1 and theta2 are the joints that makes up the
    % universal joints at the base
    % Numerically known values to compare with the symbolic expressions
    for i=1:numberofChain
        Pb0_pi(1:3,i) = Pb0_p0 + R0_ee*Pp0_p{i};
    end

    for i = 1:numberofChain
        a = Pb0_pi(3,i);
        b = -(cosd(beta(i))*Pb0_pi(1,i) + sind(beta(i))*Pb0_pi(2,i));
        %theta1 has 2 solutions
        theta1(i) = atan2(a,-b);
        a = (sind(beta(i))*Pb0_pi(1,i) - cosd(beta(i))*Pb0_pi(2,i) - baseLength)^2;
        b = ((cosd(beta(i))*Pb0_pi(1,i) + sind(beta(i))*Pb0_pi(2,i))*cos(theta1(i)) + Pb0_pi(3,i)*sin(theta1(i)))^2;
        %d has 2 solutions
        dDH(i) = sqrt(a+b);
        a = ((cosd(beta(i))*Pb0_pi(1,i) + sind(beta(i))*Pb0_pi(2,i))*cos(theta1(i)) + Pb0_pi(3,i)*sin(theta1(i)))/dDH(i);
        b = (sind(beta(i))*Pb0_pi(1,i) - cosd(beta(i))*Pb0_pi(2,i) - baseLength)/dDH(i);
        %theta2 has 1 solution
        theta2(i) = atan2(a,b);
    end

    %Solve the INVERSE KINEMATICS for the jth posture and store
    %the joint displacements in a (18x1)vector, say joints_j, which
    %includes the joint displacements for branches 1 through 6:
    joints_j =[theta1(1); theta2(1); dDH(1);
               theta1(2); theta2(2); dDH(2);
               theta1(3); theta2(3); dDH(3);
               theta1(4); theta2(4); dDH(4);
               theta1(5); theta2(5); dDH(5);
               theta1(6); theta2(6); dDH(6)];
               
    %Store vectors in matrix form for each end-effector?s position
     joints(:,j) = joints_j;
end




for k=1:length(position(1,:)) %Make sure you don?t use k afterwards. 
    
     % Extracting joints displacement from inverse kinematics
    theta1_1=joints(1,k);
    theta2_1=joints(2,k);
    L_1 = joints(3,k);
    theta1_2=joints(4,k);
    theta2_2=joints(5,k);
    L_2 = joints(6,k);
    theta1_3=joints(7,k);
    theta2_3=joints(8,k);
    L_3 = joints(9,k);
    theta1_4=joints(10,k);
    theta2_4=joints(11,k);
    L_4 = joints(12,k);
    theta1_5=joints(13,k);
    theta2_5=joints(14,k);
    L_5 = joints(15,k);
    theta1_6=joints(16,k);
    theta2_6=joints(17,k);
    L_6 = joints(18,k);
    
    Ridentity = [1 0 0; 0 1 0; 0 0 1];
    beta = [-45;45;75;165;-165;-75]; % beta is the angle from x'0 to global x0
    a = [0;0;0;0;0];
    alp = [0;90;-90;90;0];
    %LINK 1
    T00Link1 = tmat(alp(1),a(1),0,beta(1));
    T01Link1 = tmat(alp(2),a(2),baseLength,theta1_1*180/pi);
    T12Link1 = tmat(alp(3),a(3),0,theta2_1*180/pi);
    T23Link1 = tmat(alp(4),a(4),0,0);
    T3eeLink1 = tmat(alp(5),a(5),L_1,0);
    T0eeLink1 = T00Link1*T01Link1*T12Link1*T23Link1*T3eeLink1;
    %LINK 2
    T00Link2 = tmat(alp(1),a(1),0,beta(2));
    T01Link2 = tmat(alp(2),a(2),baseLength,theta1_2*180/pi);
    T12Link2 = tmat(alp(3),a(3),0,theta2_2*180/pi);
    T23Link2 = tmat(alp(4),a(4),0,0);
    T3eeLink2 = tmat(alp(5),a(5),L_2,0);
    T0eeLink2 = T00Link2*T01Link2*T12Link2*T23Link2*T3eeLink2;
    %LINK 3
    T00Link3 = tmat(alp(1),a(1),0,beta(3));
    T01Link3 = tmat(alp(2),a(2),baseLength,theta1_3*180/pi);
    T12Link3 = tmat(alp(3),a(3),0,theta2_3*180/pi);
    T23Link3 = tmat(alp(4),a(4),0,0);
    T3eeLink3 = tmat(alp(5),a(5),L_3,0);
    T0eeLink3 = T00Link3*T01Link3*T12Link3*T23Link3*T3eeLink3;
    %LINK 4
    T00Link4 = tmat(alp(1),a(1),0,beta(4));
    T01Link4 = tmat(alp(2),a(2),baseLength,theta1_4*180/pi);
    T12Link4 = tmat(alp(3),a(3),0,theta2_4*180/pi);
    T23Link4 = tmat(alp(4),a(4),0,0);
    T3eeLink4 = tmat(alp(5),a(5),L_4,0);
    T0eeLink4 = T00Link4*T01Link4*T12Link4*T23Link4*T3eeLink4;
    %LINK 5
    T00Link5 = tmat(alp(1),a(1),0,beta(5));
    T01Link5 = tmat(alp(2),a(2),baseLength,theta1_5*180/pi);
    T12Link5 = tmat(alp(3),a(3),0,theta2_5*180/pi);
    T23Link5 = tmat(alp(4),a(4),0,0);
    T3eeLink5 = tmat(alp(5),a(5),L_5,0);
    T0eeLink5 = T00Link5*T01Link5*T12Link5*T23Link5*T3eeLink5;
    %LINK 6
    T00Link6 = tmat(alp(1),a(1),0,beta(6));
    T01Link6 = tmat(alp(2),a(2),baseLength,theta1_6*180/pi);
    T12Link6 = tmat(alp(3),a(3),0,theta2_6*180/pi);
    T23Link6 = tmat(alp(4),a(4),0,0);
    T3eeLink6 = tmat(alp(5),a(5),L_6,0);
    T0eeLink6 = T00Link6*T01Link6*T12Link6*T23Link6*T3eeLink6;

    %Platform position for each configurations
    xnew = position(1,k);
    ynew = position(2,k);
    znew = position(3,k);
    Pnew = [xnew; ynew; znew];
    
    %vectors
    P1 = T0eeLink1(1:3,4) - Pnew;
    P2 = T0eeLink2(1:3,4) - Pnew;
    P3 = T0eeLink3(1:3,4) - Pnew;
    P4 = T0eeLink4(1:3,4) - Pnew;
    P5 = T0eeLink5(1:3,4) - Pnew;
    P6 = T0eeLink6(1:3,4) - Pnew;
    
    P4unit = P4/norm(P4);
    P5unit = P5/norm(P5);
    Y(1,1) = (P4(1,1)+P5(1,1))/2;
    Y(2,1) = (P4(2,1)+P5(2,1))/2;
    Y(3,1) = (P4(3,1)+P5(3,1))/2;
    
    %Unit vectos
    Yunit = Y/norm(Y);
    Zunit = cross(P4unit,P5unit);
    Xunit = cross(Yunit,Zunit);
    % Rotation matrix for each configurations
    R0_ee = [Xunit Yunit Zunit];
    
    Jq{k}=[d_length(1) 0 0 0 0 0;
         0 d_length(2) 0 0 0 0;
         0 0 d_length(3) 0 0 0;
         0 0 0 d_length(4) 0 0;
         0 0 0 0 d_length(5) 0;
         0 0 0 0 0 d_length(6)];
     Px=xee; Py=yee; Pz=zee;
     alpha = position(4,k);beta = position(5,k);gamma = position(6,k);
     
     %BRANCH 1
     Ppx=P1(1);Ppy=P1(2);Pbx=Pb0_b{1}(1);Pby=Pb0_b{1}(2);
     Jx11=2*Px - 2*Pbx - 2*Ppy*sin(beta) + 2*Ppx*cos(beta)*cos(gamma);
     Jx12=2*Py - 2*Pby + 2*Ppy*cos(alpha)*cos(beta) + 2*Ppx*sin(alpha)*sin(gamma) + 2*Ppx*cos(alpha)*cos(gamma)*sin(beta);
     Jx13=2*Pz + 2*Ppy*cos(beta)*sin(alpha) - 2*Ppx*cos(alpha)*sin(gamma) + 2*Ppx*cos(gamma)*sin(alpha)*sin(beta);
     Jx14=2*Ppy*Pz*cos(alpha)*cos(beta) + 2*Pby*Ppy*cos(beta)*sin(alpha) - 2*Ppy*Py*cos(beta)*sin(alpha) - 2*Pby*Ppx*cos(alpha)*sin(gamma) + 2*Ppx*Py*cos(alpha)*sin(gamma) + 2*Ppx*Pz*sin(alpha)*sin(gamma) + 2*Ppx*Pz*cos(alpha)*cos(gamma)*sin(beta) + 2*Pby*Ppx*cos(gamma)*sin(alpha)*sin(beta) - 2*Ppx*Py*cos(gamma)*sin(alpha)*sin(beta);
     Jx15=2*Pbx*Ppy*cos(beta) - 2*Ppy*Px*cos(beta) + 2*Pby*Ppy*cos(alpha)*sin(beta) - 2*Ppy*Py*cos(alpha)*sin(beta) + 2*Pbx*Ppx*cos(gamma)*sin(beta) - 2*Ppx*Px*cos(gamma)*sin(beta) - 2*Ppy*Pz*sin(alpha)*sin(beta) - 2*Pby*Ppx*cos(alpha)*cos(beta)*cos(gamma) + 2*Ppx*Py*cos(alpha)*cos(beta)*cos(gamma) + 2*Ppx*Pz*cos(beta)*cos(gamma)*sin(alpha);
     Jx16=2*Ppx*Py*cos(gamma)*sin(alpha) - 2*Pby*Ppx*cos(gamma)*sin(alpha) - 2*Ppx*Pz*cos(alpha)*cos(gamma) + 2*Pbx*Ppx*cos(beta)*sin(gamma) - 2*Ppx*Px*cos(beta)*sin(gamma) + 2*Pby*Ppx*cos(alpha)*sin(beta)*sin(gamma) - 2*Ppx*Py*cos(alpha)*sin(beta)*sin(gamma) - 2*Ppx*Pz*sin(alpha)*sin(beta)*sin(gamma);
     
     %BRANCH 2
     Ppx=P2(1);Ppy=P2(2);Pbx=Pb0_b{2}(1);Pby=Pb0_b{2}(2);
     Jx21=2*Px - 2*Pbx - 2*Ppy*sin(beta) + 2*Ppx*cos(beta)*cos(gamma);
     Jx22=2*Py - 2*Pby + 2*Ppy*cos(alpha)*cos(beta) + 2*Ppx*sin(alpha)*sin(gamma) + 2*Ppx*cos(alpha)*cos(gamma)*sin(beta);
     Jx23=2*Pz + 2*Ppy*cos(beta)*sin(alpha) - 2*Ppx*cos(alpha)*sin(gamma) + 2*Ppx*cos(gamma)*sin(alpha)*sin(beta);
     Jx24=2*Ppy*Pz*cos(alpha)*cos(beta) + 2*Pby*Ppy*cos(beta)*sin(alpha) - 2*Ppy*Py*cos(beta)*sin(alpha) - 2*Pby*Ppx*cos(alpha)*sin(gamma) + 2*Ppx*Py*cos(alpha)*sin(gamma) + 2*Ppx*Pz*sin(alpha)*sin(gamma) + 2*Ppx*Pz*cos(alpha)*cos(gamma)*sin(beta) + 2*Pby*Ppx*cos(gamma)*sin(alpha)*sin(beta) - 2*Ppx*Py*cos(gamma)*sin(alpha)*sin(beta);
     Jx25=2*Pbx*Ppy*cos(beta) - 2*Ppy*Px*cos(beta) + 2*Pby*Ppy*cos(alpha)*sin(beta) - 2*Ppy*Py*cos(alpha)*sin(beta) + 2*Pbx*Ppx*cos(gamma)*sin(beta) - 2*Ppx*Px*cos(gamma)*sin(beta) - 2*Ppy*Pz*sin(alpha)*sin(beta) - 2*Pby*Ppx*cos(alpha)*cos(beta)*cos(gamma) + 2*Ppx*Py*cos(alpha)*cos(beta)*cos(gamma) + 2*Ppx*Pz*cos(beta)*cos(gamma)*sin(alpha);
     Jx26=2*Ppx*Py*cos(gamma)*sin(alpha) - 2*Pby*Ppx*cos(gamma)*sin(alpha) - 2*Ppx*Pz*cos(alpha)*cos(gamma) + 2*Pbx*Ppx*cos(beta)*sin(gamma) - 2*Ppx*Px*cos(beta)*sin(gamma) + 2*Pby*Ppx*cos(alpha)*sin(beta)*sin(gamma) - 2*Ppx*Py*cos(alpha)*sin(beta)*sin(gamma) - 2*Ppx*Pz*sin(alpha)*sin(beta)*sin(gamma);
     
     %BRANCH 3
     Ppx=P3(1);Ppy=P3(2);Pbx=Pb0_b{3}(1);Pby=Pb0_b{3}(2);
     Jx31=2*Px - 2*Pbx - 2*Ppy*sin(beta) + 2*Ppx*cos(beta)*cos(gamma);
     Jx32=2*Py - 2*Pby + 2*Ppy*cos(alpha)*cos(beta) + 2*Ppx*sin(alpha)*sin(gamma) + 2*Ppx*cos(alpha)*cos(gamma)*sin(beta);
     Jx33=2*Pz + 2*Ppy*cos(beta)*sin(alpha) - 2*Ppx*cos(alpha)*sin(gamma) + 2*Ppx*cos(gamma)*sin(alpha)*sin(beta);
     Jx34=2*Ppy*Pz*cos(alpha)*cos(beta) + 2*Pby*Ppy*cos(beta)*sin(alpha) - 2*Ppy*Py*cos(beta)*sin(alpha) - 2*Pby*Ppx*cos(alpha)*sin(gamma) + 2*Ppx*Py*cos(alpha)*sin(gamma) + 2*Ppx*Pz*sin(alpha)*sin(gamma) + 2*Ppx*Pz*cos(alpha)*cos(gamma)*sin(beta) + 2*Pby*Ppx*cos(gamma)*sin(alpha)*sin(beta) - 2*Ppx*Py*cos(gamma)*sin(alpha)*sin(beta);
     Jx35=2*Pbx*Ppy*cos(beta) - 2*Ppy*Px*cos(beta) + 2*Pby*Ppy*cos(alpha)*sin(beta) - 2*Ppy*Py*cos(alpha)*sin(beta) + 2*Pbx*Ppx*cos(gamma)*sin(beta) - 2*Ppx*Px*cos(gamma)*sin(beta) - 2*Ppy*Pz*sin(alpha)*sin(beta) - 2*Pby*Ppx*cos(alpha)*cos(beta)*cos(gamma) + 2*Ppx*Py*cos(alpha)*cos(beta)*cos(gamma) + 2*Ppx*Pz*cos(beta)*cos(gamma)*sin(alpha);
     Jx36=2*Ppx*Py*cos(gamma)*sin(alpha) - 2*Pby*Ppx*cos(gamma)*sin(alpha) - 2*Ppx*Pz*cos(alpha)*cos(gamma) + 2*Pbx*Ppx*cos(beta)*sin(gamma) - 2*Ppx*Px*cos(beta)*sin(gamma) + 2*Pby*Ppx*cos(alpha)*sin(beta)*sin(gamma) - 2*Ppx*Py*cos(alpha)*sin(beta)*sin(gamma) - 2*Ppx*Pz*sin(alpha)*sin(beta)*sin(gamma);
     
     %BRANCH 4
     Ppx=P4(1);Ppy=P4(2);Pbx=Pb0_b{4}(1);Pby=Pb0_b{4}(2);
     Jx41=2*Px - 2*Pbx - 2*Ppy*sin(beta) + 2*Ppx*cos(beta)*cos(gamma);
     Jx42=2*Py - 2*Pby + 2*Ppy*cos(alpha)*cos(beta) + 2*Ppx*sin(alpha)*sin(gamma) + 2*Ppx*cos(alpha)*cos(gamma)*sin(beta);
     Jx43=2*Pz + 2*Ppy*cos(beta)*sin(alpha) - 2*Ppx*cos(alpha)*sin(gamma) + 2*Ppx*cos(gamma)*sin(alpha)*sin(beta);
     Jx44=2*Ppy*Pz*cos(alpha)*cos(beta) + 2*Pby*Ppy*cos(beta)*sin(alpha) - 2*Ppy*Py*cos(beta)*sin(alpha) - 2*Pby*Ppx*cos(alpha)*sin(gamma) + 2*Ppx*Py*cos(alpha)*sin(gamma) + 2*Ppx*Pz*sin(alpha)*sin(gamma) + 2*Ppx*Pz*cos(alpha)*cos(gamma)*sin(beta) + 2*Pby*Ppx*cos(gamma)*sin(alpha)*sin(beta) - 2*Ppx*Py*cos(gamma)*sin(alpha)*sin(beta);
     Jx45=2*Pbx*Ppy*cos(beta) - 2*Ppy*Px*cos(beta) + 2*Pby*Ppy*cos(alpha)*sin(beta) - 2*Ppy*Py*cos(alpha)*sin(beta) + 2*Pbx*Ppx*cos(gamma)*sin(beta) - 2*Ppx*Px*cos(gamma)*sin(beta) - 2*Ppy*Pz*sin(alpha)*sin(beta) - 2*Pby*Ppx*cos(alpha)*cos(beta)*cos(gamma) + 2*Ppx*Py*cos(alpha)*cos(beta)*cos(gamma) + 2*Ppx*Pz*cos(beta)*cos(gamma)*sin(alpha);
     Jx46=2*Ppx*Py*cos(gamma)*sin(alpha) - 2*Pby*Ppx*cos(gamma)*sin(alpha) - 2*Ppx*Pz*cos(alpha)*cos(gamma) + 2*Pbx*Ppx*cos(beta)*sin(gamma) - 2*Ppx*Px*cos(beta)*sin(gamma) + 2*Pby*Ppx*cos(alpha)*sin(beta)*sin(gamma) - 2*Ppx*Py*cos(alpha)*sin(beta)*sin(gamma) - 2*Ppx*Pz*sin(alpha)*sin(beta)*sin(gamma);
     %BRANCH 5
     Ppx=P5(1);Ppy=P5(2);Pbx=Pb0_b{5}(1);Pby=Pb0_b{5}(2);
     Jx51=2*Px - 2*Pbx - 2*Ppy*sin(beta) + 2*Ppx*cos(beta)*cos(gamma);
     Jx52=2*Py - 2*Pby + 2*Ppy*cos(alpha)*cos(beta) + 2*Ppx*sin(alpha)*sin(gamma) + 2*Ppx*cos(alpha)*cos(gamma)*sin(beta);
     Jx53=2*Pz + 2*Ppy*cos(beta)*sin(alpha) - 2*Ppx*cos(alpha)*sin(gamma) + 2*Ppx*cos(gamma)*sin(alpha)*sin(beta);
     Jx54=2*Ppy*Pz*cos(alpha)*cos(beta) + 2*Pby*Ppy*cos(beta)*sin(alpha) - 2*Ppy*Py*cos(beta)*sin(alpha) - 2*Pby*Ppx*cos(alpha)*sin(gamma) + 2*Ppx*Py*cos(alpha)*sin(gamma) + 2*Ppx*Pz*sin(alpha)*sin(gamma) + 2*Ppx*Pz*cos(alpha)*cos(gamma)*sin(beta) + 2*Pby*Ppx*cos(gamma)*sin(alpha)*sin(beta) - 2*Ppx*Py*cos(gamma)*sin(alpha)*sin(beta);
     Jx55=2*Pbx*Ppy*cos(beta) - 2*Ppy*Px*cos(beta) + 2*Pby*Ppy*cos(alpha)*sin(beta) - 2*Ppy*Py*cos(alpha)*sin(beta) + 2*Pbx*Ppx*cos(gamma)*sin(beta) - 2*Ppx*Px*cos(gamma)*sin(beta) - 2*Ppy*Pz*sin(alpha)*sin(beta) - 2*Pby*Ppx*cos(alpha)*cos(beta)*cos(gamma) + 2*Ppx*Py*cos(alpha)*cos(beta)*cos(gamma) + 2*Ppx*Pz*cos(beta)*cos(gamma)*sin(alpha);
     Jx56=2*Ppx*Py*cos(gamma)*sin(alpha) - 2*Pby*Ppx*cos(gamma)*sin(alpha) - 2*Ppx*Pz*cos(alpha)*cos(gamma) + 2*Pbx*Ppx*cos(beta)*sin(gamma) - 2*Ppx*Px*cos(beta)*sin(gamma) + 2*Pby*Ppx*cos(alpha)*sin(beta)*sin(gamma) - 2*Ppx*Py*cos(alpha)*sin(beta)*sin(gamma) - 2*Ppx*Pz*sin(alpha)*sin(beta)*sin(gamma);
     
     %BRANCH 6
     Ppx=P6(1);Ppy=P6(2);Pbx=Pb0_b{6}(1);Pby=Pb0_b{6}(2);
     Jx61=2*Px - 2*Pbx - 2*Ppy*sin(beta) + 2*Ppx*cos(beta)*cos(gamma);
     Jx62=2*Py - 2*Pby + 2*Ppy*cos(alpha)*cos(beta) + 2*Ppx*sin(alpha)*sin(gamma) + 2*Ppx*cos(alpha)*cos(gamma)*sin(beta);
     Jx63=2*Pz + 2*Ppy*cos(beta)*sin(alpha) - 2*Ppx*cos(alpha)*sin(gamma) + 2*Ppx*cos(gamma)*sin(alpha)*sin(beta);
     Jx64=2*Ppy*Pz*cos(alpha)*cos(beta) + 2*Pby*Ppy*cos(beta)*sin(alpha) - 2*Ppy*Py*cos(beta)*sin(alpha) - 2*Pby*Ppx*cos(alpha)*sin(gamma) + 2*Ppx*Py*cos(alpha)*sin(gamma) + 2*Ppx*Pz*sin(alpha)*sin(gamma) + 2*Ppx*Pz*cos(alpha)*cos(gamma)*sin(beta) + 2*Pby*Ppx*cos(gamma)*sin(alpha)*sin(beta) - 2*Ppx*Py*cos(gamma)*sin(alpha)*sin(beta);
     Jx65=2*Pbx*Ppy*cos(beta) - 2*Ppy*Px*cos(beta) + 2*Pby*Ppy*cos(alpha)*sin(beta) - 2*Ppy*Py*cos(alpha)*sin(beta) + 2*Pbx*Ppx*cos(gamma)*sin(beta) - 2*Ppx*Px*cos(gamma)*sin(beta) - 2*Ppy*Pz*sin(alpha)*sin(beta) - 2*Pby*Ppx*cos(alpha)*cos(beta)*cos(gamma) + 2*Ppx*Py*cos(alpha)*cos(beta)*cos(gamma) + 2*Ppx*Pz*cos(beta)*cos(gamma)*sin(alpha);
     Jx66=2*Ppx*Py*cos(gamma)*sin(alpha) - 2*Pby*Ppx*cos(gamma)*sin(alpha) - 2*Ppx*Pz*cos(alpha)*cos(gamma) + 2*Pbx*Ppx*cos(beta)*sin(gamma) - 2*Ppx*Px*cos(beta)*sin(gamma) + 2*Pby*Ppx*cos(alpha)*sin(beta)*sin(gamma) - 2*Ppx*Py*cos(alpha)*sin(beta)*sin(gamma) - 2*Ppx*Pz*sin(alpha)*sin(beta)*sin(gamma);
     Jx{k}=[Jx11 Jx12 Jx13 Jx14 Jx15 Jx16;
            Jx21 Jx22 Jx23 Jx24 Jx25 Jx26;
            Jx31 Jx32 Jx33 Jx34 Jx35 Jx36;
            Jx41 Jx42 Jx43 Jx44 Jx45 Jx46;
            Jx51 Jx52 Jx53 Jx54 Jx55 Jx56;
            Jx61 Jx62 Jx63 Jx64 Jx65 Jx66];
     xdot{k}=velocity(:,k);
     qdot{k}=inv(Jq{k})*Jx{k}*xdot{k};
     weight=100;
     F=[0 0 weight 0 0 0]';
     Torque{k}=inv(Jq{k})*Jx{k}*F;
     %Torque{k}= F*velocity(:,k);
end
for i=1:110
qdotL1(i)=qdot{i}(1);
qdotL2(i)=qdot{i}(2);
qdotL3(i)=qdot{i}(3);
qdotL4(i)=qdot{i}(4);
qdotL5(i)=qdot{i}(5);
qdotL6(i)=qdot{i}(6);
torque(i)=Torque{i}(1);
torque2(i)=Torque{i}(2);
torque3(i)=Torque{i}(3);
torque4(i)=Torque{i}(4);
torque5(i)=Torque{i}(5);
torque6(i)=Torque{i}(6);


end
figure(1)
plot(time(1,:),torque)
xlabel('time');
ylabel('torque');
title('branch1');

figure(2)
plot(time(1,:),torque2)
xlabel('time');
ylabel('torque');
title('branch2');
figure(3)
plot(time(1,:),torque3)
xlabel('time');
ylabel('torque');
title('branch3');
figure(4)
plot(time(1,:),torque4)
xlabel('time');
ylabel('torque');
title('branch4');

figure(5)
plot(time(1,:),torque5)
xlabel('time');
ylabel('torque');
title('branch5');
figure(6)
plot(time(1,:),torque6)
xlabel('time');
ylabel('torque');
title('branch6');


% figure(1)
% plot(time(1,:),qdotL1)
% xlabel('time');
% title('joint rates branch1')
% figure(2)
% plot(time(2,:),qdotL2)
% xlabel('time');
% title('joint rates branch2')
% figure(3)
% plot(time(3,:),qdotL3)
% xlabel('time');
% title('joint rates branch3')
% figure(4)
% plot(time(4,:),qdotL4)
% xlabel('time');
% title('joint rates branch4')
% figure(5)
% plot(time(5,:),qdotL5)
% xlabel('time');
% title('joint rates branch5')
% figure(6)
% plot(time(6,:),qdotL6)
% xlabel('time');
% title('joint rates branch6')

for i = 1:110-1
    a = joints(16,i);
    b = joints(16,i+1);
    t_i = time(1,i);
    t_f = time(1,i+1);
    a2 = joints(17,i);
    b2 = joints(17,i+1);
    t_i2 = time(1,i);
    t_f2 = time(1,i+1);
    v(1,i) = (b-a)/(t_f-t_i);
    v(2,i) = (b2-a2)/(t_f2-t_i2);
end
for i = 1:108
    a = v(1,i);
    b = v(1,i+1);
    t_i = time(1,i);
    t_f = time(1,i+1);
    a2 = v(2,i);
    b2 = v(2,i+1);
    t_i2 = time(1,i);
    t_f2 = time(1,i+1);
    a(1,i) = (b-a)/(t_f-t_i);
    a(2,i) = (b2-a2)/(t_f2-t_i2);
end

% syms Px Py Pz Ppx Ppy Pbx Pby alpha beta gamma
% R0_p0=[cos(beta)*cos(gamma)                                 ,       -sin(beta)      , cos(beta)*sin(gamma);...
%        cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma) , cos(alpha)*cos(beta) , cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma);...
%        sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma) , sin(alpha)*cos(beta) , sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma)];
% P_b0_p0=[Px; Py; Pz];
% P_p0_pi=[Ppx; Ppy; 0];
% P_b0_bi=[Pbx; Pby; 0];
% P_bi_pi=simplify(sum(expand((P_b0_p0+R0_p0*P_p0_pi-P_b0_bi).^2)))
% 
% dPx=diff(P_bi_pi,Px)
% dPy=diff(P_bi_pi,Py)
% dPz=diff(P_bi_pi,Pz)
% dalpha=diff(P_bi_pi,alpha)
% dbeta=diff(P_bi_pi,beta)
% dgamma=diff(P_bi_pi,gamma)






% plot(time(1,1:108),a)
% xlabel('time');
% ylabel('joints acceleration')
% title('Branch6 Joint Acceleration')
% legend('theta1','theta2')
% figure(1)
% plot(time,joints(16,:)*180/pi);
% hold on
% plot(time,joints(17,:)*180/pi);
% xlabel('time');
% ylabel('joints deg')
% title('Branch6')
% legend('theta1','theta2')
% 
% figure(2)
% plot(time,joints(3,:));
% hold on
% plot(time,joints(6,:));
% plot(time,joints(9,:));
% plot(time,joints(12,:));
% plot(time,joints(15,:));
% plot(time,joints(18,:));
% xlabel('time');
% ylabel('prismatic length')
% title('All branch')
% %legend('theta1','theta2')

