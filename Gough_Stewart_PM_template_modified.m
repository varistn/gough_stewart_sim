%1.1. Define size of figure and create figure handle (DO NOT MODIFY)
close all, clear all;

set(0,'Units','pixels'); %unit of length is in pixel
dim = get(0,'ScreenSize'); 
fig_handle = figure('doublebuffer','on','Position',[0,35,dim(3),dim(4)-100],...
    'Name','3D Object','NumberTitle','off');
set(gcf,'color', [1 1 1]) %Background Colour

%1.2 Define the light in the figure (CHANGE POSITION VECTOR IF FIGURE IS TOO BRIGHT/DARK)
set(fig_handle,'Renderer','zbuffer','doublebuffer','off')
light('color',[.5,.5,.5],'position',[0,1,3],'Style','infinite')
lighting gouraud
daspect([1 1 1]);
axis off
view(60,30)

%Arrows (CHANGE PARAMETERS IF THEY ARE TOO SMALL OR TOO BIG)
% You need to have the file arrow3 in the same directory
   arrow_length=400; hold on
   line([0,0],[0,0], [0,arrow_length]); text(0,0,arrow_length*1.1,'z_0','FontSize',14); 
   line([0,0],[0,arrow_length],[0,0]); text(0,arrow_length*1.1, 0,'y_0','FontSize',14); 
   line([0,arrow_length],[0,0],[0,0]); text(arrow_length*1.1, 0, 0,'x_0','FontSize',14); 

% Convert figure into Object (LOAD YOUR PARTS)       
load('base.mat');
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{1}=object; 

load('topwithchair.mat');
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{2}=object; 

load('lowerJoint.mat');
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{3}=object; 

load('upperJoint.mat');
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{4}=object; 

for i = 5:10
    load('lowerTube.mat');
    setappdata(0,'object_data',object);
    object = getappdata(0,'object_data');
    obj{i}=object; 
end

for i = 11:16
    load('upperTube.mat');
    setappdata(0,'object_data',object);
    object = getappdata(0,'object_data');
    obj{i}=object; 
end

for i=1:16  %(CHANGE if you have 10 parts, change it to i=1:10)
    q(i) = patch('faces', obj{i}.F, 'vertices', obj{i}.V);
    set(q(i),'EdgeColor','none');
end

%Set colour to the componenets (CHANGE colours of new parts)
set(q(1),'FaceColor', [1 0 0]);
set(q(2),'FaceColor', [.6,0.6,1]);
set(q(3),'FaceColor', [.2,.2,.2]);
set(q(4),'FaceColor', [.3,0.2,1]);
for i = 5:10
    set(q(i),'FaceColor', [1 1 1]);
end

%       -x    x    -y    y   -z    z
axis([-2000 2000 -2000 2000 -500 4000]);

% ANIMATION
% Animation (DO NOT CHANGE)
RGB=256; %Resolution
fm = getframe; [img,map] = rgb2ind(fm.cdata,RGB,'nodither');

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

%Output result in table 
% PrismaticLength=d_length;
% PrismaticLength_DH=dDH;
% Revolute1Rad=theta1;
% Revolute2Rad=theta2;
% RevoluteAngle1=theta1*180/pi;
% RevoluteAngle2=theta2*180/pi;
% ChainLink=[1;2;3;4;5;6];
% 
% disp('Comparing PrismaticLength with solution from DH');
% disp(table(ChainLink,PrismaticLength,PrismaticLength_DH));
% disp('Revolute angle in Radian');
% disp(table(ChainLink,Revolute1Rad,Revolute2Rad));
% disp('Inverse Kinematics solutions');
% disp(table(ChainLink,PrismaticLength,RevoluteAngle1,RevoluteAngle2));

%Velocity





% ANIMATION (DO NOT CHANGE)
 n=length(position(1,:));
 mov(1:length(n)) = struct('cdata', [],'colormap', []);
 [a,b]=size(img); gifim=zeros(a,b,1,n-1,'uint8');
 
%%%%%%%%%%%%%%%%%%%%
%FORWARD KINEMATICS%
%%%%%%%%%%%%%%%%%%%%

%FORWARD KINEMATICS
%NEW LOOP
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
    
    %Moving Parts
    %Rename of all the vertices. This step is redundant obj{}.V will not longer be used.  
    for i=1:16 %(CHANGE n=2 for the number of parts that you have)
        V{i} = obj{i}.V'; 
    end

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
    
    xnew = position(1,k);
    ynew = position(2,k);
    znew = position(3,k);
    Pnew = [xnew; ynew; znew];
    
    alpha = position(4,k);
    beta = position(5,k);
    gamma = position(6,k);

    R0_ee=[    cosd(beta)*cosd(gamma)                                 ,       -sind(beta)      , cosd(beta)*sind(gamma);...
       cosd(alpha)*sind(beta)*cosd(gamma)+sind(alpha)*sind(gamma) , cosd(alpha)*cosd(beta) , cosd(alpha)*sind(beta)*sind(gamma)-sind(alpha)*cosd(gamma);...
       sind(alpha)*sind(beta)*cosd(gamma)-cosd(alpha)*sind(gamma) , sind(alpha)*cosd(beta) , sind(alpha)*sind(beta)*sind(gamma)+cosd(alpha)*cosd(gamma)];
    
    %Position of End Effector
    %BASE - no rotation, no translation
    newV{1} = Ridentity*V{1};
    newV{1} = newV{1} + repmat([x0,y0,z0]',[1 length(V{1}(1,:))]); %Find new position of moving platform
    %PLATFORM - rotation and traslation from 0 -> p
    newV{2} = R0_ee*V{2};
    newV{2} = newV{2} + repmat(Pnew,[1 length(V{2}(1,:))]); %Find new position of moving platform   
    %BASE JOINTS - no rotation, no translation
    newV{3} = Ridentity*V{3};
    newV{3} = newV{3} + repmat([x0,y0,z0]',[1 length(V{3}(1,:))]); %Find new position of moving platform
    %PLATFORM JOINTS - rotation and traslation from 0 -> p
    newV{4} = R0_ee*V{4};
    newV{4} = newV{4} + repmat(Pnew,[1 length(V{4}(1,:))]); %Find new position of moving platform

    %LOWER JOINTS 1 - rotate from 0 -> ee and translate from 0 ->b1
    newV{5} = T0eeLink1(1:3,1:3)*V{5};
    newV{5} = newV{5} + repmat(Pb0_b{1},[1 length(V{5}(1,:))]); %Find new position of moving platform
    %LOWER JOINTS 2
    newV{6} = T0eeLink2(1:3,1:3)*V{6};
    newV{6} = newV{6} + repmat(Pb0_b{2},[1 length(V{6}(1,:))]); %Find new position of moving platform
    %LOWER JOINTS 3
    newV{7} = T0eeLink3(1:3,1:3)*V{7};
    newV{7} = newV{7} + repmat(Pb0_b{3},[1 length(V{7}(1,:))]); %Find new position of moving platform
    %LOWER JOINTS 4
    newV{8} = T0eeLink4(1:3,1:3)*V{8};
    newV{8} = newV{8} + repmat(Pb0_b{4},[1 length(V{8}(1,:))]); %Find new position of moving platform
    %LOWER JOINTS 5
    newV{9} = T0eeLink5(1:3,1:3)*V{9};
    newV{9} = newV{9} + repmat(Pb0_b{5},[1 length(V{9}(1,:))]); %Find new position of moving platform
    %LOWER JOINTS 6
    newV{10} = T0eeLink6(1:3,1:3)*V{10};
    newV{10} = newV{10} + repmat(Pb0_b{6},[1 length(V{10}(1,:))]); %Find new position of moving platform

    %UPPER JOINTS 1
    newV{11} = T0eeLink1(1:3,1:3)*V{11};
    newV{11} = newV{11} + repmat(T0eeLink1(1:3,4),[1 length(V{11}(1,:))]); %Find new position of moving platform
    %UPPER JOINTS 2
    newV{12} = T0eeLink2(1:3,1:3)*V{12};
    newV{12} = newV{12} + repmat(T0eeLink2(1:3,4),[1 length(V{12}(1,:))]); %Find new position of moving platform
    %UPPER JOINTS 3
    newV{13} = T0eeLink3(1:3,1:3)*V{13};
    newV{13} = newV{13} + repmat(T0eeLink3(1:3,4),[1 length(V{13}(1,:))]); %Find new position of moving platform
    %UPPER JOINTS 4
    newV{14} = T0eeLink4(1:3,1:3)*V{14};
    newV{14} = newV{14} + repmat(T0eeLink4(1:3,4),[1 length(V{14}(1,:))]); %Find new position of moving platform
    %UPPER JOINTS 5
    newV{15} = T0eeLink5(1:3,1:3)*V{15};
    newV{15} = newV{15} + repmat(T0eeLink5(1:3,4),[1 length(V{15}(1,:))]); %Find new position of moving platform
    %UPPER JOINTS 6
    newV{16} = T0eeLink6(1:3,1:3)*V{16};
    newV{16} = newV{16} + repmat(T0eeLink6(1:3,4),[1 length(V{16}(1,:))]); %Find new position of moving platform

    for ii=[1:16] %(CHANGE n=2 to the number of parts that you have)
        set(q(ii),'Vertices',newV{ii}(1:3,:)'); %Set the new position in the handle (graphical link)
    end
    
    
    
    drawnow 
    im= frame2im(getframe);
    gifim(:,:,:,k) = rgb2ind(im, map);
    mov(k)=getframe(gcf);
end


%ANIMATION, creates animated gif (DO NOT MODIFY)
imwrite(gifim,map,'Project_new_code.gif','DelayTime',0) %,'LoopCount',inf)
