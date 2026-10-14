function Nquadrotor_parametros(n,scale,color1,color2)

% QUADROTOR_PARAMETERS 

if nargin < 1
    n = 1;
    scale = 1;
    color1 = [0.9 0 0];
    color2 = [0 0 0.9];
end 

if nargin < 2
    scale = 1;
    color1 = [0.9 0 0];
    color2 = [0 0 0.9];
end 

if nargin < 3
    color1 = [0.9 0 0];
    color2 = [0 0 0.9];
end 

if nargin < 4
    color2 = [0 0 0.9];
end 

global Quadrotor

%% Phisical Parameters
d = 0.315; %Hub displacement from COG
r = 0.165; %Rotor radius

N=1;S=2;E=3;W=4;
D(:,N) = [d;0;0]; %displacements
D(:,S) = [-d;0;0];
D(:,E) = [0;d;0];
D(:,W) = [0;-d;0];
%CALCULATE FLYER TIP POSITONS USING COORDINATE FRAME ROTATION
% F = [1 0 0;0 -1 0;0 0 -1];

%Draw flyer rotors
N_R = 12;
Thick = r; % Thickness
t = linspace(0,2*pi,N_R+1);
% Main Rotor
xt = Thick*cos(t);
yt = Thick*sin(t);
% zt = ones(size(t))*0.08;
zt = zeros(size(t));

for i=1:4
    xtt=xt*0.25;ytt=yt*0.25;ztt=zt*0.25;
    Quadrotor(n).Point{i} = [xtt + D(1,i); ytt + D(2,i); ztt + D(3,i)]*scale;
    if i==1
        Quadrotor(n).PointColor(i,:) = color1;
    else
        Quadrotor(n).PointColor(i,:) = color2;
    end

    Quadrotor(n).Hub{i} = D(:,i)*scale;
    Quadrotor(n).Rotor{i} = [xt+ D(1,i); yt+ D(2,i); zt+ D(3,i)]*scale;
    Quadrotor(n).RotorColor(i,:) = color2;
end
%     %Draw flyer
Quadrotor(n).Frame{1}= [Quadrotor(n).Hub{1}(1,1) Quadrotor(n).Hub{2}(1,1);Quadrotor(n).Hub{1}(2,1) Quadrotor(n).Hub{2}(2,1);Quadrotor(n).Hub{1}(3,1) Quadrotor(n).Hub{2}(3,1)];
Quadrotor(n).FrameColor(1,:) = color2;
Quadrotor(n).Frame{2}= [Quadrotor(n).Hub{3}(1,1) Quadrotor(n).Hub{4}(1,1);Quadrotor(n).Hub{3}(2,1) Quadrotor(n).Hub{4}(2,1);Quadrotor(n).Hub{3}(3,1) Quadrotor(n).Hub{4}(3,1)];
Quadrotor(n).FrameColor(2,:) = color2; 