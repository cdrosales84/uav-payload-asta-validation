function quad = Nquadrotor(n,u)

global Quadrotor;

%PREPROCESS ROTATION MATRIX
phi = u(4);    %Euler angles
the = u(5);
psi = u(6);

% Rot = [cos(the)*cos(phi) sin(psi)*sin(the)*cos(phi)-cos(psi)*sin(phi) cos(psi)*sin(the)*cos(phi)+sin(psi)*sin(phi);   %BBF > Inertial rotation matrix
%     cos(the)*sin(phi) sin(psi)*sin(the)*sin(phi)+cos(psi)*cos(phi) cos(psi)*sin(the)*sin(phi)-sin(psi)*cos(phi);
%     -sin(the)         sin(psi)*cos(the)                            cos(psi)*cos(the)];

%Manual Construction
Q3 = [cos(psi) -sin(psi) 0;sin(psi) cos(psi) 0;0 0 1];   %Rotation mappings
Q2 = [cos(the) 0 sin(the);0 1 0;-sin(the) 0 cos(the)];
Q1 = [1 0 0;0 cos(phi) -sin(phi);0 sin(phi) cos(phi)];
Rot = Q3*Q2*Q1;    %Rotation matrix

quad = gobjects(length(Quadrotor(n).Rotor),1);
% Rotors
for i = 1:length(Quadrotor(n).Rotor)
    rQuadrotor = Rot * Quadrotor(n).Rotor{i};
    hold on;
    quad(i) = plot3(rQuadrotor(1,:)+u(1),rQuadrotor(2,:)+u(2),rQuadrotor(3,:)+u(3),'Color',Quadrotor(n).RotorColor(i,:));
end

tam = i;

%Four Points
for i = 1:length(Quadrotor(n).Point)
    rQuadrotor = Rot * Quadrotor(n).Point{i};
    quad(tam+i) = patch(rQuadrotor(1,:)+u(1),rQuadrotor(2,:)+u(2),rQuadrotor(3,:)+u(3),Quadrotor(n).PointColor(i,:));
end

tam = tam + i;

%Frame
rQuadrotor = Rot * Quadrotor(n).Frame{1};
quad(tam+1) = plot3(rQuadrotor(1,:)+u(1),rQuadrotor(2,:)+u(2),rQuadrotor(3,:)+u(3),'Color',Quadrotor(n).FrameColor(1,:));
rQuadrotor = Rot * Quadrotor(n).Frame{2};
quad(tam+2) = plot3(rQuadrotor(1,:)+u(1),rQuadrotor(2,:)+u(2),rQuadrotor(3,:)+u(3),'Color',Quadrotor(n).FrameColor(2,:));
