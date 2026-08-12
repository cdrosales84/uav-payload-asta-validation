function [xt,yt,zt,psit,dxt,dyt,dzt,dpsit,ddxt,ddyt,ddzt,ddpsit] = ReferenciasCarga_ST(T,deltat,trayectoria)
% ReferenciasCarga_ST(T,deltat)           -> pide trayectoria por teclado
% ReferenciasCarga_ST(T,deltat,trayectoria) -> usa trayectoria (1-6) sin preguntar (para batch/paper)
if nargin < 3
    disp('Elija Referencia');
    disp(' ');
    trayectoria=input('Ingrese Trayectoria:(circulo/elipse=1; recta=2; espiral=3; Ocho=4; Ocho Inclinado=5 ; Punto=6):');
    fprintf('\n\n');
end
% trayectoria=1;

switch(trayectoria)
    case 1  %%%%%%  CIRCULO/ELIPSE   %%%%%%
        t=0:deltat:T;
        TT = 10;
        a=3; b=3; c=2*pi/TT;
        xt=a*sin(c*t);
        yt=b*cos(c*t);
        zt=6*ones(1,length(xt)) + 1/TT*t; %%circulo cte
%         zt=6*ones(1,length(xt)); %%circulo cte
%         zt=10+sin(c*t);         %% circulo inclinado
        dxt=a*c*cos(c*t) ;
        dyt=-b*c*sin(c*t);
        dzt=1/TT*ones(1,length(xt));    %%circulo cte
%         dzt=c*cos(c*t) ;         %% circulo inclinado
        psit=atan2(dyt,dxt) ;      %% phi del trayectoria
%         psit_aux = unwrap(psit);       %% derivada de psi de la trayectoria
%         dpsit=[0 diff(psit_aux,1,2)]/deltat;
        dpsit= -c*( (a*b*cos(c*t).^2 + a*b*sin(c*t).^2) ./ (a^2*cos(c*t).^2 + b^2*sin(c*t).^2) );
        psit_aux = unwrap(dpsit);
        ddpsit=[0 diff(psit_aux,1,2)]/deltat;
        
        ddxt=-a*c^2*sin(c*t);
        ddyt=-b*c^2*cos(c*t);
        ddzt=0*ones(1,length(xt));    %%circulo cte
%         ddzt=-c^2*sin(c*t);    %%circulo cte
        %         psit_aux = unwrap(dpsit);       %% derivada de psi de la trayectoria
        %         ddpsit=-c^2*cos(c*t);%[0 diff(psit_aux,1,2)]/deltat;
        
         
        
    case 2  %%%%%%  RECTA   %%%%%%
        t=0:deltat:T;
        a=2.5;
        b=2.5;
        xt=a*t;
        yt=5 + b*t;
        zt=6*ones(1,length(xt));
        dxt=a*ones(1,length(xt));
        dyt=b*ones(1,length(xt));
        dzt=0*ones(1,length(xt));
        ddxt=zeros(1,length(xt));
        ddyt=zeros(1,length(xt));
        ddzt=zeros(1,length(xt));
        psit=atan2(dyt,dxt) ;      %% phi del trayectoria
        psit_aux = unwrap(psit);       %% derivada de psi de la trayectoria
        dpsit=[0 diff(psit_aux,1,2)]/deltat;
        psit_aux = unwrap(dpsit);       %% derivada de psi de la trayectoria
        ddpsit=[0 diff(psit_aux,1,2)]/deltat;
        
        xt(1)=0; yt(1)=0; dxt(1)=0; dyt(1)=0; dzt(1)=0;
        ddxt(1)=0; ddyt(1)=0; ddzt(1)=0; psit(1)=0; dpsit(1)=0; ddpist(1)=0;
        
    case 3  %%%%%%  CIRCULO/ELIPSE   %%%%%%
        %vel_camino = 4.0;
        t=0:deltat:T;
        %         theta=0:0.01:10*pi;
        Periodo = 5;
        r=1; c=2*pi/Periodo;
        xt=r*sin(c*t);
        yt=r*cos(c*t);
%         zt=2*ones(1,length(xt)); %%circulo cte
                zt=1+0.3*sin(c*t);         %% circulo inclinado
        dxt=r*c*cos(c*t) ;
        dyt=-r*c*sin(c*t);
%         dzt=0*ones(1,length(xt));    %%circulo cte
                dzt=0.3*c*cos(c*t) ;         %% circulo inclinado
        psit=atan2(dyt,dxt) ;      %% phi del trayectoria
        %         psit_aux = unwrap(psit);       %% derivada de psi de la trayectoria
        %         dpsit=[0 diff(psit_aux,1,2)]/deltat;
        dpsit = -c*ones(1,length(xt));
        ddxt=-r*c^2*sin(c*t);
        ddyt=-r*c^2*cos(c*t);
        ddzt=0*ones(1,length(xt));    %%circulo cte
        ddpsit=zeros(1,length(xt));
        
    case 4  %%%%%%  OCHO  %%%%%%
        
        t=0:deltat:T;
        T=10;
        a=1.5; b=1.5; c=2*pi/T; d=0.5;
        xt=a*sin(c*t).*cos(c*t);
        yt=b*sin(c*t);
        zt=1*ones(1,length(xt));
%         zt=d*t;  %% circulo ascendente
        dxt=a*c*cos(c*t).^2 - a*c*sin(c*t).^2;
        dyt=b*c*cos(c*t);
        dzt=0*ones(1,length(xt));
%         dzt=d*ones(1,length(xt));
        ddxt=-a*c^2*2*sin(c*t) - a*c^2*2*cos(c*t);
        ddyt=-b*c^2*sin(c*t);
        ddzt=zeros(1,length(xt));
        psit=atan2(dyt,dxt) ;      %% phi del trayectoria
        %         psit_aux = unwrap(psit);       %% derivada de psi de la trayectoria
        psit_aux = psit;
        dpsit=[0 diff(psit_aux,1,2)]/deltat;
        psit_aux = unwrap(dpsit);       %% derivada de psi de la trayectoria
        ddpsit=[0 diff(psit_aux,1,2)]/deltat;
        
    case 5  %%%%%%  OCHO INCLINADO   %%%%%%
        t=0:deltat:T;
        a=10; b=7.5; c=0.025*2*pi; d = 1;%d=6+7.5/T;
        xt=a*sin(c*t).*cos(c*t);
        yt=b*sin(c*t);
        %         zt=6*ones(1,length(xt));
        zt=6 + d*sin(c*t);  %% circulo ascendente
        %pt = interparc(length(xt),xt,yt,'spline');
        %xt = pt(:,1);
        %yt = pt(:,2);
        dxt=a*c*cos(c*t).^2 - a*c*sin(c*t).^2;
        dyt=b*c*cos(c*t);
        %         dzt=0*ones(1,length(xt));
        %         dzt=d*ones(1,length(xt));
        dzt=d*c*cos(c*t);
        ddxt=-a*c^2*2*sin(c*t) - a*c^2*2*cos(c*t);
        ddyt=-b*c^2*sin(c*t);
        %         ddzt=zeros(1,length(xt));
        ddzt=-d*c^2*sin(c*t);
        psit=atan2(dyt,dxt) ;      %% phi del trayectoria
        psit_aux = unwrap(psit);       %% derivada de psi de la trayectoria
        dpsit=[0 diff(psit_aux,1,2)]/deltat;
        psit_aux = unwrap(dpsit);       %% derivada de psi de la trayectoria
        ddpsit=[0 diff(psit_aux,1,2)]/deltat;
        
    case 6  %%%%%%  Estabilizacion en un punto   %%%%%%
        t=0:deltat:T;
        
        xt=2*ones(1,length(t));
        yt=2*ones(1,length(t));
        zt=2*ones(1,length(t));
        dxt=zeros(1,length(t));
        dyt=zeros(1,length(t));
        dzt=zeros(1,length(t));
        
        ddxt=zeros(1,length(t));
        ddyt=zeros(1,length(t));
        ddzt=zeros(1,length(xt));
        psit= pi/4*ones(1,length(t));      %% phi del trayectoria
        dpsit=zeros(1,length(t));
        ddpsit = zeros(1,length(t));
        
        xt(1)=0; yt(1)=0; zt(1)=0; dxt(1)=0; dyt(1)=0; dzt(1)=0;
        ddxt(1)=0; ddyt(1)=0; ddzt(1)=0; psit(1)=0; dpsit(1)=0; ddpist(1)=0;
end %%% end switch case