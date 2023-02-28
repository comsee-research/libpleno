function [loop,X1,Y1,Z1] = gradient(auto, Xinit, Yinit)
    % Algorithm constants
    loop    = 0;
    maxloop = 10000;
    pas     = sqrt(2^-52);
    eps     = 10^-12;

    % Rosenbrock function calculations
    x = -1.5:0.01:1.5;
    y = -1.5:0.01:1.5;
    z = zeros(length(y),length(x));

    for i=1:length(x)
       for j=1:length(y)
           z(j,i)=(1-x(i))^2+100*(y(j)-x(i)^2)^2;
       end
    end

    % Rosenbrock function display
    t  = 0.1:0.2:3;
    t2 = 10.^t;

    figure(1);
    hold('on'); contour(x,y,z,t2);
    axis([-1.5,1.5,-1.5,1.5],'square');

    % Initialization
    %[X,Y] = ginput(1);
    X = Xinit; Y = Yinit;
    plot(X,Y,'+r'); hold('off');
    Xmin = zeros(1, maxloop); Xmin(loop+1) = X;
    Ymin = zeros(1, maxloop); Ymin(loop+1) = Y;

    Z1 = (1-X)^2+100*(Y-X^2)^2;
    Z  = 2*Z1;

    while abs(Z1-Z)>eps && loop < maxloop
       % Z update
       Z=Z1;

       % DZX calculation
       if abs(X)<pas
           h=pas;
       else
           h=pas*abs(X);
       end

       X1=X+h; X0=X-h;
       ZX1=(1-X1)^2+100*(Y-X1^2)^2;
       ZX0=(1-X0)^2+100*(Y-X0^2)^2;
       DZX=(ZX1-ZX0)/(2*h);

       % DZY calculation
       if abs(Y)<pas
           h=pas;
       else
           h=pas*abs(Y);
       end

       Y1=Y+h; Y0=Y-h;
       ZY1=(1-X)^2+100*(Y1-X^2)^2;
       ZY0=(1-X)^2+100*(Y0-X^2)^2;
       DZY=(ZY1-ZY0)/(2*h);

       % Find a new position
       lambda=1;
       X1=X-DZX*lambda;
       Y1=Y-DZY*lambda;
       Z1=(1-X1)^2+100*(Y1-X1^2)^2;

       if Z1<Z
           X2=X1; Y2=Y1; Z2=Z1;
           X1=X;  Y1=Y;  Z1=Z;
           while Z2<Z1
               X1=X2; Y1=Y2; Z1=Z2;

               lambda=lambda*2;
               X2=X-DZX*lambda;
               Y2=Y-DZY*lambda;
               Z2=(1-X2)^2+100*(Y2-X2^2)^2;
           end
       else
           while Z1>Z
               lambda=lambda/2;
               X1=X-DZX*lambda;
               Y1=Y-DZY*lambda;
               Z1=(1-X1)^2+100*(Y1-X1^2)^2;
           end
       end

       if ~strcmp(auto,'auto')
           fprintf(1, '      val = %f\n', Z);

           figure(1); hold('on');
           contour(x,y,z,t2); axis('square');
           plot(Xmin(1:loop+1),Ymin(1:loop+1),'.r');
           plot(Xmin(1:loop+1),Ymin(1:loop+1),'k');
           hold('off');
           pause;
       end

       loop = loop + 1;
       Xmin(loop+1) = X1; X=X1;
       Ymin(loop+1) = Y1; Y=Y1;
    end

    % Final plot
    hold('on');
    plot(Xmin(1:loop),Ymin(1:loop),'.r');
    plot(Xmin(1:loop),Ymin(1:loop),'k');
    hold('off');

    fprintf(1, '      loop = %d\n', loop);
    fprintf(1, '      [X,Y,Z] = [%f, %f, %f]\n', X1, Y1, Z1);
    pause; clear all; close all;
