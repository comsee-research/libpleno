function [loop,Zf,Xf,Yf] = simplex(auto, Xinit, Yinit)
    % Algorithm constants
    loop = 0;
    maxloop = 1000;
    eps = 10^-8;

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
    hold('off');
    newplot; axis([-1.5,1.5,-1.5,1.5],'square');
    hold('on');
    t  = 0.1:0.2:3;
    t2 = 10.^t;
    contour(x,y,z,t2);

    % Graphical initialization
    X = zeros(1,3); Y = zeros(1,3);

    for k=1:3
     %[X(k),Y(k)] = ginput(1);
     X(k) = Xinit(k); Y(k) = Yinit(k);
     plot(X(k),Y(k),'+r');
    end

    plot([X,X(1)],[Y,Y(1)],'r');
    Xmin = zeros(1, maxloop); Xmin(loop+1) = X(1);
    Ymin = zeros(1, maxloop); Ymin(loop+1) = Y(1);

    Z = (1-X).^2+100*(Y-X.^2).^2;
    [Z,I] = sort(Z);
    X = X(I); Y = Y(I);

    while abs(Z(3)-Z(1))>eps

       Xm=(X(1)+X(2))/2;
       Ym=(Y(1)+Y(2))/2;
       X4=X(3)+2*(Xm-X(3));
       Y4=Y(3)+2*(Ym-Y(3));
       Z4=(1-X4)^2+100*(Y4-X4^2)^2;

       if ~strcmp(auto,'auto')
           fprintf(1, '      Val = [%f, %f, %f]\n', Z(1),X(1),Y(1));
           plot(X4,Y4,'+r');
           pause;
       end

       if Z4<Z(3)
           X(3)=X4; Y(3)=Y4; %Z(3)=Z4;
       else
           X4=X(3)+3*(Xm-X(3));
           Y4=Y(3)+3*(Ym-Y(3));
           Z4=(1-X4)^2+100*(Y4-X4^2)^2;

           if ~strcmp(auto,'auto')
               plot(X4,Y4,'+g');
               pause;
           end

           if Z4<Z(3)
               X(3)=X4; Y(3)=Y4; %Z(3)=Z4;
           else
               X4=X(3)+(Xm-X(3))/2;
               Y4=Y(3)+(Ym-Y(3))/2;
               Z4=(1-X4)^2+100*(Y4-X4^2)^2;

               if ~strcmp(auto,'auto')
                   plot(X4,Y4,'+b');
                   pause;
               end

               if Z4<Z(3)
                   X(3)=X4; Y(3)=Y4; %Z(3)=Z4;
               else
                   X(2)=X(1)+4/5*(X(2)-X(1));
                   Y(2)=Y(1)+4/5*(Y(2)-Y(1));
                   X(3)=X(1)+4/5*(X(3)-X(1));
                   Y(3)=Y(1)+4/5*(Y(3)-Y(1));

                   if ~strcmp(auto,'auto')
                       plot([X(2),X(3)],[Y(2),Y(3)],'+y');
                       pause;
                   end
               end
           end
       end

       if ~strcmp(auto,'auto')
           hold('off');
           newplot; axis('square');
           contour(x,y,z,t2);
           hold('on');
           plot([X,X(1)],[Y,Y(1)],'.r');
           plot([X,X(1)],[Y,Y(1)],'r');
           plot(Xmin(1:loop),Ymin(1:loop),'k');
       end

       Z=(1-X).^2+100*(Y-X.^2).^2;
       [Z,I]=sort(Z);
       X = X(I);
       Y = Y(I);

       loop = loop + 1;
       Xmin(loop+1) = X(1);
       Ymin(loop+1) = Y(1);
    end

    hold('off');
    newplot;
    axis([-1.5,1.5,-1.5,1.5],'square');
    hold('on');
    contour(x,y,z,t2);

    % Final plot
    plot(Xmin(1:loop),Ymin(1:loop),'.r');
    plot(Xmin(1:loop),Ymin(1:loop),'k');
    fprintf(1, '      loop = %d\n', loop);
    fprintf(1, '      [Z,X,Y] = [%f, %f, %f]\n', Z(1), X(1), Y(1));
    Zf = Z(1); Xf = X(1); Yf = Y(1);
    pause; close;
