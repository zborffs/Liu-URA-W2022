function drawPoleAnim(~,p, xLow, xUpp, yLow, yUpp)
% drawCartPoleTraj(t,p,xLow, xUpp, yLow, yUpp)
%
% INPUTS:
%   t =  [1,n] = time stamp for the data in p1 and p2
%   p =  [4,n] = [p1];
%

clf; hold on;

p1 = p;


Pole_Width = 4;  %pixels
Limits = [xLow-0.1,xUpp+0.1,yLow-0.1,yUpp+0.1];

%%%% Figure out the window size:

%%%% Get color map for the figure
% map = colormap;
% tMap = linspace(t(1),t(end),size(map,1))';

%%%% Draw the trace of the pendulum tip  (continuously vary color)
% nTime = length(t);
% for i=1:(nTime-1)
%     idx = i:(i+1);
%     x = p2(1,idx);
%     y = p2(2,idx);
%     c = interp1(tMap,map,mean(t(idx)));
%     plot(x,y,'Color',c);
% end

%%%% Compute the frames for plotting:
% tFrame = linspace(t(1), t(end), nFrame);
% cart = interp1(t',p1',tFrame')';
% pole = interp1(t',p2',tFrame')';

pole = p1;

% for i = 1:nFrame
    
    % Compute color:
    color = [0.2,0.7,0.1];  %interp1(tMap,map,tFrame(i));
    
    %Plot Pendulum
    Rod_X = [0, pole(1)];
    Rod_Y = [0, pole(2)];
    plot(Rod_X,Rod_Y,'k-','LineWidth',Pole_Width,'Color',color)
    
    %Plot Bob and hinge
    plot(pole(1),pole(2),'k.','MarkerSize',40,'Color',color)
    
% end

%These commands keep the window from automatically rescaling in funny ways.
axis(Limits);
axis('equal');
axis manual;
axis off;

end
