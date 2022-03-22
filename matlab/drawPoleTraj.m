function drawCartPoleTraj(t,p1,nFrame)
% drawCartPoleTraj(t,p1,p2,nFrame)
%
% INPUTS:
%   t =  [1,n] = time stamp for the data in p1 and p2
%   p1 = [2,n] = [x;y] = position of the tip of the pendulum
%   nFrame = scalar integer = number of "freeze" frames to display
%

clf; hold on;

Pole_Width = 4;  %pixels

%%%% Get color map for the figure
map = colormap;
tMap = linspace(t(1),t(end),size(map,1))';

%%%% Draw the trace of the pendulum tip  (continuously vary color)

nTime = length(t);
for i=1:(nTime-1)
    idx = i:(i+1);
    x = p1(1,idx);
    y = p1(2,idx);
    c = interp1(tMap,map,mean(t(idx)));
    plot(x,y,'Color',c);
end

%%%% Compute the frames for plotting:
tFrame = linspace(t(1), t(end), nFrame);
pole = interp1(t',p1',tFrame')';

for i = 1:nFrame
    
    % Compute color:
    color = interp1(tMap,map,tFrame(i));
    
    %Plot Pendulum
    Rod_X = [0, pole(1,i)];
    Rod_Y = [0, pole(2,i)];
    plot(Rod_X,Rod_Y,'k-','LineWidth',Pole_Width,'Color',color)
    
    %Plot Bob and hinge
    plot(pole(1,i),pole(2,i),'k.','MarkerSize',40,'Color',color)
end

%These commands keep the window from automatically rescaling in funny ways.
axis('equal');
axis manual;
axis off;

end



function [xLow, xUpp, yLow, yUpp] = getBounds(p1,p2)
%
% Returns the upper and lower bound on the data in val
%

val = [p1,p2];
xLow = min(val(1,:));
xUpp = max(val(1,:));
yLow = min(val(2,:));
yUpp = max(val(2,:));

end