function cf=progress(cf,elapsed)
%  fighandle=progress(fighandle,elapsed)
%
%     Displays the completed fraction of a simulation.
%
%     fighandle=progress;
%       initializes the figure window and returns a handle.
%
%     progress(fighandle,elapsed)
%       updates the plot (0<=elapsed<=100)
%
%     progress(fighandle,100)
%       removes the plot
%
% Magnus Norgaard, Department of Automation, Technical University of Denmark
% LastEditDate: June 5, 1997.

% ----- Initialize figure window -----
if nargin==0,
  cf=figure('Units','Centimeters','Position',[1.5 1.5 10 1.5]);
  ca=gca;
  set(gca,'Units','Normalized');
  set(gca,'Position',[0.05 0.35 0.9 0.5])
  axis([0 100 0 1]);
  set(cf,'Numbertitle','off','Name','% of simulation completed');
  set(ca,'box','on','YTickLabel',[]);
  
elseif nargin==2,
% ----- First time it is called with a % -----
  if isempty(get(gca,'Children')),
    patch([0 0 elapsed elapsed]',[0 1 1 0]','r')
    set(get(gca,'Children'),'EraseMode','None','Edgecolor','r');
    
% ----- When it is called again -----
  else
    set(get(gca,'Children'),'EraseMode','None','Edgecolor','r');
    ax2=get(gca,'Children');
    xdat=get(ax2,'XData');
    set(ax2,'XData',[xdat(3) xdat(3) elapsed elapsed])
  end
  drawnow
  if elapsed>=100, pause(1),close(cf); end
end  
