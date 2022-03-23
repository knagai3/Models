
function skyplot(SVinfo)
% Function to plot satelite positions from azimuth and elevation.
%
% Input:
% SVinfo - [prn az el] degree in rad [n x 3]
%
% Output:
% Skyplot
%
% By Kana Nagai


prn = SVinfo(:,1);
azimuth = SVinfo(:,2);
elevation = SVinfo(:,3);

clf;
axis([-1 1.5 -1.1 1.5]);
axis('off'); axis equal; hold on;

% plot titles
set(gca,'DefaultTextFontSize',12)
text(0,1.6,'Athmith-Elevation Sky Plot','HorizontalAlignment','center');

% plot circular axis and labels
th = pi/2:pi/50:pi*5/2;
x = [ cos(th) .67.*cos(th) .33.*cos(th) ];
y = [ sin(th) .67.*sin(th) .33.*sin(th) ];
p = plot(x,y);
p(1).LineWidth = 1;
text(0,1.2,'North', 'HorizontalAlignment','center');
text(1.2,0,'East');
text(1.1,0,'90','horizontalalignment','center');
text(0,1.1,'0','horizontalalignment','center');
text(-1.1,0,'270','horizontalalignment','center');
text(0,-1.1,'180','horizontalalignment','center'); 

th = (1:6)*2*pi/12;
x = [ -cos(th); cos(th) ];
y = [ -sin(th); sin(th) ];
plot(x,y,'color','b');
text(-.46,.93,'0','horizontalalignment','center');
text(-.30,.66,'30','horizontalalignment','center');
text(-.13,.36,'60','horizontalalignment','center');
text(.04,.07,'90','horizontalalignment','center');

% plot SVs
for i = 1:size(prn)
el = elevation(i);
az = azimuth(i);    
el=pi/2-el;
x1 = 2/pi*el.*cos(az); % ENU(xyz)
y1 = 2/pi*el.*sin(az); % ENU(xyz)
x = y1; % Due to NED
y = x1; % Due to NED 
p = plot(x,y,'r.');
p(1).MarkerSize = 30;
text(x, y+0.15, int2str(prn(i)),'horizontalalignment', ...
          'center','color','r');
end
  









