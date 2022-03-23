clc
clear

h = figure;
filename = 'Animated.gif';

for theta = 0:pi/6:2*pi
   
    % Draw plot
    plot(0,0,'r>','MarkerSize',15)    
    hold on
    R = [cos(theta) -sin(theta);sin(theta) cos(theta)];
    r = 6.3;
    p0 = [0; r];
    p1 = [r*sind(30); r*cosd(30)];
    p0 = R*p0;
    p1 = R*p1;
    plot(p0(1),p0(2),'g*','MarkerSize',15)
    plot(p1(1),p1(2),'g*','MarkerSize',15)
    axis equal
    grid on
    xlim([-r,r])
    ylim([-r,r])
    drawnow
    
    % Capture the plot as an image 
    frame = getframe(h); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    if theta == 0 
    imwrite(imind,cm,filename,'gif','Loopcount',inf); 
    else 
    imwrite(imind,cm,filename,'gif','WriteMode','append'); 
    end 
    hold off
end