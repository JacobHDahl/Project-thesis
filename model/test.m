x = [-1  , 1/3, 1/3, 1, 1/3, 1/3,-1  ];
y = [-1/3,-1/3,-1/2, 0, 1/2, 1/3, 1/3];
g = hgtransform;
patch('XData',x,'YData',y,'FaceColor','yellow','Parent',g)

axis equal
xlim([-10 10])
ylim([-10 10])

pt1 = [-3 -4 0];
pt2 = [5 2 0];
for t=linspace(0,1,100)
  g.Matrix = makehgtform('translate',pt1 + t*(pt2-pt1));
  drawnow
end

s1 = 1/2;
s2 = 2;
r1 = 0;
r2 = 2*pi/3;
for t=linspace(0,1,100)
  g.Matrix = makehgtform('translate',pt1 + t*(pt2-pt1), ...
                         'scale',s1 + t*(s2-s1), ...
                         'zrotate',r1 + t*(r2-r1));
  drawnow
  pause(0.1)
end