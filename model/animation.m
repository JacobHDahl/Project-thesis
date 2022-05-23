classdef animation < handle
    %ANIMATION 
    %Animates a crude 2D aeroplane 
    
    properties
        body_handle
    	Vertices
    	Faces
    	facecolors
        plot_initialized
        %axis
        ylim_min
        ylim_max
        xlim_min
        xlim_max
        l %Length of UAV
        h %height of uav
        s %nose extrusion
        t %tail height
        wh %bottom of wing in z
        w_start %bottom of wing in x
        w_end %end point of wing in x
        w_height %top point of wing
    end
    
    methods

        %constructor
        function self = animation
            self.l = 3; %Length of UAV
            self.h = 1.5; %height of uav
            self.s = 1; %nose extrusion
            self.t = 1; %tail height
            self.wh = self.h/3; %bottom of wing in z
            self.w_start = self.l/3; %bottom of wing in x
            self.w_end = self.l*0.75; %end point of wing in x
            self.w_height = self.h*0.6; %top point of wing

            self.ylim_max = 10;
            self.xlim_max = 10;
            self.ylim_min = -10;
            self.xlim_min = -10;
            self.body_handle = [];
            [self.Vertices, self.Faces, self.facecolors] = self.define_spacecraft();
            self.plot_initialized = 0;           
        end
        function self=update(self, state)
            if self.plot_initialized==0
                figure(1); clf;
                pe = state(1); pd = state(2); theta = state(3);
                self=self.drawBody(pe, pd, theta);
                title('UAV')
                xlabel('East')
                ylabel('Down')  % set the vieew angle for figure
                axis([self.xlim_min,self.xlim_max,self.ylim_min,self.ylim_max]);
                hold on
                grid on
                self.plot_initialized = 1;
            else
                pe = state(1); pd = state(2); theta = state(3);
                self = self.followObject(pe,pd);
                self=self.drawBody(pe, pd, theta);
                

            end
        end

        function self = drawBody(self, pe, pd, theta)
            Vertices = self.rotate(self.Vertices,theta); 
            Vertices = self.translate(Vertices, pe, pd);
              % rotate rigid body  
                 % translate after rotation
            
            if isempty(self.body_handle)
                self.body_handle = patch('Vertices', Vertices', 'Faces', self.Faces,...
                                             'FaceVertexCData',self.facecolors,...
                                             'FaceColor','flat');
            else
                set(self.body_handle,'Vertices',Vertices','Faces',self.Faces);
                drawnow
                
            end
        end 
        
        function pts=rotate(self, pts, theta)
            % define rotation matrix (right handed)
            theta = -theta;
            R_pitch = [cos(theta), -sin(theta);
                    sin(theta), cos(theta)];
            R = R_pitch;  % inertial to body
            R = R';  % body to inertial
            pts = R*pts;
        end
        function self=followObject(self,pe,pd)
            axis([self.xlim_min+pe,self.xlim_max+pe,self.ylim_min+pd,self.ylim_max+pd]);

            
        end
        %---------------------------
        % translate vertices by pn, pe, pd
        function pts = translate(self, pts, pe, pd)
            pts = pts + repmat([pe;pd],1,size(pts,2));
        end
        %---------------------------
        function [V, F, colors] = define_spacecraft(self)
            % Define the vertices (physical location of vertices)
            V = [...
                self.l,0;...     %point 1
                self.l+self.s,self.h/2;... %point 2 nose tip
                self.l,self.h;...     %point 3
                0,self.h;...     %point 4
                0,0;...     %point 5 origin
                0,self.h+self.t;...   %point 6 tail top
                self.l/5,self.h;...   %point 7
                self.w_start, self.wh;... %point 8
                self.w_end, self.wh;... %point 9
                self.w_end*0.9, self.w_height*0.9;... %point 10
                self.w_end*0.8, self.w_height;... %point 11r
                ]';

            % define faces as a list of vertices numbered above
            F = [...
                1, 2,  3;...  % nose
                5, 1,  3;...  % lower body
                5, 3,  4;...  % upper body
                4, 7,  6;...  % tail
                8, 9, 10;...  % lower wing
                8, 10, 11;... % upper wing

                ];

            % define colors for each face
            myred = [1, 0, 0];
            mygreen = [0, 1, 0];
            myblue = [0, 0, 1];
            myyellow = [1, 1, 0];
            mycyan = [0, 1, 1];

            colors = [...
                myblue;... % front
                myblue;...   % back
                myblue;...  % bottom
                mygreen;... %tail
                myred; ... %bottom wing
                myred;... %top wing
                ];
        end
    end
end

