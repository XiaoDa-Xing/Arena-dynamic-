classdef Agent < handle
    properties
        x % x coordinate
        y % y coordinate
        h % heading
        satLevel %satuation Level
        id
        
        w % width
        ht %height
        
        
    end
    
    properties (Access = protected)
        occupy_map
        color
        action
        scanBox
        sensor
        lastState% last state;
        
        headOffset
        bHandle;
        lwHandle;
        rwHandle;
        headHandle;
    end
    
    methods
        
        function self = Agent(x ,y ,h,scanRange,scanAngle,id)
            self.x = x;
            self.y = y;
            self.h = h;
            self.size(3,2);
            self.id=id;
            self.headOffset=[self.w*1.5/4;0];
            self.sensor=RangeFinder(0.3,scanRange,scanAngle,self.headOffset);
            self.sensor.updateScan(self.TF());
        end

        function setAgent(self,x ,y ,h)
            self.x = x;
            self.y = y;
            self.h = h;
            if ~isempty(self.sensor)
                self.sensor.updateScan(self.TF());
            end
        end
        
        
        function size(self,w,ht)
            self.w=w;
            self.ht=ht;
        end
        
        function retreat(self)
            self.x =self.lastState(1);
            self.y = self.lastState(2);
            self.h = self.lastState(3);
            if ~isempty(self.sensor)
                self.sensor.updateScan(self.TF());
            end
        end
        
        function step(self,tspan,action)
            self.lastState=[self.x,self.y,self.h];
            sat=utils('sat');
            self.action=sat(action,self.satLevel);
            [t, state] = ode45(@(t, state)self.dynamics(t, state), tspan, self.lastState);
            self.x=state(end,1);
            self.y=state(end,2);
            self.h=state(end,3);
            self.sensor.updateScan(self.TF());
            self.updateOccupyMap();
            %action 1 is for velocity, aciton 2 is for angular velocity
        end
        function tf=TF(self)
            tf= [self.x,self.y,self.h];
        end
        
        function setSatLevel(self,level)
            self.satLevel=level;
        end
        function ds = dynamics(self,t,state)
            self.h=state(3);
            u=self.action(1);
            v=self.action(2);
            dx=u*cos(self.h);
            dy=u*sin(self.h);
            dh=v;
            ds=[dx;dy;dh];
        end
        
        function in=insideScan(self, point)
            in= self.sensor.insideScan(point);
        end
        
        
        function plot(self,handle)
            rotMat2=utils('rotMat2');
            rm=rotMat2(self.h);
            corners=[-self.w/2,-self.ht/2;
                -self.w/2,self.ht/2;
                self.w/2,self.ht/2;
                self.w/2,-self.ht/2;];
            body_corners=[self.x;self.y]+rm*corners';
            if isempty(self.bHandle)
                self.bHandle=fill(handle,body_corners(1,:),body_corners(2,:),'g');
            else
               set(self.bHandle,'XData',body_corners(1,:),'YData',body_corners(2,:));
            end
            
            
            wheel_h=0.2;
            wheel_y=self.ht*0.8/2;
            corners=[-self.w/4,-wheel_y;
                self.w/4,-wheel_y;
                self.w/4,-wheel_y+wheel_h;
                -self.w/4,-wheel_y+wheel_h; ];
            rwheel_corners=[self.x;self.y]+rm*corners';
            if isempty(self.lwHandle)
                self.lwHandle=fill(handle,rwheel_corners(1,:),rwheel_corners(2,:),'b');
            else
               set(self.lwHandle,'XData',rwheel_corners(1,:),'YData',rwheel_corners(2,:));
            end
            
            corners=[-self.w/4,wheel_y;
                self.w/4,wheel_y;
                self.w/4,wheel_y-wheel_h;
                -self.w/4,wheel_y-wheel_h; ];
            lwheel_corners=[self.x;self.y]+rm*corners';
            if isempty(self.rwHandle)
                self.rwHandle=fill(handle,lwheel_corners(1,:),lwheel_corners(2,:),'b');  
            else
               set(self.rwHandle,'XData',lwheel_corners(1,:),'YData',lwheel_corners(2,:));
            end
            
            
             
             
             head=self.headOffset;
             offset=self.ht/8;
             head_courners=[head(1) head(1)-offset  head(1)-offset head(1);
                          head(2) head(2)+offset head(2)-offset head(2)];           
             head_courners=[self.x;self.y]+rm*head_courners;                   
            if isempty(self.headHandle)
                self.headHandle=plot(head_courners(1,:),head_courners(2,:),'k-');
            else
               set(self.headHandle,'XData',head_courners(1,:),'YData',head_courners(2,:));
            end
            
            if ~isempty(self.sensor)
                self.sensor.plot(handle,self.TF());
            end
        end
        
            
        function coords=updateOccupyMap(self) 
            rotMat2=utils('rotMat2');
            rm=rotMat2(self.h);
            
            [X,Y]=meshgrid(-self.w/2:1:self.w/2, ... 
                -self.ht/2:1:self.ht/2);
            X=reshape(X,1,numel(X));
            Y=reshape(Y,1,numel(X));
            coords=[X;Y];
            coords=rm*coords;
            coords=coords+[self.x;self.y];
            coords=ceil(coords);
            coords=unique(coords','rows');
            coords=coords';
           self.occupy_map=coords;
        end
        
        function coords=scanMap(self)
            coords=self.sensor.occupyMap(self.TF());
        end
        
        function coords=occupyMap(self) 
            coords=self.occupy_map;
        end
        
    end
    
end