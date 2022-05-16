classdef SmartAgent < Agent
    properties
        targetReached
    end
    
    properties (Access = private)
        targetPos
        xgrad
        ygrad
    end
    
    methods
        
        function self = SmartAgent(x ,y ,h,satLevel,id)
            self = self@Agent(x,y,h,0,0,id);
            self.satLevel=satLevel;
            self.targetReached=0;
            self.sensor='';
        end
        
        function setTarget(self,target)
            self.targetPos=target;
            self.targetReached=0;
            self.targetPos
        end
        
        function setPFMap(self,pfmap)
            
            a_scale=1/500;
            
            mypfmap=pfmap.pfmap;
            
            
            mypfmap=(mypfmap-min(min(mypfmap)))/(max(max(mypfmap))-min(min(mypfmap)));
            mapSize=size(mypfmap);
            
            
            
            [x,y]=meshgrid(1:mapSize(2),1:mapSize(1));
            
            attractive_potential=a_scale*((x-self.targetPos(1)).^2+(y-self.targetPos(2)).^2);
            
            
            attractive_potential=(attractive_potential-min(min(attractive_potential)))/(max(max(attractive_potential))-min(min(attractive_potential)));
            mypfmap=mypfmap+attractive_potential;
            [self.xgrad,self.ygrad]=gradient(-mypfmap);
  
            
%             figure;
%             s=size(mypfmap);
%             [xx,yy] = meshgrid(1:1:s(2),1:1:s(1));
%             quiver(xx,yy,self.xgrad,self.ygrad)
%             figure;
%             surface(xx,yy,attractive_potential);
        end
        
        function a=alphafun(self,s,d,mu)
            a=100;
            if s>d && s<mu
                a=(s-d)^(-1)-(mu-d)^(-1);
            elseif  s>=mu
                a=0;
            end
            if a>100
                a=100;
            end
        end
        function action=smartAction(self,pfmap) 
            maph=size(pfmap.pfmap,1);
            mapw=size(pfmap.pfmap,2);
            xi=ceil(self.x);
            yi=ceil(self.y);
            if xi<1 
                xi=1; 
            elseif xi>mapw
                xi=mapw;
            end
            if yi<1
                yi=1;
            elseif yi>maph
                yi=maph;
            end
            
            pf_v=[pfmap.xgrad(yi,xi);pfmap.ygrad(yi,xi)];
            pf_v=pf_v/norm(pf_v);
            d=pfmap.dismap(yi,xi);
            
            alertb=max(self.w,self.ht)/2;
            a=self.alphafun(d,alertb,alertb*1.5);
            pf_v=pf_v*a;
            
           % pf_v=pf_v/norm(pf_v);
            
            target_v=[self.targetPos(1)-self.x;self.targetPos(2)-self.y];
            target_v=target_v/norm(target_v);
            
            scale=0.5;
            sum_v=pf_v+scale*target_v;
            projVec=utils('projVec');
            projOrthVec=utils('projOrthVec');
            rotMat2=utils('rotMat2');
            heading=[cos(self.h);sin(self.h)];
            a=projVec(sum_v,heading);
            b=projOrthVec(sum_v,heading);
            
            if(a'*heading>=0)
                asgn=1;
            else
                asgn=-1;
            end
            
            rot_heading=rotMat2(pi/2)*heading;
            if(b'*rot_heading>=0)
                bsgn=1;
            else
                bsgn=-1;
            end
            action=[asgn*norm(a) bsgn*norm(b)];
            action=double(action)
        end
        
        function action=smartAction1(self)
            maph=size(self.xgrad,1);
            mapw=size(self.xgrad,2);
            xi=ceil(self.x);
            yi=ceil(self.y);
            if xi<1 
                xi=1; 
            elseif xi>mapw
                xi=mapw;
            end
            if yi<1
                yi=1;
            elseif yi>maph
                yi=maph;
            end
            sum_v=[self.xgrad(yi,xi);self.ygrad(yi,xi)];
            sum_v=sum_v/norm(sum_v);
            projVec=utils('projVec');
            projOrthVec=utils('projOrthVec');
            rotMat2=utils('rotMat2');
            heading=[cos(self.h);sin(self.h)];
            a=projVec(sum_v,heading);
            b=projOrthVec(sum_v,heading);
            
            if(a'*heading>=0)
                asgn=1;
            else
                asgn=-1;
            end
            
            rot_heading=rotMat2(pi/2)*heading;
            if(b'*rot_heading>=0)
                bsgn=1;
            else
                bsgn=-1;
            end
            action=[asgn*norm(a) bsgn*norm(b)];
            action=double(action)
        end
        
        function step(self,tspan,pfmap)
            %map=self.pfmap;
            self.lastState=[self.x,self.y,self.h];
            
            disp(self.targetPos);
            
            %action=self.smartAction(pfmap);
            action=self.smartAction(pfmap);
            sat=utils('sat');
            self.action=sat(action,self.satLevel);
            [t, state] = ode23(@(t, state)self.dynamics(t, state), tspan, self.lastState);
            self.x=state(end,1);
            self.y=state(end,2);
            self.h=state(end,3);
            
            if(abs(self.x-self.targetPos(1))<2 && abs(self.y-self.targetPos(2))<2)
                self.targetReached=1;
            end
            self.updateOccupyMap();
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
        
    
        
    end
    
end