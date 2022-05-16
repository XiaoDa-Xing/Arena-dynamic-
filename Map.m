classdef Map < handle
    properties
        w % width
        h % height
        gs %%grid size
        
    end
    
    properties (SetAccess = private)
        id
        obstacles
        xnum
        ynum
        map
        mapDir
    end
    
    methods
        
        function self = Map(w, h)
            self.w = w;
            self.h = h;
            self.gs=1;
            self.map=zeros(h,w);
            self.xnum=floor(self.w/self.gs);
            self.ynum=floor(self.h/self.gs);
            self.mapDir='maps/';
        end
        
        function randomMap(self)
            self.random();
            self.save('ranmap');
            self.load('ranmap');
        end
        
        function mapFromObstacles(self)
            self.map=zeros(self.ynum,self.xnum);
            for i=1:size(self.obstacles,2)
                self.map(self.obstacles(2,i),self.obstacles(1,i))=1;
            end
        end
        
        function pos=generatePos(self)
           while 1
               pos=[rand(1,1)*self.w;rand(1,1)*self.h];
               pos=ceil(pos);
               pos=self.clipCoord(pos);
               if self.map(pos(2),pos(1))~=1
                   break;
               end
           end
        end
        
        function obs=getObstacles(self)
            obs= self.obstacles;
        end
        
        function obmap=occupyMap(self)  % the occupation map for the obstacles
            obmap= self.map;
        end
        
        function random(self)
            len=20;
            coords=[rand(1,len)*self.w;rand(1,len)*self.h];
            
            idx=len+1;
            for i=1:len
                size=rand(2,1)*10; %size=[w,h]
                for j=0:size(1)
                    for k=0:size(2)
                        coords(1,idx)=coords(1,i)+j;
                        coords(2,idx)=coords(2,i)+k;
                        idx=idx+1;
                    end
                end
            end
            coords=ceil(coords);
            coords=self.clipCoord(coords);
            coords=unique(coords','rows');
            coords=coords';
            
            self.obstacles=coords;
            self.mapFromObstacles();
        end
        
        
        function coords=clipCoord(self,coords)
            coords(coords<1)=1;
            coords(1,coords(1,:)>self.w)=self.w;
            coords(2,coords(2,:)>self.h)=self.h;
        end
        
        function load(self,filename)
            
            abspath=utils('abspath');
            filename=strcat(self.mapDir,filename,'.mat');
            load(abspath(filename),'mapdata');
            
            self.map=mapdata;
            [I,J]=find(mapdata==1);
            self.obstacles=[J I]';
        end
        
        function save(self,filename)
            abspath=utils('abspath');
            mapdata=self.map;
            filename=strcat(self.mapDir,filename,'.mat');
            save(abspath(filename),'mapdata');
        end
        
        function plotFog(self,handle)
            persistent fillHandle;
            corners=[0 self.w self.w 0;
                          0 0 ,self.h self.h];
            if isempty(fillHandle)
                fillHandle=fill(handle,corners(1,:),corners(2,:),[0.1 0.1 0.1]);  
                fillHandle.FaceAlpha=0.9;
            else
               set(fillHandle,'XData',corners(1,:),'YData',corners(2,:));
            end
        end
        
        
        function plot(self,handle)
            persistent fillHandle;
            len=size(self.obstacles,2);
            for i=1:len
                x=self.obstacles(1,i)-1;
                y=self.obstacles(2,i)-1;
                corners=[x,y;x+self.gs,y;x+self.gs,y+self.gs;x,y+self.gs];
                if isempty(fillHandle)
                    fillHandle=zeros(len,1);
                end
                if fillHandle(i)==0
                    fillHandle(i)=fill(handle,corners(:,1),corners(:,2),'k');
                else
                    set(fillHandle(i),'XData',corners(:,1),'YData',corners(:,2));
                end
            end
            
            %self.plotFog(handle);
            
        end
    end
    
end