classdef RangeFinder < handle
    properties
        lidarRadius
        scanRange
        scanAngle
        offset
        scanBox
    end
    properties (SetAccess = private)
        ax
        fh% figure handle
    end
    
    
    methods
        
        function self = RangeFinder(lidarRadius,scanRange,scanAngle,offset)
            self.lidarRadius=lidarRadius;
            self.scanRange=scanRange;
            self.scanAngle=scanAngle;
            self.offset=offset;
        end
        
        
        function plot(self,handle,TF)
            persistent sHandle;
            %h=TF(3);
            %x=TF(1);
            %y=TF(2);
            
            %rotMat2=utils('rotMat2');
            %rm=rotMat2(h);
            
            
            %circleCenter=[x;y]+rm*self.offset;
            
            %viscircles(circleCenter', self.lidarRadius,'Color','r','LineWidth',3);
            
            if isempty(sHandle)
                sHandle=fill(handle,self.scanBox(1,:),self.scanBox(2,:),'r','FaceAlpha',0.5);
            else
               set(sHandle,'XData',self.scanBox(1,:),'YData',self.scanBox(2,:));
            end
            
            
        end
        
        
        function in=insideScan(self, point)
            insidePolygon=utils('insidePolygon');
            in=insidePolygon(self.scanBox,point);
        end
        
        
        function coords=occupyMap(self,TF)
                        
            h=TF(3);
            x=TF(1);
            y=TF(2);
            rotMat2=utils('rotMat2');
            rm=rotMat2(h);
            s=2;
            dt=pi*s/(self.scanRange*2*18); %every 10s degree
            angles= -self.scanAngle:dt:self.scanAngle;
            longitudes=0:1:self.scanRange;
            len=length(angles)*length(longitudes);
            i=1;
            coords=zeros(2,len);
            for l=longitudes
                for a=angles
                    coords(1,i)=l*cos(a);
                    coords(2,i)=l*sin(a);
                    i=i+1;
                end
            end
            coords=rm*coords+self.offset+[x;y];
            
            coords=ceil(coords);
            
            coords=unique(coords','rows');
            coords=coords';

        end
        function updateScan(self,TF)
            
            h=TF(3);
            x=TF(1);
            y=TF(2);
            rotMat2=utils('rotMat2');
            rm=rotMat2(h);
            % cc=[self.w*1.5/4 0];
            
            sb(1,:)=[self.offset(1), self.offset(2)];
            sb(2,:)=[sb(1,1)+self.scanRange*cos(self.scanAngle), sb(1,2)+self.scanRange*sin(self.scanAngle)];
            sb(3,:)=[sb(1,1)+self.scanRange*cos(self.scanAngle), sb(1,2)-self.scanRange*sin(self.scanAngle)];
            self.scanBox=[x;y]+ rm*sb';
        end
    end
end
