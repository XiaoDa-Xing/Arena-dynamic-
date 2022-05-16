classdef Viewer < handle
    properties
        h % heading
        w % width
    end
    
   properties (Access =  {?Evn})
        fh% figure handle
    end
    properties (SetAccess = private)
        ax
    end
    
    
    methods
    
        
        function self = Viewer(w,h)
            self.h=h;
            self.w=w;
            self.fh=figure('Name','Game Arena','NumberTitle','off');
            %fh.Color=[0.4 0.6 0.7];
            self.fh.MenuBar='none';
            
            
            screenSize=get (0, 'ScreenSize');
            screenw=screenSize(3);
            screenh=screenSize(4);
            
            posw=screenw*1.5/4;
            posh=screenh*1.5/4;
            
            poswh=max(posw,posh);
            
            posx=screenw-poswh;
            posy=screenh-poswh;
            
            self.fh.Position =[posx posy poswh poswh];
            self.ax=axes('Position',[0.05 0.05 0.9 0.9],'Box','on');
            self.reInitAxe(0);   
        end
        
        function h=getHandle(self)
            h= self.fh;
        end
        
        function show(self,bshow)
            if bshow
                set(self.fh, 'Visible', 'on');
            else
                set(self.fh, 'Visible', 'off');
            end
        end
       
        function title(self,text)
            self.ax.Title.String=text;
        end
        function reInitAxe(self,t)
            %refresh (self.fh);
            %self.ax
            self.ax.YTick = 0:10:self.h;
            self.ax.XTick = 0:10:self.w;
            self.ax.FontSize = 12;
            self.ax.XLimMode='manual';
            self.ax.YLimMode='manual';
            self.ax.XLim=[0 self.w];
            self.ax.YLim=[0 self.h];       
            hold on;
        end
        
        
         
    end
    
end