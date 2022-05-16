classdef Policy < handle
    properties % A*算法变量
        nextAx
        path
        lastpath
    end
    
    properties 
        getpathflag %全局下是否搜索到路径 0没有 1有
        getlastpathflag
        pL
    end
    
    properties 
        mapdata%全局地图信息
        dilateMap
        borderMap
        map
        obstacle
        
    end
    
    properties 
        nextborder
    end
    
    properties 
        fis
        intergral
        last_error
        fuzflag
        now_state
        
        kp = 125;
        ki = 0.3;
        kd = 530;
    end

    methods
        %% 构造函数
        function self = Policy()
            self.getpathflag=0;
            
            
            self.nextborder = [-1,1;...
                        0,1;...
                        1,1;...
                        -1,0;...
                        1,0;...
                        -1,-1;...
                        0,-1;...
                        1,-1];
                    
            self.nextAx = [-1,1,14;...
                        0,1,10;...
                        1,1,14;...
                        -1,0,10;...
                        1,0,10;...
                        -1,-1,14;...
                        0,-1,10;...
                        1,-1,14];
                    
            self.fis=readfis('arenafuz.fis');
            self.intergral = 0;
            self.last_error = 0;
            self.fuzflag=0;
            
            
            self.getlastpathflag=0;%局部变量
            self.mapdata=zeros(50);
        end
        
        %% action主函数
        function action=action(self,observation)
%             if observation.collide
%                 action=[-10,rand(1)-0.5];
%             else
%                 action=[10,rand(1)-0.5];
%             end
            self.mapdata=double(self.mapdata|observation.scanMap');
            
            self.Initgetpath(observation);
            self.dilate();
            self.border();
            
            if self.getlastpathflag == 1
                self.checkpath();
                
            end
            
            
            if self.getpathflag==0
                %self.mapdata=observation.map';
                %self.Initgetpath(observation);
                %self.border();
                self.getobstacle();
                
                self.Ax();
                
                len=length(self.path(:,1));

                for i=2:len-2
                    kangle1=atan2((self.path(i-1,2)-self.path(i,2)),(self.path(i-1,1)-self.path(i,1)));
                    kangle2=atan2((self.path(i,2)-self.path(i+1,2)),(self.path(i,1)-self.path(i+1,1)));
                    dkangle=self.changerad(kangle1-kangle2);
%                     if self.borderMap(self.path(i,2),self.path(i,1))==2 && abs(dkangle)>0.5
%                         self.path(i)=self.path(i+1)+(self.path(i-2)-self.path(i+1))/3;
%                         self.path(i-1)=self.path(i+1)+(self.path(i-2)-self.path(i+1))/3*2;
%                     end
                    if self.path(i,2)==fix(self.path(i,2)) && self.path(i,1)==fix(self.path(i,1))
                        if self.borderMap(self.path(i,2),self.path(i,1))==2 && abs(dkangle)>0.7
                            if abs(kangle2)==pi/2 && self.borderMap(self.path(i,2),self.path(i,1)-1)==1 && self.borderMap(self.path(i,2),self.path(i,1)+1)==1
                                self.path(i,:)=self.path(i+2,:)+(self.path(i-1,:)-self.path(i+2,:))/3*2;
                                self.path(i+1,:)=self.path(i+2,:)+(self.path(i-1,:)-self.path(i+2,:))/3;
                            elseif abs(kangle1)==pi/2 && dkangle>0
                                self.path(i,:)=self.path(i+1,:)+(self.path(i-2,:)-self.path(i+1,:))/3;
                                self.path(i-1,:)=self.path(i+1,:)+(self.path(i-2,:)-self.path(i+1,:))/3*2;
                                self.path(i-2,:)=[self.path(i-2,1),self.path(i-2,2)+0.1];
                            elseif abs(kangle1)==pi/2 && dkangle<0
                                if i+1<len
                                    self.path(i+1,:)=self.path(i+1,:)-[1,0];
                                end
                                if i+2<len
                                    self.path(i+2,:)=self.path(i+2,:)-[1,0];
                                end
                            end
                            if abs(kangle2)==0 && self.borderMap(self.path(i,2)-1,self.path(i,1))==1 && self.borderMap(self.path(i,2)+1,self.path(i,1))==1
                                self.path(i,:)=self.path(i+2,:)+(self.path(i-1,:)-self.path(i+2,:))/3*2;
                                self.path(i+1,:)=self.path(i+2,:)+(self.path(i-1,:)-self.path(i+2,:))/3;
                            elseif abs(kangle1)==0
                                self.path(i,:)=self.path(i+2,:)+(self.path(i-1,:)-self.path(i+2,:))/3*2;
                                self.path(i+1,:)=self.path(i+2,:)+(self.path(i-1,:)-self.path(i+2,:))/3;
%                                 self.path(i,:)=self.path(i+1,:)+(self.path(i-2,:)-self.path(i+1,:))/3;
%                                 self.path(i-1,:)=self.path(i+1,:)+(self.path(i-2,:)-self.path(i+1,:))/3*2;
%                                 self.path(i-2,:)=[self.path(i-2,1),self.path(i-2,2)];
                            end
                        end
                    end
                end
                
                self.lastpath=self.path;
                self.getlastpathflag=1;
                
                %self.path=[self.path;self.map.start+0.5];
                self.path=self.path-0.5;
                self.getpathflag=1;
                self.pathinter();
                
                if length(self.path(:,1))>=1
                    %plot(path(:,1)-0.5,path(:,2)-0.5,'-c','LineWidth',2);
                    delete(self.pL);
                    self.pL=plot(self.path(:,1),self.path(:,2),'-r','LineWidth',2);
                end
            end
            
            
            self.now_state=[observation.agent.x,observation.agent.y,observation.agent.h];
            
            d = self.path(:,1:2) - self.now_state(1:2);
            all_distance = d(:,1).^2 + d(:,2).^2;
            [~,index] = min(all_distance);             
            min_dx = self.now_state(1) - self.path(index,1);
            min_dy = self.now_state(2) - self.path(index,2);

            angle1=atan2((self.path(index-1,2)-self.path(index,2)),(self.path(index-1,1)-self.path(index,1)));
            angle2=atan2((self.path(index-2,2)-self.path(index-1,2)),(self.path(index-2,1)-self.path(index-1,1)));
            dangle1=self.changerad(self.now_state(3)-angle1);
            dangle2=self.changerad(self.now_state(3)-angle2);
            
            if  abs(dangle1)>0.35 || abs(dangle2)>0.35
                self.fuzflag=1;
            end
            
            if  self.fuzflag==1 && abs(dangle2)<0.03
                self.fuzflag=0;
                self.intergral = 0;
                self.last_error = 0;
            end
            
            if self.fuzflag==1
                action=[0,evalfis(self.fis,dangle2)];
                return;
            end
            
            error = (sin(self.now_state(3) - atan2(min_dy,min_dx))) * sqrt(min_dx*min_dx+min_dy*min_dy);
            
            self.intergral = self.intergral + error;
            pid_value = self.kp*error + self.ki*self.intergral + self.kd*(error - self.last_error);
            self.last_error = error;
            
            action = [1, pid_value/100];
        end
        
        %% A*代价函数H
        function hcost=H(self,m,goal)
            hcost=10*abs(m(1)-goal(1))+10*abs(m(2)-goal(2));
        end
        
        %% A*搜索路径
        function []=Ax(self)
            dmap=self.borderMap';
            openlist=[];
            closelist=[];
            findflag=false;
            
            openlist=[self.map.start(1),self.map.start(2),0+self.H(self.map.start,self.map.goal),0,0,0];
            
            while ~findflag
                if isempty(openlist)
                    disp('no path!!');
                    return
                end

                [openflag,id]=self.getopenflag(self.map.goal,openlist);

                if openflag
                    disp('get goal!!');
                    closelist = [openlist(id,:);closelist];
                    findflag=true;
                    break;
                end

                minFinopen=find(openlist(:,3)==min(openlist(:,3)),1);

                closelist=[openlist(minFinopen,:);closelist]; 
                current=openlist(minFinopen,:);
                %%plot(current(:,1),current(:,2),'rx');%%
                openlist(minFinopen,:)=[];

                for i=1:8
                    nextPos=[current(1)+self.nextAx(i,1),current(2)+self.nextAx(i,2),0,0,0,0];

                    if self.isobstacle(nextPos(1:2))
                        continue;
                    end

                    nextPos(4)=current(4)+self.nextAx(i,3);

                    if dmap(nextPos(1),nextPos(2))==2
                        nextPos(3)=nextPos(4)+self.H(nextPos(1:2),self.map.goal)+50;
                    else
                        nextPos(3)=nextPos(4)+self.H(nextPos(1:2),self.map.goal);
                    end

                    [listflag,id]=self.isinlist(nextPos(1:2),openlist(:,1:2),closelist(:,1:2));
                    %flag=1不在openlist和closelist
                    %flag=2在openlist
                    %flag=3在closelist
                    if listflag == 3
                        continue;
                    elseif listflag == 1
                        nextPos(5:6)=current(1:2);
                        openlist=[openlist;nextPos];
                        continue;
                    else
                        if nextPos(3)<openlist(id,3)
                            nextPos(5:6)=current(1:2);
                            openlist(id,:)=nextPos;
                            continue;
                        end
                    end
                end
            end
            self.findpath(closelist,self.map.start);
        end
        
        %% 地图膨胀
        function []=dilate(self)
            self.dilateMap=zeros(50,50);
            for i=1:50
                for j=1:50
                    if self.mapdata(i,j)==1
                        if i>1 && i<50 && j>1 && j<50
                            self.dilateMap(i-1:i+1,j-1:j+1)=1;         
                        elseif i==1 && j>1 && j<50
                            self.dilateMap(2,j-1:j+1)=1;       
                        elseif i==50 && j>1 && j<50
                            self.dilateMap(49,j-1:j+1)=1;    
                        elseif i>1 && i<50 && j==1
                            self.dilateMap(i-1:i+1,2)=1;      
                        elseif i>1 && i<50 && j==50
                            self.dilateMap(i-1:i+1,49)=1; 
                        elseif i==1 && j==1
                            self.dilateMap(2,2)=1;            
                        elseif i==1 && j==50
                            self.dilateMap(2,49)=1;              
                        elseif i==50 && j==1
                            self.dilateMap(49,2)=1;
                        elseif i==50 && j==50
                            self.dilateMap(49,49)=1;
                        end
                    end
                end
            end
            self.dilateMap(1:50,1)=1;
            self.dilateMap(1:50,50)=1;
            self.dilateMap(1,1:50)=1;
            self.dilateMap(50,1:50)=1;
            %self.dilateMap(48:50,41:43)=0;
            self.dilateMap(self.map.goal(2)-1:self.map.goal(2)+1,self.map.goal(1)-1:self.map.goal(1)+1)=0;
        end
        
        %% 地图边缘增加权重
        function []=border(self)
            self.borderMap=self.dilateMap;
            for i=2:49
                for j=2:49
                    if self.dilateMap(i,j)==1
                        for k=1:8
                            if self.dilateMap(i+self.nextborder(k,1),j+self.nextborder(k,2))~=1
                                self.borderMap(i+self.nextborder(k,1),j+self.nextborder(k,2))=2;
                            else
                                continue;
                            end
                        end
                    end
                    if i==2||j==2||i==49||j==49
                        if self.dilateMap(i,j)==0
                            self.borderMap(i,j)=2;
                        end
                    end
                end
            end
            %self.borderMap(48:50,41:43)=0;
            self.borderMap(self.map.goal(2)-1:self.map.goal(2)+1,self.map.goal(1)-1:self.map.goal(1)+1)=0;
        end
        
        %% 寻路初始化
        function []=Initgetpath(self,observation)
            self.map.start=[round(observation.agent.x),round(observation.agent.y)];
            self.map.goal=[observation.endPos.x+1,observation.endPos.y];
            self.map.XYMAX=50;
            %self.dilate();
            %self.border();
%             self.map.start=[observation.agent.x,observation.agent.y];
%             self.map.goal=[observation.endPos.x+1,observation.endPos.y];
%             self.map.XYMAX=50;
            
%             self.obstacle=[];
%             for i=1:self.map.XYMAX
%                 for j=1:self.map.XYMAX
%                     if self.borderMap(i,j) == 1
%                         self.obstacle=[self.obstacle;[j i]];
%                     end
%                     if self.borderMap(i,j) ==1
%                         plot(j-0.5,i-0.5,'rx');%在每一格是障碍物的地方画红色叉
%                     end
%                     if self.borderMap(i,j) ==2
%                         plot(j-0.5,i-0.5,'b*');%在每一格是障碍物的地方画红色叉
%                     end
%                 end
%             end
        end
       
        
        %%
        function []=getobstacle(self)
            self.obstacle=[];
            for i=1:self.map.XYMAX
                for j=1:self.map.XYMAX
                    if self.borderMap(i,j) == 1
                        self.obstacle=[self.obstacle;[j i]];
                    end
                    if self.borderMap(i,j) ==1
                        plot(j-0.5,i-0.5,'rx');%在每一格是障碍物的地方画红色叉
                    end
                    if self.borderMap(i,j) ==2
                        plot(j-0.5,i-0.5,'b*');%在每一格是障碍物的地方画红色叉
                    end
                end
            end
        end
        
        %% Ax算法中
        function [openflag,id] = getopenflag(self,node,open)
            openflag = 0;
            id = 0;
            if  isempty(open)
                openflag = 0;
            else
                for i = 1:length(open(:,1))
                   if isequal(node(1:2),open(i,1:2))
                        openflag = 1;
                        id = i;
                        return;
                   end 
                end
            end
        end
        
        %%
        function [flag,id]=isinlist(self,pos,open,close)
        %flag=1不在openlist和closelist
        %flag=2在openlist
        %flag=3在closelist
            if isempty(open)
                flag=1;
                id=[];
            else
                for i=1:length(open(:,1))
                    if isequal(pos,open(i,:))
                        flag=2;
                        id=i;
                        return;
                    else
                        flag=1;
                        id=[];
                    end
                end
            end
            for i=1:length(close(:,1))
                if isequal(pos,close(i,:))
                    flag=3;
                    id=[];
                    return;
                end
            end
        end
        
        %%
        function flag=isobstacle(self,pos)
            for i=1:length(self.obstacle(:,1))
                if isequal(self.obstacle(i,:),pos)
                    flag=true;
                    return;
                end
            end
            flag=false;
        end
        
        %%
        function []=findpath(self,close,start)
            id=1;
            self.path=[];
            while 1
                self.path=[self.path;close(id,1:2)];
                if isequal(close(id,1:2),start)
                    break;
                end
                for i=1:length(close(:,1))
                    if isequal(close(i,1:2),close(id,5:6))
                        %close(id,:)=[];
                        id=i;
                        break;
                    end
                end
            end
        end
        
        %% path进行插值
        function []=pathinter(self)
            pathin=zeros(9,2);
            len=length(self.path(:,1));
            for i=1:len-1
                k=10*i-9;
                for j=1:9
                    pathin(j,1)=self.path(k,1)+j*(self.path(k+1,1)-self.path(k,1))/10;
                    pathin(j,2)=self.path(k,2)+j*(self.path(k+1,2)-self.path(k,2))/10;
                end
                self.path=[self.path(1:k,:);pathin;self.path(k+1:end,:)];
            end
        end
        %%
        function out = changerad(self,in)
            in = mod(in,2*pi);
            out = in.*(0<=in & in <= pi) + (in - 2*pi).*(pi<in & in<2*2*pi);   % x in (-pi,pi]
        end
        %%
        function checkpath(self)
            len = length(self.lastpath(:,1));
            for i=1:len
                if self.borderMap(self.lastpath(i,2),self.lastpath(i,1))==1||self.borderMap(self.lastpath(i,2),self.lastpath(i,1))==2
                    self.getpathflag=0;
                    break;
                end
            end
        end
    end
end