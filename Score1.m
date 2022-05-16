classdef Score < handle
    properties
        score
    end
    properties (Access = private)
        rewardTime
        lastTime
        outofbound
        collide
    end
    
    
    methods
    
        function self = Score(env)
            self.score=env.sysInfo.score;
            
            
            l=sqrt(env.map.w*env.map.w+env.map.h*env.map.h);
            self.rewardTime=ceil(l/env.agentInfo.usat*env.sysInfo.scoreTimes);
            %disp(self.rewardTime);
            self.lastTime=0;
            self.outofbound=0;
            self.collide=0;
        end

        function assess(self,env)
            agent=env.getMainAgent();
            mapw=env.map.w;
            maph=env.map.h;
            if agent.x<0 || agent.x>mapw || agent.y<0 || agent.y>maph
                self.outofbound=1;
            end
            if env.collide
                self.collide=1;
            end
            if(env.t>=self.lastTime+1)
                if(self.outofbound)
                    self.score=self.score-env.sysInfo.obDeducts;
                end
                if(self.collide)
                    self.score=self.score-env.sysInfo.collideDeducts;
                end
                if(env.t>self.rewardTime)
                    self.score=self.score-env.sysInfo.timeoutDeducts;
                end
                self.lastTime=env.t;
                self.collide=0;
                self.outofbound=0;
            end
            
        end
       
    end
end