classdef Observation < handle
    properties
        agent  
        scanMap
        t
        collide
        score
        startPos
        endPos
        map
    end
    
    
    methods
    
        function setObservation(self,env,globalview)
            self.agent=env.getMainAgent();
            self.scanMap=env.scanMap';
            self.t=env.t;
            self.score=env.score.score;
            self.collide=env.collide;
            self.startPos=env.startPos;
            self.endPos=env.endPos;
            if globalview
                self.map=env.map.occupyMap();
                self.map=self.map';
            else
                self.map='';
            end
        end


        function self = Observation()
        end

       
    end
end