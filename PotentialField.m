

classdef PotentialField < handle
    properties
        pfmap % potential map for the smart agent to avoid obstacles
        xgrad
        ygrad
        dismap
    end

    methods
        
        function self = PotentialField(map)
            self.dismap=bwdist(map);
            rho=bwdist(map)/100+1;
            
%             tmp=(rho-min(min(rho)))/(max(max(rho))-min(min(rho)));
%             figure;
%             imshow(tmp)
%             figure;
%             imshow(map);
            influence=1.1;
            scale=5000;
            self.pfmap=scale*((1./rho-1/influence).^2);
            self.pfmap(rho>influence)=0;
            
            [self.xgrad,self.ygrad]=gradient(-self.pfmap);
           
%             figure;
%             s=size(map);
%              [xx,yy] = meshgrid(1:1:s(2),1:1:s(1));
%              surface(xx,yy,self.pfmap);
%             figure;
%             s=size(map);
%             [xx,yy] = meshgrid(1:1:s(2),1:1:s(1));
%             quiver(xx,yy,self.xgrad,self.ygrad)
            
        end
        
    end
end