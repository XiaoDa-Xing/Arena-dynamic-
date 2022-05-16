% functions for planar drawing
function f=utils(refname)
if strcmp(refname,'drawCircles')
    f=@drawCircles;
end
if strcmp(refname,'drawTriange')
    f=@drawTriange;
end
if strcmp(refname,'drawRect')
    f=@drawRect;
end
if strcmp(refname,'drawAxis')
    f=@drawAxis;
end
if strcmp(refname,'randomLaplacian')
    f=@randomLaplacian;
end

if strcmp(refname,'rotMat2')
    f=@rotMat2;
end
if strcmp(refname,'insidePolygon')
    f=@insidePolygon;
end


if strcmp(refname,'sat')
    f=@sat;
end

if strcmp(refname,'abspath')
    f=@abspath;
end

if strcmp(refname,'projVec')
    f=@projVec;
end

if strcmp(refname,'projOrthVec')
    f=@projOrthVec;
end

end

function p=abspath(file)
    if ismac
    	c='/';
    elseif isunix
    	c='/';
    elseif ispc
        c='\';
    else
        c='';
        error('Platform not supported');
    end
    p=strcat(pwd,c,file);
end

function proj=projVec(from,to)
    proj=dot(from,to)*to/norm(to);
    if(size(proj)==[1 2])
        proj=proj';
    end
end

function proj=projOrthVec(from,to)
    to=[-to(2) to(1)];
    proj=projVec(from,to);
end

function drawArc(handle,center,radius,st_ang,end_ang) % radius-Arc Radius
circr = @(radius,rad_ang)  [radius*cos(rad_ang);  radius*sin(rad_ang)];         % Circle Function For Angles In Radians
circd = @(radius,deg_ang)  [radius*cosd(deg_ang);  radius*sind(deg_ang)];       % Circle Function For Angles In Degrees
N = 10;                                                         % Number Of Points In Complete Circle
N=max(N,ceil((end_ang-st_ang)*180/pi/3.6))
linspace(st_ang,end_ang,N)
                                            
xy_r = circr(radius,r_angl);                                    % Matrix (2xN) Of (x,y) Coordinates
xy_r=xy_r+center;
plot(handle,xy_r(1,:), xy_r(2,:), '-') ;
end

function rm=rotMat2(angle) % return rotation matrix by rads
rm=[cos(angle) -sin(angle);sin(angle) cos(angle)];
end

function action=sat(action,satLevel)
if(length(action)~=length(satLevel))
    action =-1;
    return;
end
for i=1:length(action)
    
    if(abs(action(i))>satLevel(i))
        action(i)=satLevel(i)*sign(action(i));
    end
end
end

function istrue=insidePolygon(polygon,point)
len=size(polygon,2);

A=zeros(len,1);B=A;C=A;D=A;
for i =1:len
    p1 = polygon(:,i);
    if(i+1<=len)
        p2 = polygon(:,i+1);
    else
        p2 = polygon(:,rem(1,len));
    end
    
    a = -(p2(2) - p1(2));
    b = p2(1) - p1(1);
    c = -(a * p1(1) + b * p1(2));
    
    A(i)=a;
    B(i)=b;
    C(i)=c;
end


for i =1:len
    d = A(i) * point(1)+ B(i) * point(2) + C(i);
    D(i)=d;
end

t1=sum( D>=0)==len;
t2=sum(D<=0)==len;
istrue= t1|t2;
end


function  drawCircles(center,s,color)
[N,n]=size(center);
if n~=2
    return
end
for n=1:N
    viscircles(center(n,:), s,'Color',color,'LineWidth',0.5);
end
end

function drawTriange(center,s,color)
[N,n]=size(center);
theta=60/180*pi;
if n~=2
    return
end
for n=1:N
    x=center(n,1);
    y=center(n,2);
    p=[x y+s;
        x-s*cos(theta) y-s*sin(theta);
        x+s*cos(theta) y-s*sin(theta)];
    p=[p;
        p(1,:)];
    plot(p(:,1),p(:,2),color);
end
end


function drawRect(center,s,color)
[N,n]=size(center);
if n~=2
    return
end
for n=1:N
    x=center(n,1);
    y=center(n,2);
    p=[x-s y-s;
        x-s y+s;
        x+s y+s;
        x+s y-s];
    p=[p;
        p(1,:)];
    plot(p(:,1),p(:,2),color);
end
end


function drawAxis(sp,ep,color)
[N1,n1]=size(sp);
[N2,n2]=size(ep);
if n1~=2 || n2~=2 || N1~=N2
    return
end

ang_inc=15/180*pi;

len=1;
for n=1:N1
    ang=angle(sp(n,1)-ep(n,1)+i*(sp(n,2)-ep(n,2)));
    a1=ang+ang_inc;
    a2=ang-ang_inc;
    plot([sp(n,1) ep(n,1)],[sp(n,2) ep(n,2)],'Color',color);
    plot([ep(n,1) ep(n,1)+len*cos(a1)],[ep(n,2) ep(n,2)+len*sin(a1)],'Color',color);
    plot([ep(n,1) ep(n,1)+len*cos(a2)],[ep(n,2) ep(n,2)+len*sin(a2)],'Color',color);
    % spp=sp(n,:);
    % epp=ep(n,:);
    % annotation('arrow',[spp(1) epp(1)],[spp(2) epp(2)],'Color',color);
end
end


function drawAxis1(sp,ep,color)
[N1,n1]=size(sp);
[N2,n2]=size(ep);
if n1~=2 || n2~=2 || N1~=N2
    return
end

for n=1:N1
    spp=sp(n,:);
    epp=ep(n,:);
    quiver(spp(1),spp(2),epp(1)-spp(1),epp(2)-spp(2),'Color',color )
end
end



