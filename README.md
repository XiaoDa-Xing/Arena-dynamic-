# Arena Challenge简介
Arena Challenge是一个由Matlab代码编写的仿真环境。主要要求挑战者编写Matlab代码控制小车在一个地图环境中从起点运行至终点，地图中随机放置着障碍物(见下图）。小车是一个unicycle的动力系统（见下文解释），并装有“雷达”探测器，可探测前方障碍物情况。最终的成绩由小车到达终点的时间以及小车是否撞上障碍物等情况综合评估而得。

挑战者提交其设计的控制策略，我们将测试控制策略在随机地图和不同配置下的得分情况，得到策略的最终得分。

<div align="left">
<img src=https://gitee.com/coralab/ic-challenge/raw/master/Arena/pics/arena_preview.png width=60%/>
</div>


## 1. 小车动力学模型
小车的动力学模型，如下公式所示
<div align="left">
<img src=https://gitee.com/coralab/ic-challenge/raw/master/Arena/pics/unicycle.png width=30%/>
</div>


其中，[x,y,$\theta$]分别是小车当前的空间位置和朝向。[u,v]是小车的控制量，u相当于小车的“油门”踏板，控制小车的前车速度；v是小车的“方向盘”，控制小车的转动速度。这是挑战者在编写代码时，唯一可以控制小车的两个物理量。

另外仿真环境对小车设置了“饱和”机制（如下图所示），即挑战者传递给仿真器的小车两个控制量，将会被限制在一定范围内。

<div align="left">
<img src=https://gitee.com/coralab/ic-challenge/raw/master/Arena/pics/saturation.png width=30%/>
</div>


## 2. 雷达探测器
小车前方有一个前向的雷达传感器，可探测前方是否有障碍物，如有障碍物将，将会把障碍物信息告知挑战者。见图中的小车前方红色三角形所表示的雷达探测范围。

## 3. Observation（当前环境信息）

仿真环境每隔一段时间，将会把仿真环境的信息以Observation类的形式告知挑战者，它的成员变量包含  
```
agent		%当前小车信息  
scanMap	%当前雷达探测器探测到的信息  
t            	%当前所用时间  
collide    	%当前是否发生碰撞  
score      	%当前挑战者的分数  
startPos   	%小车起始位置  
endPos     	%小车目标位置  
map         	%全局地图  
```

## 4. 得分
在挑战开始时，挑战者将有一个基本分数，如挑战者控制小车与障碍物发生碰撞则会相应地扣分，如挑战者控制小车驶离地图范围会相应地扣分（扣分权重很大），如挑战者控制小车未在规定时间内到达终点，则超出时间每秒都会相应地扣分。

## 5. 设计控制策略
挑战者需要设计并提交一个Policy类文件，主要完成action函数。action函数传入参数为observation，传出action。仿真器会在特点的时间间隔调用action，依据挑战者设计的策略得到action，即控制量[u,v]，从而控制小车。
```
classdef Policy < handle        function action=action(self,observation)            if observation.collide                action=[-10,rand(1)-0.5];            else                action=[10,rand(1)-0.5];            end        end
end
```

## 6. Main函数
以下是Main函数的基本代码，main读取系统配置文件对仿真环境进行配置，之后进入仿真循环。在循环中，仿真器对Arena Challenge进行物理仿真，计算出小车位置和状态，是否发生碰撞等信息。并且每一次都用挑战者设计控制策略，然后把控制策略作用于小车的控制中。
```
env = Env('sys.ini');   %读取系统配置文件policy=Policy();if (env.succeed)    observation = env.reset();    while 1        env.render();        action=policy.action(observation); %调用挑战者的控制策略        [observation,done,info]=env.step(action); %物理仿真        disp(info);        if(done)            break;        end        wait(100);    endend```

## 7. 仿真配置文件sys.ini
为了方便挑战者进行测试，挑战者可以通过仿真配置文件sys.ini，进行相应配置。例如配置小车的起始和终止点，小车控制饱和范围，是否录制游戏运行过程等。具体见该文件。

## 8. 挑战模式
在sys.ini中，把globalview设置为1，挑战者就可以在开始时从observation中获取全局地图信息；如把globalview设置为0，挑战者就只在每次系统刷新时从observation中获取传感器获得的局部地图信息；


# 代码提交
请以队伍的形式登录以下网站注册并提交Policy类代码

<http://www.rayliable.net/manage/account/login>

# Leader Board(排名榜)
<http://www.rayliable.net/manage/item/ranking>


