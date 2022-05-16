clear all;
close all;
clc;

abspath=utils('abspath');
env = Env(abspath('sys.ini'));
policy=Policy();
if (env.succeed)
    observation = env.reset();
    while 1
        env.render();
        action=policy.action(observation);
        
        [observation,done,info]=env.step(action);
        
        disp(info);
        if(done)
            break;
        end
        wait(10);
    end
end



function wait (ms)
time=ms/1000;
% tic
pause(time)
% toc
end

