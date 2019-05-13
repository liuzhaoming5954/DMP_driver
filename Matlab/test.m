clear;
load trajectory.txt;

traj.times=trajectory(:,1)';
traj.x=trajectory(:,2)';
traj.vx=trajectory(:,3)';
traj.ax=trajectory(:,4)';

% traj.x=targetJointsActual(:,jointnum)';%filter(f,x);
% traj.vx=JointsVelocity(:,jointnum)';%filter(f,vx);
% traj.ax=JointsAcceleration(:,jointnum)';%filter(f,ax);

% dt = 0.0;
% t = 0:dt:1;
% traj.times=t;

% start time is initialized to zero! important for s
%traj.times=traj.times-traj.times(1);

nbfs = 20;
par=struct('ng', nbfs, 'h', ones(1,nbfs)*0.5, 's', 1, 'as', 0.05, 'K', 5, 'D', 10);
r=dmpTrain_1D(traj, par); 
result=dmpTest_1D(r);
figure(1);
plot(traj.times,traj.x(1,:),traj.times,result.x_dmp(1,:),'linewidth', 2);
%plot(traj.times,result.x_dmp(1,:));
legend('original', 'dmp');

    