%
% set v and omega to allow smart car to drive in circle
%

robot.close;

robot=ElegooRobotCar('192.168.4.1');

robot.stop;
robot.setWatchdog(1000);
stopOnExit = onCleanup(@() stopRobotSafely(robot)); %#ok<NASGU>

clear t y img 

v = 100;
w = 10;
T = 5;
img = {};

% start the loop
k=1;
k_s=1;
n_save=50;
t(1)=0;

while t(k)<T
tic;
if mod(k-1,n_save)==0
    img{k_s} = robot.capture;
    k_s=k_s+1;
end
robot.forward(v);
y(k)=robot.readIMU;
robot.turnRight(w);
k=k+1;
t(k)= t(k-1)+toc;
end

robot.stop

w = [y.gyro];
a = [y.accel];

figure,plot(t(1:end-1),w(3,:));
xlabel('time (s)');ylabel('\omega_z');
figure,plot(t(1:end-1),a(1,:),t(1:end-1),a(2,:));
xlabel('time (s)');ylabel('a_x/a_y');legend('a_x','a_y');


function stopRobotSafely(robot)
    try
        robot.stop();
    catch
end

end
