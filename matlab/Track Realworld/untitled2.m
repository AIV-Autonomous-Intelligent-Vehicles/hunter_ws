sub = rossubscriber("/imu/data","DataFormat","struct");


while 1
    imuMsg = receive(sub);
    
    quat = [imuMsg.Orientation.W,imuMsg.Orientation.X,imuMsg.Orientation.Y,imuMsg.Orientation.Z];
    euler = quat2eul(quat);
    disp(euler(1))
    % pause(2)
end