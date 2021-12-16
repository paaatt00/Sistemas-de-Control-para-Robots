angulo = -pi/2;

if (angulo > 0)
    velocidad = 0.1;
else
    velocidad = -0.1;
end

msg_vel.Linear.X = 0;
msg_vel.Linear.Y = 0;
msg_vel.Linear.Z = 0;
msg_vel.Angular.X = 0;
msg_vel.Angular.Y = 0;
msg_vel.Angular.Z = velocidad;

initori = sub_odom.LatestMessage.Pose.Pose.Orientation;
initang_euler = quat2eul([initori.W initori.X initori.Y initori.Z]);
yawini = initang_euler(1);

while (1)
    ori = sub_odom.LatestMessage.Pose.Pose.Orientation;
    ang_euler = quat2eul([ori.W ori.X ori.Y ori.Z]);
    yaw = ang_euler(1);
    ang = angdiff(yawini,yaw);
    if (abs(ang) > abs(angulo))
       msg_vel.Angular.Z = 0;
       send(pub_vel, msg_vel);
       break;
   else
       send(pub_vel, msg_vel);
   end
   lee_sensores;
   waitfor(r);
end