distancia = 1;

msg_vel.Linear.X = 0.1;
msg_vel.Linear.Y = 0;
msg_vel.Linear.Z = 0;
msg_vel.Angular.X = 0;
msg_vel.Angular.Y = 0;
msg_vel.Angular.Z = 0;

% Guardamos la posici�n inicial
initpos = sub_odom.LatestMessage.Pose.Pose.Position;

while (1)
   % Actualizamos la posici�n
   pos = sub_odom.LatestMessage.Pose.Pose.Position;
   d = sqrt((initpos.X - pos.X)^2 + (initpos.Y - pos.Y)^2); 
   disp(d);
   if (d > distancia)
       msg_vel.Linear.X = 0;
       send(pub_vel, msg_vel);
       break;
   else
       send(pub_vel, msg_vel);
   end
   lee_sensores;
   waitfor(r);
end

