close all

% figuras para láser y VFH
fig_laser = figure; title('LASER')
fig_vfh = figure; title('VFH')

% crear el objeto VFH
VFH = robotics.VectorFieldHistogram; 

% ajustamos sus propiedades
VFH.NumAngularSectors = 180;
VFH.DistanceLimits = [0.05 3];
VFH.RobotRadius = 0.15; % 0.15
VFH.SafetyDistance = 0.2; % 0.2
VFH.MinTurningRadius = 0.1; % 0.1
VFH.TargetDirectionWeight = 5; % 6
VFH.CurrentDirectionWeight = 2; % 2
VFH.PreviousDirectionWeight = 2; % 2
VFH.HistogramThresholds = [3 10]; % [0.2 6]
VFH.UseLidarScan = true; % para permitir utilizar la notación del scan

% rellenamos los campos por defecto de la velocidad del robot, para que la lineal sea siempre 0.1 m/s
target_dir = 0;
msg_vel.Linear.X = 0.1;
msg_vel.Linear.Y = 0;
msg_vel.Linear.Z = 0;
msg_vel.Angular.X = 0;
msg_vel.Angular.Y = 0;
msg_vel.Angular.Z = 0;
send(pub_vel, msg_vel);

while(1)
    
    % leer y dibujar los datos del láser en la figura ‘fig_laser’
    figure(fig_laser)
    lee_sensores;
    
    % llamar al objeto VFH para obtener la dirección a seguir por el robot para
    % evitar los obstáculos. Mostrar los resultados del algoritmo (histogramas)
    % en la figura ‘fig_vfh’
    steeringDir = VFH(lidarScan(msg_laser), target_dir);
    figure(fig_vfh);
    show(VFH);
    
    % rellenar el campo de la velocidad angular del mensaje de velocidad con un
    % valor proporcional a la dirección anterior (K = 0.1)
    K = 0.1;
    V_ang = K * steeringDir;
    msg_vel.Angular.Z = V_ang;
    
    % publicar el mensaje de velocidad
    pub_vel = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
   	send(pub_vel, msg_vel);
    
    % esperar al siguiente periodo de muestreo
    waitfor(r);
end