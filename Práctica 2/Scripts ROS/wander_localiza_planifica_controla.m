clc;
close all;

% load the map of the simulation world /real environment
load simple_rooms_modified.mat;
map = map_modified;
% show(map);

% figuras para láser y VFH
fig_laser = figure; title('LASER')
fig_vfh = figure; title('VFH')

% crear el objeto VFH
VFH = robotics.VectorFieldHistogram; 

% setup the laser sensor model and amigobot motion model
odometryModel = robotics.OdometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];

% The sensor on amigobot is a simulated/real range finder converted from laser readings
rangeFinderModel = robotics.LikelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0 8];
rangeFinderModel.Map = map;

% query the Transformation Tree (tf tree) in ROS
tftree = rostf;

% obtener transformada entre los frames del robot y del sensor_laser
waitForTransform(tftree,'/robot0','/robot0_laser_1');
sensorTransform = getTransform(tftree,'/robot0', '/robot0_laser_1');

% get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

% setup the |SensorPose|, which includes the translation along base_link's 
% +X, +Y direction in meters and rotation angle along base_link's +Z axis
% in radians.
rangeFinderModel.SensorPose = [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];

% initialize AMCL object
amcl = robotics.MonteCarloLocalization;
amcl.UseLidarScan = true;

% assign the |MotionModel| and |SensorModel| properties in the |amcl| object.
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;

% The particle filter only updates the particles when the robot's movement exceeds 
% the |UpdateThresholds|, which defines minimum displacement in [x, y, yaw] to 
% trigger filter update. This prevents too frequent updates due to sensor noise. 
% Particle resampling happens after the |amcl.ResamplingInterval| filter updates. 
% Using larger numbers leads to slower particle depletion at the price of slower 
% particle convergence as well.

amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;

% configure AMCL object for localization with initial pose estimate
amcl.ParticleLimits = [500 50000];       % Minimum and maximum number of particles
amcl.GlobalLocalization = true;          % global = true      local = false
amcl.InitialPose = [0 0 0];              % Initial pose of vehicle   
amcl.InitialCovariance = eye(3);         % Covariance of initial pose

% setup helper for visualization and driving AmigoBot

visualizationHelper = ExampleHelperAMCLVisualization(map);

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

% crear el objeto PurePursuit y ajustar sus propiedades
controller = robotics.PurePursuit;
controller.DesiredLinearVelocity = 0.1;
controller.LookaheadDistance = 1; % 0.5
controller.MaxAngularVelocity = 0.3;

% rellenamos los campos por defecto de la velocidad del robot, para que la lineal sea siempre 0.1 m/s
target_dir = 0;
msg_vel.Linear.X = 0.1;
msg_vel.Linear.Y = 0;
msg_vel.Linear.Z = 0;
msg_vel.Angular.X = 0;
msg_vel.Angular.Y = 0;
msg_vel.Angular.Z = 0;
send(pub_vel, msg_vel);

% localization procedure
i = 1;
while (1)
    
    % leer y dibujar los datos del láser en la figura ‘fig_laser’
    figure(fig_laser)
    lee_sensores;
    
    % receive laser scan and odometry message.
    scan = receive(sub_laser);
    odompose = sub_odom.LatestMessage;
    
    % crear objeto para almacenar el escaneo LiDAR 2-D
    scans = lidarScan(scan);
    
    % compute robot's pose [x, y, yaw] from odometry message.
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y, odomRotation(1)];
    
    % update estimated robot's pose and covariance using new odometry and sensor readings.
    [isUpdated, estimatedPose, estimatedCovariance] = amcl(pose, scans);
    
    % plot the robot's estimated pose, particles and laser scans on the map.
    if isUpdated
        i = i + 1;
        plotStep(visualizationHelper, amcl, estimatedPose, scans, i)
    end
    
    estimatedCovariance
    
    % si la covarianza está por debajo de un umbral, el robot está 
    % localizado y finaliza el programa
    % global 0.03 0.02 0.03
    % local 0.015 0.015 0.05
    if (estimatedCovariance(1,1) < 0.015 && estimatedCovariance(2,2) < 0.015 && estimatedCovariance(3,3) < 0.05) 
        disp('Robot Localizado');
        break;
    end
    
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

% paramos el robot, para que no avance mientras planificamos
% hacemos una copia del mapa, para “inflarlo” antes de planificar
msg_vel.Linear.X = 0;
pub_vel = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
send(pub_vel, msg_vel);

disp(estimatedPose)
% definir posición de inicio y de destino
startLocation = [estimatedPose(1,1) estimatedPose(1,2)]
endLocation = [0 0];

cpMap = copy(map);
inflate(cpMap, 0.25);

% crear el objeto PRM y ajustar sus parámetros
planner = robotics.PRM(cpMap, 500);
planner.ConnectionDistance = 2;

% obtener la ruta hacia el destino desde la posición actual del robot y mostrarla
% en una figura
ruta = findpath(planner, startLocation, endLocation);

figure;
show(planner);

%Indicamos al controlador la lista de waypoints a recorrer (ruta)
controller.Waypoints = ruta;

figure;
cpMap = copy(map);
inflate(cpMap, 0.25);

while (1)
    
    % leer el láser y la odometría
    lee_sensores;
    
    scan = receive(sub_laser);
    odompose = sub_odom.LatestMessage;
    scans = lidarScan(scan);
    
    % obtener la posición pose = [x,y,yaw] a partir de la odometría anterior
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y, odomRotation(1)];
    
    % ejecutar amcl para obtener la posición estimada estimatedPose
    [isUpdated, estimatedPose, estimatedCovariance] = amcl(pose, scans);
    
    % dibujar los resultados del localizador con el visualizationHelper
    if isUpdated
        i = i + 1;
        plotStep(visualizationHelper, amcl, estimatedPose, scans, i)
    end
    
    % ejecutar el controlador PurePursuit para obtener las velocidades
    % lineal y angular
    [lin_vel, ang_vel] = controller(estimatedPose);
    
    % rellenar los campos del mensaje de velocidad
    msg_vel.Linear.X = lin_vel;
    msg_vel.Angular.Z = ang_vel;
    
    % publicar el mensaje de velocidad
    pub_vel = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
   	send(pub_vel, msg_vel);
    
    % comprobar si hemos llegado al destino, calculando la distancia 
    % euclidea y estableciendo un umbral
    if(estimatedPose(1,1) < 0.2 && estimatedPose(1,2) < 0.2)
        disp('Navegación completada');
        msg_vel.Linear.X = 0;
        msg_vel.Angular.Z = 0;
        pub_vel = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
        send(pub_vel, msg_vel);
        break;
    end
    
    % esperar al siguiente periodo de muestreo
    waitfor(r);

end