clc;
close all;

% load the map of the simulation world /real environment
% load simple_rooms_modified.mat;
load pasillo_modified.mat;
map = map_modified;
% show(map);
        
% figuras para l?ser y VFH
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
waitForTransform(tftree, 'base_link', 'laser_frame');
sensorTransform = getTransform(tftree, 'base_link', 'laser_frame');

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
%amcl.GlobalLocalization = true;          % global = true      local = false
%amcl.InitialPose = [0 0 0];              % Initial pose of vehicle   
amcl.InitialCovariance = eye(3);         % Covariance of initial pose

amcl.GlobalLocalization = true;          % global = true      local = false
amcl.InitialPose = [0 0 0];              % Initial pose of vehicle 

% setup helper for visualization and driving AmigoBot
visualizationHelper = ExampleHelperAMCLVisualization(map);

% ajustamos sus propiedades
VFH.NumAngularSectors = 180;
VFH.DistanceLimits = [0.2 2];
VFH.RobotRadius = 0.2; 
VFH.SafetyDistance = 0.3; 
VFH.MinTurningRadius = 0.1; 
VFH.TargetDirectionWeight = 5; 
VFH.CurrentDirectionWeight = 2; 
VFH.PreviousDirectionWeight = 2; 
VFH.HistogramThresholds = [3 10]; 
VFH.UseLidarScan = true; % para permitir utilizar la notaci?n del scan

% crear el objeto PurePursuit y ajustar sus propiedades
controller = robotics.PurePursuit;
controller.DesiredLinearVelocity = 0.1;
controller.LookaheadDistance = 3; 
controller.MaxAngularVelocity = 0.5;

% rellenamos los campos por defecto de la velocidad del robot, para que la lineal sea siempre 0.1 m/s
target_dir = 0;
msg_vel.Linear.X = 0.1;
msg_vel.Linear.Y = 0;
msg_vel.Linear.Z = 0;
msg_vel.Angular.X = 0;
msg_vel.Angular.Y = 0;
msg_vel.Angular.Z = 0;
send(pub_vel, msg_vel);

% activar motores enviando enable_motor = 1
msg_enable_motor.Data = 1;
send(pub_enable, msg_enable_motor);

% localization procedure
i = 1;
while (1)
    
    % leer y dibujar los datos del l?ser en la figura ?fig_laser?
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
    
    % si la covarianza est? por debajo de un umbral, el robot est? 
    % localizado y finaliza el programa
    if (estimatedCovariance(1,1) < 0.08 && estimatedCovariance(2,2) < 0.06 && estimatedCovariance(3,3) < 0.7) 
        disp('Robot Localizado');
        break;
    end
    
    % llamar al objeto VFH para obtener la direcci?n a seguir por el robot para
    % evitar los obst?culos. Mostrar los resultados del algoritmo (histogramas)
    % en la figura ?fig_vfh?
    steeringDir = VFH(lidarScan(msg_laser), target_dir);
    figure(fig_vfh);
    show(VFH);
    
    % rellenar el campo de la velocidad angul ar del mensaje de velocidad con un
    % valor proporcional a la direcci?n anterior (K = 0.1)
    K = 0.5;
    V_ang = K * steeringDir;
    msg_vel.Angular.Z = V_ang;
    
    % publicar el mensaje de velocidad
   	send(pub_vel, msg_vel);
    
    % esperar al siguiente periodo de muestreo
    waitfor(r);

end

% paramos el robot, para que no avance mientras planificamos
% hacemos una copia del mapa, para ?inflarlo? antes de planificar
msg_vel.Linear.X = 0;
msg_vel.Angular.Z = 0;
send(pub_vel, msg_vel);

% desactivar motores enviando enable_motor = 0
% msg_enable_motor.Data = 0;
% send(pub_enable, msg_enable_motor);

disp(estimatedPose)
% definir posici?n de inicio y de destino
startLocation = [estimatedPose(1,1) estimatedPose(1,2)]
endLocation = [0 0]; %[17 3];

cpMap = copy(map);
inflate(cpMap, 0.25);

% crear el objeto PRM y ajustar sus par?metros
planner = robotics.PRM(cpMap, 1000);
planner.ConnectionDistance = 2;

% obtener la ruta hacia el destino desde la posici?n actual del robot y mostrarla
% en una figura
ruta = findpath(planner, startLocation, endLocation);

figure(4);
show(planner);

% Indicamos al controlador la lista de waypoints a recorrer (ruta)
controller.Waypoints = ruta;

figure(5);
cpMap = copy(map);
inflate(cpMap, 0.25);
show(cpMap);

% activar motores enviando enable_motor = 1
% msg_enable_motor.Data = 1;
% send(pub_enable, msg_enable_motor);
msg_vel.Linear.X = 0.1;
send(pub_vel, msg_vel);

while (1)
    
    % leer el l?ser y la odometr?a
    figure(fig_laser);
    lee_sensores;
    
    scan = receive(sub_laser);
    odompose = sub_odom.LatestMessage;
    scans = lidarScan(scan);
    
    % obtener la posici?n pose = [x,y,yaw] a partir de la odometr?a anterior
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y, odomRotation(1)];
    
    % ejecutar amcl para obtener la posici?n estimada estimatedPose
    [isUpdated, estimatedPose, estimatedCovariance] = amcl(pose, scans);
    
    % dibujar los resultados del localizador con el visualizationHelper
    if isUpdated
        i = i + 1;
        plotStep(visualizationHelper, amcl, estimatedPose, scans, i)
    end
    
    % ejecutar el controlador PurePursuit para obtener las velocidades
    % lineal y angular
    [lin_vel, ang_vel] = controller(estimatedPose);
    
    % llamar a VFH pas?ndole como ?targetDir? un valor proporcional a la
    % velocidad angular calculada por el PurePursuit
    K1 = 1; 
    K2 = 0.45; 
    targetdir = K1 * ang_vel;
    direccion = VFH(scans, targetdir);
    ang_vel_vfh = K2 * direccion;
    
    figure(fig_vfh);
    show(VFH);
    
    % calcular la velocidad angular final como una combinaci?n lineal de la
    % generada por el controlador PurePursuit y la generada por VFH
    ang_vel_tot = ang_vel + ang_vel_vfh;
    
    % rellenar los campos del mensaje de velocidad
    msg_vel.Linear.X = lin_vel;
    msg_vel.Angular.Z = ang_vel_tot;
    
    % publicar el mensaje de velocidad
   	send(pub_vel, msg_vel);
    
    estimatedPose
    
    % comprobar si hemos llegado al destino, calculando la distancia 
    % euclidea y estableciendo un umbral
    dist_euclidea = sqrt((endLocation(1,1) - (estimatedPose(1,1)))^2 + (endLocation(1,2) - (estimatedPose(1,2)))^2);
    if (dist_euclidea < 0.2)
        disp('Navegaci?n completada');
        % desactivar motores enviando enable_motor = 0
        msg_enable_motor.Data = 0;
        send(pub_enable, msg_enable_motor);
        break;
    end
    
    % esperar al siguiente periodo de muestreo
    waitfor(r);
end