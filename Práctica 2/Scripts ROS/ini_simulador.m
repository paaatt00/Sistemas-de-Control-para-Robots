% DECLARACIÓN DE SUBSCRIBERS

% Odometría
% sub_odom = rossubscriber('/robot0/odom', 'nav_msgs/Odometry');
sub_odom = rossubscriber('/robot0/local_odom', 'nav_msgs/Odometry'); %Odometria del robot

% Láser
sub_laser = rossubscriber('/robot0/laser_1', 'sensor_msgs/LaserScan');

% Sensores
sub_sonar0 = rossubscriber('/robot0/sonar_0', 'sensor_msgs/Range');
sub_sonar1 = rossubscriber('/robot0/sonar_1', 'sensor_msgs/Range');
sub_sonar2 = rossubscriber('/robot0/sonar_2', 'sensor_msgs/Range');
sub_sonar3 = rossubscriber('/robot0/sonar_3', 'sensor_msgs/Range');
sub_sonar4 = rossubscriber('/robot0/sonar_4', 'sensor_msgs/Range');
sub_sonar5 = rossubscriber('/robot0/sonar_5', 'sensor_msgs/Range');
sub_sonar6 = rossubscriber('/robot0/sonar_6', 'sensor_msgs/Range');
sub_sonar7 = rossubscriber('/robot0/sonar_7', 'sensor_msgs/Range');

% DECLARACIÓN DE PUBLISHERS

% Velocidad
pub_vel = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');

% GENERACIÓN DE MENSAJES

msg_vel = rosmessage(pub_vel);

% Definimos la periodicidad del bucle
r = robotics.Rate(10);

% Nos aseguramos de recibir un mensaje relacionado con el robot
% while (strcmp(sub_odom.LatestMessage.ChildFrameId, 'robot0' ~= 1)
%     sub_odom.LastestMessage
% end

disp('Inicialización finalizada correctamente.');

