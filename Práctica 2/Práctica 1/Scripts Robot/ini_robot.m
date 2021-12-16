% DECLARACIÓN DE SUBSCRIBERS

% Odometría
sub_odom = rossubscriber('/pose', 'nav_msgs/Odometry');

% Láser
sub_laser = rossubscriber('/scan', 'sensor_msgs/LaserScan');

% Sensores
sub_sonar0 = rossubscriber('/sonar_0', 'sensor_msgs/Range');
sub_sonar1 = rossubscriber('/sonar_1', 'sensor_msgs/Range');
sub_sonar2 = rossubscriber('/sonar_2', 'sensor_msgs/Range');
sub_sonar3 = rossubscriber('/sonar_3', 'sensor_msgs/Range');
sub_sonar4 = rossubscriber('/sonar_4', 'sensor_msgs/Range');
sub_sonar5 = rossubscriber('/sonar_5', 'sensor_msgs/Range');
sub_sonar6 = rossubscriber('/sonar_6', 'sensor_msgs/Range');
sub_sonar7 = rossubscriber('/sonar_7', 'sensor_msgs/Range');

% DECLARACIÓN DE PUBLISHERS

% Velocidad
pub_vel = rospublisher('/cmd_vel', 'geometry_msgs/Twist');

pub_enable = rospublisher('/cmd_motor_state', 'std_msgs/Int32')

% GENERACIÓN DE MENSAJES

msg_vel = rosmessage(pub_vel);

msg_enable_motor = rosmessage(pub_enable);

% Definimos la periodicidad del bucle
r = robotics.Rate(10);

% Nos aseguramos de recibir un mensaje relacionado con el robot
% while (strcmp(sub_odom.LatestMessage.ChildFrameId, 'robot0' ~= 1)
%     sub_odom.LastestMessage
% end

disp('Inicialización finalizada correctamente.');

