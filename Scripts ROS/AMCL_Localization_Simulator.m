%% Localize amigobot using Monte Carlo Localization
%% Introduction
% This example demonstrates an application of the Monte Carlo Localization (MCL) 
% algorithm on amigobot in simulated STDR environment.
% 
% Monte Carlo Localization (MCL) is an algorithm to localize a robot using a 
% particle filter. The algorithm requires a known map and the task is to estimate 
% the pose (position and orientation) of the robot within the map based on the 
% motion and sensing of the robot. The algorithm starts with an initial belief 
% of the robot pose's probability distribution, which is represented by particles 
% distributed according to such belief. These particles are propagated following 
% the robot's motion model each time the robot's pose changes. Upon receiving 
% new sensor readings, each particle will evaluate its accuracy by checking how 
% likely it would receive such sensor readings at its current pose. Next the algorithm 
% will redistribute (resample) particles to bias particles that are more accurate. 
% Keep iterating these moving, sensing and resampling steps, and all particles 
% should converge to a single cluster near the true pose of robot if localization 
% is successful.
% 
% Adaptive Monte Carlo Localization (AMCL) is the variant of MCL implemented 
% in <docid:robotics_ref.bu31hfz-1 |robotics.MonteCarloLocalization|>. AMCL dynamically 
% adjusts the number of particles based on KL-distance [1] to ensure that the 
% particle distribution converge to the true distribution of robot state based 
% on all past sensor and motion measurements with high probability.
% 
% The current MATLAB� AMCL implementation can be applied to any differential 
% drive robot equipped with a range finder.
% 
% The STDR simulation must be running for this example to work._
% 
% Prerequisites: <docid:robotics_examples.example-GettingStartedWithGazeboExample 
% Getting Started With Gazebo Example>, <docid:robotics_examples.example-ROSTransformationTreeExample 
% Accessing the tf Transformation Tree in ROS Example>, <docid:robotics_examples.example-ROSPublishAndSubscribeExample 
% Exchanging Data with ROS Publishers and Subscribers Example>,
%% Connect to the AmigoBot in Simulation / Real
% In simulation: spawn a simulated AmigoBot inside an office environment in 
% a virtual machine In real: connect to AmigoBot In your MATLAB instance on the 
% host computer, run the following commands to initialize ROS global node in MATLAB 
% and connect to the ROS master in the virtual machine through its IP address 
% |ipaddress|. Replace '172.28.195.100' with the IP address of your robot in virtual 
% machine or real.

%rosinit(ipaddress);
%% Load the map of the simulation world /real environment
% Load a binary occupancy grid of the office environment in STDR. The map is 
% generated by driving AmigoBot inside the office environment. The map is constructed 
% using range-bearing readings from laser sensor and ground truth poses from STDR 
% or real. Refer to <docid:robotics_examples.example-MappingWithKnownPosesExample 
% Mapping With Known Poses> for a more detailed explanation.

load mapa_con_OnlineSLAM_Simulator.mat
show(map);
%% Setup the laser sensor model and amigobot motion model
% AmigoBot can be modeled as a differential drive robot and its motion can be 
% estimated using odometry data. The |Noise| property defines the uncertainty 
% in robot's rotational and linear motion. Increasing the |odometryModel.Noise| 
% property will allow more spread when propagating particles using odometry measurements. 
% Please refer to <docid:robotics_ref.bu359h6-1 |robotics.OdometryMotionModel|> 
% for property details.

odometryModel = robotics.OdometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];
%% 
% The sensor on amigobot is a simulated/real range finder converted from laser 
% readings. The likelihood field method is used to compute the probability of 
% perceiving a set of measurements by comparing the end points of the range finder 
% measurements to the occupancy map. If the end points match the occupied points 
% in occupancy map, the probability of perceiving such measurements is high. The 
% sensor model should be tuned to match the actual sensor property to achieve 
% better test results. The property |SensorLimits| defines the minimum and maximum 
% range of sensor readings. The property |Map| defines the occupancy map used 
% for computing likelihood field. Please refer to <docid:robotics_ref.bu31hrp-1 
% |robotics.LikelihoodFieldSensorModel|> for property details.

rangeFinderModel = robotics.LikelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0 8];
rangeFinderModel.Map = map;
%% 
% Set |rangeFinderModel.SensorPose| to the coordinate transform of the fixed 
% laser sensor with respect to the robot base. This is used to transform the laser 
% readings from laser frame to the base frame of AmigoBot. Please refer to <docid:robotics_examples.example-ROSTransformationTreeExample 
% docid:robotics_examples.example-ROSTransformationTreeExample> for details on 
% coordinate transformations.
% 
% Note that currently |SensorModel| is only compatible with sensors that are 
% fixed on the robot's frame, which means the sensor transform is constant.

% Query the Transformation Tree (tf tree) in ROS.
tftree = rostf;
%Obtener transformada entre los frames del robot y del sensor_laser
waitForTransform(tftree,'...','...');
sensorTransform = getTransform(tftree,'...', '...');

% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
    sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

% Setup the |SensorPose|, which includes the translation along base_link's
% +X, +Y direction in meters and rotation angle along base_link's +Z axis
% in radians.
rangeFinderModel.SensorPose = ...
    [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];
%% Interface for receiving sensor measurements from AmigoBot and sending velocity commands to AmigoBot.
% Create ROS subscribers for retrieving sensor and odometry measurements from 
% TurtleBot. sub_laser = rossubscriber('scan'); sub_odom = rossubscriber('odom');

% Hecho en ini_simulator.m o ini_amigobot.m
%% 
% Create ROS publisher for sending out velocity commands to AmigoBot. TurtleBot 
% subscribes to |'/mobile_base/commands/velocity'| for velocity commands. [velPub,velMsg] 
% = ... rospublisher('/mobile_base/commands/velocity','geometry_msgs/Twist');

% Hecho en ini_simulator.m o ini_amigobot.m
% Se usa pub_vel y msg_vel
%% Initialize AMCL object
% Instantiate an AMCL object |amcl|. See <docid:robotics_ref.bu31hfz-1 robotics.MonteCarloLocalization> 
% for more information on the class.

amcl = robotics.MonteCarloLocalization;
amcl.UseLidarScan = true;
%% 
% Assign the |MotionModel| and |SensorModel| properties in the |amcl| object.

amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;
%% 
% The particle filter only updates the particles when the robot's movement exceeds 
% the |UpdateThresholds|, which defines minimum displacement in [x, y, yaw] to 
% trigger filter update. This prevents too frequent updates due to sensor noise. 
% Particle resampling happens after the |amcl.ResamplingInterval| filter updates. 
% Using larger numbers leads to slower particle depletion at the price of slower 
% particle convergence as well.

amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;
%% Configure AMCL object for localization with initial pose estimate.
% |amcl.ParticleLimits| defines the lower and upper bound on the number of particles 
% that will be generated during the resampling process. Allowing more particles 
% to be generated may improve the chance of converging to the true robot pose, 
% but has an impact on computation speed and particles may take longer time or 
% even fail to converge. Please refer to the 'KL-D Sampling' section in [1] for 
% computing a reasonable bound value on the number of particles. Note that global 
% localization may need significantly more particles compared to localization 
% with an initial pose estimate. If the robot knows its initial pose with some 
% uncertainty, such additional information can help AMCL localize robots faster 
% with a less number of particles, i.e. you can use a smaller value of upper bound 
% in |amcl.ParticleLimits|.
% 
% Now set |amcl.GlobalLocalization| to false and provide an estimated initial 
% pose to AMCL. By doing so, AMCL holds the initial belief that robot's true pose 
% follows a Gaussian distribution with a mean equal to |amcl.InitialPose| and 
% a covariance matrix equal to |amcl.InitialCovariance|. Initial pose estimate 
% should be obtained according to your setup. This example helper retrieves the 
% robot's current true pose from Gazebo.
% 
% Please refer to section *Configure AMCL object for global localization* for 
% an example on using global localization.

amcl.ParticleLimits = ...;           % Minimum and maximum number of particles
amcl.GlobalLocalization = ....;      % global = true      local=false
amcl.InitialPose = ...;              % Initial pose of vehicle   
amcl.InitialCovariance = diag([1 1 1])*...; % Covariance of initial pose
%% Setup helper for visualization and driving AmigoBot.
% Setup ExampleHelperAMCLVisualization to plot the map and update robot's estimated 
% pose, particles, and laser scan readings on the map.

visualizationHelper = ExampleHelperAMCLVisualization(map);
%% Robot Motion
% Robot motion is essential for the AMCL algorithm. In this example, we drive 
% Amigobot randomly using the "rosrun teleop_twist_keyboard teleop_twist_keyboard.py" 

%class, which
% drives the robot inside the environment while avoiding obstacles using the
% |<docid:robotics_ref.buv7g7y robotics.VectorFieldHistogram>| class.
%wanderHelper = ...
%    ExampleHelperAMCLWanderer(sub_laser, sensorTransform, pub_vel, msg_vel);
%% Localization procedure
% The AMCL algorithm is updated with odometry and sensor readings at each time 
% step when the robot is moving around. Please allow a few seconds before particles 
% are initialized and plotted in the figure. In this example we will run |numUpdates| 
% AMCL updates. If the robot doesn't converge to the correct robot pose, consider 
% using a larger |numUpdates|.

i=1;
while (1)
    % Receive laser scan and odometry message.
    scan = receive(sub_laser);
    odompose = sub_odom.LatestMessage;
    
    %Crear objeto para almacenar el escaneo LiDAR 2-D
    scans = lidarScan(scan);
    
    % For sensors that are mounted upside down, you need to reverse the
    % order of scan angle readings using 'flip' function.
    
    % Compute robot's pose [x,y,yaw] from odometry message.
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];
    
    % Update estimated robot's pose and covariance using new odometry and
    % sensor readings.
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scans);
    
    % Drive robot to next pose.
    %wander(wanderHelper);
    
    % Plot the robot's estimated pose, particles and laser scans on the map.
    if isUpdated
        i = i + 1
        plotStep(visualizationHelper, amcl, estimatedPose, scans, i)
    end
    
end
%% Stop the AmigoBot and shutdown ROS in MATLAB

%stop(wanderHelper);
%rosshutdown
%% 
% _Copyright 2015-2016 The MathWorks, Inc._