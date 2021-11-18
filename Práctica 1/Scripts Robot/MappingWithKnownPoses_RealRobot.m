%% Mapeado con posiciones conocidas
%% Introducci�n
% Antes de ejecutar este c�digo, debe haberse inicializado el sistema ROS con 
% el script 'conectar.m' y deben haberse creado las subscripciones necesarias 
% para leer los datos de la odometr�a y del l�ser (mediante el script 'ini_simulador.m' 
% o 'ini_amigobot.m')
% 
% Este ejemplo crea un mapa a partir de un sensor de distancia (l�ser) y posiciones 
% del robot tomadas como conocidas (odometr�a).
%% Lectura del �rbol de transformadas de ROS 
% Mediante la funci�n <docid:robotics_ref.bupf5_j_12 |rostf|>|, |podremos acceder 
% al �rbol de transformadas de ROS (/tf) para leer de ah� la posici�n del robot 
% en el momento exacto en que se adquieran los datos del l�ser.

tftree = rostf;

% Pause for a second for the transformation tree object to finish
% initialization.

pause(1);
%% Definici�n de un mapa (rejilla) vac�o
% A continuaci�n, crea una variable llamada 'map' (linea 7) utilizando la clase 
% <docid:robotics_ref.bvaw60t-1 |robotics.OccupancyGrid|> para contener un mapa 
% de hasta 25x25 metros, con una resoluci�n de 20 celdas por metro. Despu�s, escribe 
% un valor apropiado para la propiedad GridLocationInWorld (linea 8) de manera 
% que, en funci�n de la posici�n inicial del robot dentro del entorno, el mapa 
% a obtener quepa bien dentro de la rejilla: 

...
...
%% 
% Visualizamos el mapa en una figura:

figureHandle = figure('Name', 'Map');
axesHandle = axes('Parent', figureHandle);
mapHandle = show(map, 'Parent', axesHandle);
title(axesHandle, 'OccupancyGrid: Update 0');
%% Bucle de creaci�n del mapa
% El siguiente bucle ir� construyendo el mapa del entorno a medida que el robot 
% se mueva por el mismo (se debe teleoperar el robot mientras tanto). Rellena 
% los pasos indicados del bucle de control, teniendo en cuenta los siguientes 
% aspectos:
%% 
% * En primer lugar debe obtenerse el �ltimo mensaje del l�ser a trav�s del 
% subscriber correspondiente. Despu�s se debe utilizar <docid:robotics_ref.buqbijb 
% |getTransform|> , pasando el 'time stamp' del mensaje anterior, para obtener 
% la posici�n exacta del robot en el momento de la lectura del l�ser. Se tomar� 
% esa posici�n de la odometr�a, es decir de la relaci�n entre el frame del robot 
% y el frame de odometr�a (comprobar sus nombres en ROS mediante el comando 'rqt').
% * Obtener la posici�n y la orientaci�n a partir de la transformada anterior. 
% La orientaci�n del robot es el �ngulo yaw entorno al eje z. �ste se puede obtener 
% a partir del cuaternio de posici�n con la funci�n <docid:robotics_ref.buofjpw 
% |quat2eul|>.
% * Extraer las distancias y los �ngulos del mensaje del laser. En caso de que 
% se obtenga una medida infinita, sustituirla por la m�xima distancia.
% * Insertar la observaci�n del laser utilizando el m�todo <docid:robotics_ref.bvaw7o8-1 
% |insertRay|> de la variable |map|.
% * Visualizar el mapa cada 50 actualizaciones del mismo.

updateCounter = 1;
while(1)
    % Lee en msg_laser el �ltimo mensaje del l�ser
    msg_laser = sub_laser.LatestMessage;
    
    % Obtener la posici�n del robot en el momento de la lectura del laser.
    % Para ello, rellenar correctamente los valores 'target_frame' y
    % 'source_frame' con los nombres correctos de los frames en la
    % siguiente llamada
    pose = getTransform(tftree, '...', '...', msg_laser.Header.Stamp, 'Timeout', 2);
    
    % Convierte la posici�n del robot a vector [x y yaw]
    position = [pose.Transform.Translation.X, pose.Transform.Translation.Y];
    orientation =  quat2eul([pose.Transform.Rotation.W, pose.Transform.Rotation.X, ...
        pose.Transform.Rotation.Y, pose.Transform.Rotation.Z], 'ZYX');
    robotPose = [position, orientation(1)];
    
    % Extraer los rangos y los �ngulos del mensaje del l�ser.
    ranges = ...
    angles = ...
    ranges(isinf(ranges)) = 8;  %Eliminar datos infinitos
    
    % Insertar la medida del laser en el mapa utilizando 'insertRay',
    % pasandole los datos apropiados obtenidos anteriormente
    insertRay(...);  %Es un m�todo de la clase OccupancyGrid. Consular la ayuda.
    
    % Visualizamos el mapa cada 50 actualizaciones.
    if ~mod(updateCounter,50)
        mapHandle.CData = occupancyMatrix(map);
        title(axesHandle, ['OccupancyGrid: Update ' num2str(updateCounter)]);
    end
    updateCounter = updateCounter+1;
        
    % Wait for control rate to ensure 10 Hz rate
    waitfor(r);
end
%% Dibujar y salvar el mapa obtenido
% Para salir del bucle anterior, el usuario deber� finalizar el programa mediante 
% Ctr+C, por lo que se abandonar� el script. 
% 
% Para dibujar el mapa final obtenido y guardarlo en un fichero 'mi_mapa.mat', 
% ejecutar en linea de comandos las siguientes lineas:

show(map, 'Parent', axesHandle);
title(axesHandle, 'OccupancyGrid: Final Map');

save mi_mapa.mat map
%% 
%