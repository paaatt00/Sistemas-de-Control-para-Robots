%%%%%%%%%%
% Programa para exportar un mapa a formato imagen, modificar la imagen con un editor y volver a generar el mapa modificado
%%%%%%%%%%

clear all
close all
clc

%%%%%%%%
% PASO 1
% CARGAR EL MAPA
load pasillo_profe_SLAM.mat 

fig = figure(1)
show(map)

axis('off')
title('')
ylabel('')
xlabel('')

%GUARDAR EL MAPA COMO IMAGEN
imwrite((1.-map.occupancyMatrix),'map_original.png')

axis('on')

%%%%%%%%
% PASO 2
% EDITAR EL MAPA CON UN EDITOR Y GUARDAR EL MAPA_IMAGEN LIMPIADO
% Editar con un editor de imagen (GIMP, PAINT ...) sin cambiar la
% resolucion, ni el tama�o
%
% Se guarda la imagen del editor limpiada en formato png (map_limpio.png)


%%%%%%%%
% PASO 3
% CARGAR LA IMAGEN LIMPIADA 

image = imread('map_original.png');

% Unknown areas (gray) should be removed and treated as free space. Create
% a logical matrix based on a threshold. Depending on your image, this value
% could be different. Occupied space should be set as 1 (white in image).


%%%%%%CUIDADO%%%%%%%%%%
% % Es posible que al trabajar con PAINT la imagen que se adquiera tenga 3
% componenetes (RGB). Eso se ve si el tama�o es image = A x L x 3  
% Si es as� hay que quedarse con s�lo una componente image=image(:,:,1)
%%%%%

%%%%%%%%
% PASO 4
% CONVERTIR IMAGEN EN MAPA (occupancygrid) Y VISUALIZARLO

image=image(:,:,1);  %Esta l�nea hay que comentarla si la imagen s�lo tiene una componenente A x L x 1   
imageNorm = double(image())/255;
imageOccupancy = 1 - imageNorm;

% Create OccupancyGrid object using adjusted map image.
map_modified = robotics.OccupancyGrid(imageOccupancy ,map.Resolution);  %pixels/metros(medidos antes de convertir a imagen)

%locate the new map in the same word coordinates than the original
map_modified.GridLocationInWorld = map.GridLocationInWorld

figure(2)
show(map_modified)


%%%%%%%%
% PASO 5
% GUARDAR MAPA LIMPIADO
save map_modified.mat map_modified






