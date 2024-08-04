%------------------------------------------------------------------
% Leitura (STM32F303K8) e impressao de dados 
%------------------------------------------------------------------
% Leituras do Acelerometro e do Giroscopio 
% 
% Fabi�n Barrera Prieto
% Mestrado em Sistemas Mecatr�nicos
% 09/03/2017
%------------------------------------------------------------------
% function [values] = DadosTest ()
    
close all;
clear all;
clc;

oldobj = instrfind;     %elimina resquicios presentes na porta serial
if not(isempty(oldobj)) 
    fclose(oldobj);     
    delete(oldobj);
end

if ~exist('s','var')
    s = serial('COM3','BaudRate',9600,'DataBits',8,'Parity','None','StopBits',1);
end
if strcmp(get(s,'status'),'closed')
    fopen(s);
end

SENSITIVITY_ACCEL = 2.0/32768.0;
SENSITIVITY_GYRO = 250.0/32768.0;

offset_accelx = 444.00;
offset_accely = -62.00;
offset_accelz = 16494.00;
offset_gyrox = 212.00;
offset_gyroy = -47.00;
offset_gyroz = 77.50;

disp('En sus marcas. Posicione el sensor en la posici�n inicial')
pause(); %Aguarda qualquer tecla.

disp('comienza')
fprintf(s,'H');
i = 1;
while(1) %Lee datos en un tiempo determinado en la stm32
    str{i} = fscanf(s);
    if(str{i}(1) == 'A')
        disp('termina')
        break;
    end
    i = i + 1;
end

fclose(s);
n = length(str)-1;

for i=1:n
    temp = cellfun(@str2num,strsplit(str{i},',')); %temp = eval(['[',str{i},'];']); %Selecciona un string para separarlo posteriormente en celdas
    if numel(temp) == 8 
        values(i,:) = temp;
    end
end
    
save EP2_Aula3 values

%-------------------------------------------------------------------------------------------------------------------------
%                                                       Figuras
%-------------------------------------------------------------------------------------------------------------------------
Nsamples = length(values);
dt = 0.01;
t = 0:dt:Nsamples*dt-dt;
%------------------------------------
% Aceler�metros RAW
%------------------------------------
figure;
plot(t, values(:,3)*SENSITIVITY_ACCEL, 'b') %ax
hold on
plot(t, values(:,4)*SENSITIVITY_ACCEL, 'r'); %ay
plot(t, values(:,5)*SENSITIVITY_ACCEL, 'g'); %az
title('Aceler�metros da MPU9250 sem calibra��o')
ylabel('acelera��o (g)')
xlabel('Tempo (segundos)')
legend('ax', 'ay', 'az', 'Location','northeast','Orientation','horizontal')
% %------------------------------------
% % Aceler�metros calibrados
% %------------------------------------
figure;
plot(t, (values(:,3)-offset_accelx)*SENSITIVITY_ACCEL, 'b') %ax
hold on
plot(t, (values(:,4)-offset_accely)*SENSITIVITY_ACCEL, 'r'); %ay
plot(t, (values(:,5)-(offset_accelz-(32768/2)))*SENSITIVITY_ACCEL, 'g'); %az
title('Aceler�metros da MPU9250 calibrados')
ylabel('acelera��o (g)')
xlabel('Tempo (segundos)')
legend('ax', 'ay', 'az', 'Location','northeast','Orientation','horizontal')

% %------------------------------------
% % Girosc�pios RAW
% %------------------------------------
figure;
plot(t, values(:,6)*SENSITIVITY_GYRO, 'b') %gx
hold on
plot(t, values(:,7)*SENSITIVITY_GYRO, 'r'); %gy
plot(t, values(:,8)*SENSITIVITY_GYRO, 'g'); %gz
title('Girosc�pios da MPU9250 sem calibra��o')
ylabel('Velocidade angular (�/s)')
xlabel('Tempo (segundos)')
legend('gx', 'gy', 'gz', 'Location','southeast','Orientation','horizontal')
% %------------------------------------
% % Girosc�pios calibrados
% %------------------------------------
figure;
plot(t, (values(:,6)-offset_gyrox)*SENSITIVITY_GYRO, 'b') %gx
hold on
plot(t, (values(:,7)-offset_gyroy)*SENSITIVITY_GYRO, 'r'); %gy
plot(t, (values(:,8)-offset_gyroz)*SENSITIVITY_GYRO, 'g'); %gz
title('Girosc�pios da MPU9250 calibrados')
ylabel('Velocidade angular (�/s)')
xlabel('Tempo (segundos)')
legend('gx', 'gy', 'gz', 'Location','northeast','Orientation','horizontal')