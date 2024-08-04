%------------------------------------------------------------------
% Leitura (STM32F303K8) e impressao de dados 
%------------------------------------------------------------------
% Leituras do Acelerometro e do Giroscopio 
% 
% Fabián Barrera Prieto
% Mestrado em Sistemas Mecatrônicos
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

offset_accelx = -188.00;
offset_accely = -72.00;
offset_accelz = 16222.00;
offset_gyrox = 177.50;
offset_gyroy = -32.50;
offset_gyroz = -26.50;

disp('En sus marcas. Posicione el sensor en la posición inicial')
pause(); %Aguarda qualquer tecla.

disp('comienza')
fprintf(s,'H');
i = 1;
while(1) %Lee datos en un tiempo determinado en la stm32
    str{i} = fscanf(s);
    if(str{i}(2) == 'A')
        disp('termina')
        fclose(s);
        break;
    end
    i = i + 1;
end

fclose(s);
n = length(str)-1;

for i=1:n
    temp = cellfun(@str2num,strsplit(str{i},'\t')); %temp = eval(['[',str{i},'];']); %Selecciona un string para separarlo posteriormente en celdas
    if numel(temp) == 10 
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
% Acelerômetros RAW
%------------------------------------
figure;
plot(t, values(:,4)*SENSITIVITY_ACCEL, 'b') %ax
hold on
plot(t, values(:,5)*SENSITIVITY_ACCEL, 'r'); %ay
plot(t, values(:,6)*SENSITIVITY_ACCEL, 'g'); %az
title('Acelerômetros da MPU6050 sem calibração')
ylabel('aceleração (g)')
xlabel('Tempo (segundos)')
legend('ax', 'ay', 'az', 'Location','northeast','Orientation','horizontal')
% %------------------------------------
% % Acelerômetros calibrados
% %------------------------------------
figure;
plot(t, (values(:,4)-offset_accelx)*SENSITIVITY_ACCEL, 'b') %ax
hold on
plot(t, (values(:,5)-offset_accely)*SENSITIVITY_ACCEL, 'r'); %ay
plot(t, (values(:,6)-(offset_accelz-(32768/2)))*SENSITIVITY_ACCEL, 'g'); %az
title('Acelerômetros da MPU6050 calibrados')
ylabel('aceleração (g)')
xlabel('Tempo (segundos)')
legend('ax', 'ay', 'az', 'Location','northeast','Orientation','horizontal')

% %------------------------------------
% % Giroscópios RAW
% %------------------------------------
figure;
plot(t, values(:,7)*SENSITIVITY_GYRO, 'b') %gx
hold on
plot(t, values(:,8)*SENSITIVITY_GYRO, 'r'); %gy
plot(t, values(:,9)*SENSITIVITY_GYRO, 'g'); %gz
title('Giroscópios da MPU6050 sem calibração')
ylabel('Velocidade angular (°/s)')
xlabel('Tempo (segundos)')
legend('gx', 'gy', 'gz', 'Location','southeast','Orientation','horizontal')
% %------------------------------------
% % Giroscópios calibrados
% %------------------------------------
figure;
plot(t, (values(:,7)-offset_gyrox)*SENSITIVITY_GYRO, 'b') %gx
hold on
plot(t, (values(:,8)-offset_gyroy)*SENSITIVITY_GYRO, 'r'); %gy
plot(t, (values(:,9)-offset_gyroz)*SENSITIVITY_GYRO, 'g'); %gz
title('Giroscópios da MPU6050 calibrados')
ylabel('Velocidade angular (°/s)')
xlabel('Tempo (segundos)')
legend('gx', 'gy', 'gz', 'Location','northeast','Orientation','horizontal')