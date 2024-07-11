%% Importar datos de los motores
data_motors=readmatrix("motor_velocity_data_50pwm_md.txt");

off_time=data_motors(1,1); % Número a restar para adecuar el tiempo

% Restar 'n' de la primera columna
data_motors(:, 1) = data_motors(:, 1) - off_time;

vel_mr=[data_motors(:, 1),data_motors(:, 2)]

vel_ml=[data_motors(:, 1),data_motors(:, 3)]

% ts

ts= data_motors(3,1)-data_motors(2,1);

%% Entradas 
% Entrada motor derecho
%in_v_md=18.5*ones(size(data_motors));
in_pwm_mr=50*ones(size(data_motors(:,1)));

% Entrada motor izquierdo
%in_v_mi=18.5*ones(size(data_mi));
in_pwm_ml=50*ones(size(data_motors(:,1)));

%% Graficas de entradas y datos

figure(1)
plot(vel_mr(:,1),vel_mr(:,2),'r')
hold on
plot(vel_mr(:,1),in_pwm_mr,'k')

figure(2)
plot(vel_ml(:,1),vel_ml(:,2),'b')
hold on
plot(vel_ml(:,1),in_pwm_ml,'k')


%% Verificacion de la tf
% Motor Derecho
[d_w,d_t]=step(50*tfd_mr,10)
figure(3)
plot(d_t,d_w,'r')

% Motor Izquierdo
[d_w,d_t]=step(50*tfd_ml,10)
figure(4)
plot(d_t,d_w,'b')

%% PID_ Motor derecho
Kp=10.4198
Ki=28.9816
Kd=0.93657

Ts=0.1;

K1 = (2*Ts*Kp + Ki*Ts^2 + 2*Kd)/(2*Ts);
K2 = (Ki*Ts^2 - 2*Ts*Kp - 4*Kd)/(2*Ts);
K3 = (2*Kd)/(2*Ts);

tfd_mr

%% PID_ Motor Izquierdo
Kp=43.2543;
Ki=111.0993;
Kd=4.21;

Kp=11.9431
Ki=31.9201
kd=1.1171

Ts=0.1;

K1 = (2*Ts*Kp + Ki*Ts^2 + 2*Kd)/(2*Ts);
K2 = (Ki*Ts^2 - 2*Ts*Kp - 4*Kd)/(2*Ts);
K3 = (2*Kd)/(2*Ts);

tfd_mr
