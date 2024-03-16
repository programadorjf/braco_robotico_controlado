clear; clc;
%valores num�ricos que descrevem o manipulador
L1 = 0.5;
L2 = 0.5;
m1 = 4.6;
m2 = 2.3;
g = 9.8;
atrito = 10; %valor de atrico considerado (2)
%velocidade do movimento
T = 0.003; %passo para os incrementos do vetor tempo(0.01)
tempo = 0:T:5; %cria um vetor com valores de 0 at� 10, com passo T
% plot(tempo*180/pi,theta*180/pi) %direto na janela de comando para gerar
% grafico de posicao
thetad = [45*pi/180; 45*pi/180];
Kp1 = 90;
Kp2 = 20;

%condi��es iniciais
torque = [0 0]'; %cria uma matriz de zeros, o ap�strofo define que � matriz coluna: 2x1
theta = [0*pi/180 0]'; %cria a matriz inicial dos �lgulos theta, 2x1 (pi/2 pi/10)

%velocidade angular
v_ang = [0; 0]; %cria um vetor velocidade angular, 2x1, o ; avan�a a linha (0; 0)

%la�o para calcular os angulos
for i = 2:length(tempo) %varre todas as posi��es do vetor tempo, come�ando pela posi��o 2
  theta1 = theta(1, i-1); %coloca em theta1 o valor da linha 1, coluna i-1, do vetor theta
  theta2 = theta(2, i-1); %coloca em theta2 o valor da linha 2, coluna i-1, do vetor theta
  v_ang1 = v_ang(1, i-1); %coloca em v_ang1 o valor da linha 1, coluna i-1, do vetor v_ang
  v_ang2 = v_ang(2, i-1); %coloca em v_ang2 o valor da linha 2, coluna i-1, do vetor v_ang
  F = v_ang(:, i-1); %cria um vetor com os valores da velocidade angular atualizados
  
  e1 = thetad - [theta1; theta2];
  torque = [Kp1 Kp2; Kp2 Kp1]*e1;
  
  %matriz de massa do manipulador - M
  M11 = L2^2*m2+2*L1*L2*m2*cos(theta2)+L1^2*(m1+m2);
  M12 = L2^2*m2+L1*L2*m2*cos(theta2);
  M21 = L2^2*m2+L1*L2*m2*cos(theta2);
  M22 = L2^2*m2;
  M = [M11 M12; M21 M22];
  
  %vetor de termos centr�fugos - V
  V11 = -m2*L1*L2*sin(theta2)*v_ang2^2-2*m2*L1*L2*sin(theta2)*v_ang1*v_ang2;
  V21 = m2*L1*L2*sin(theta2)*v_ang1^2;
  V = [V11 V21]';
  
  %vetor de termos de gravidade - G
  G11 = m2*L2*g*cos(theta1+theta2)+(m1+m2)*L1*g*cos(theta1);
  G21 = m2*L2*g*cos(theta1+theta2);
  G = [G11 G21]';
  
  %acelera��o angular
  a_ang(:, i) = M^-1*(torque - V - G - atrito*F); % atualiza os valores de cada linha da coluna i
  
  %atualiza��o dos valores de v_ang e posi��o do bra�o, com a nova a_ang
  v_ang(:, i) = a_ang(:, i)*T + v_ang(:, i-1);
  theta(:, i) = v_ang(:, i)*T + theta(:, i-1) + 0.5*a_ang(:, i)*T^2;
end

%la�o para plotar os gr�ficos
for i = 1:length(tempo)
  %criando elo 1
  x_elo1 = L1*cos(theta(1,i));  
  y_elo1 = L1*sin(theta(1,i));
  %criando elo 2
  x_elo2 = L2*cos(theta(1,i)+theta(2,i))+x_elo1;  
  y_elo2 = L2*sin(theta(1,i)+theta(2,i))+y_elo1;
  pause(0); % aguarda (x) segundos, para deixar o gr�fico vis�vel
  clf(); % limpa a figura para a cria��o do novo gr�fico, clf = clear figure
  axis off; % desativa a exibi��o dos eixos x e y do gr�fico
  %plotando elo 1
  line([0 x_elo1], [0 y_elo1], 'Color', 'black');
  %plotando elo 2
  line([x_elo1 x_elo2], [y_elo1 y_elo2], 'Color', 'blue');
  
  xlim([-1, 1]); % limita o tamanho dos elos
  ylim([-1, 1]);
end
