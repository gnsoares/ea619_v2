% inicializa a remoteAPI
vrep=remApi('remoteApi');
% por seguranca, fecha todas as conexoes abertas
vrep.simxFinish(-1);
%se conecta ao V-rep usando a porta 19997 (mesmo computador)
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

if clientID == -1
    fprintf('aconteceu algum problema na conexao com o servidor da remoteAPI!\n');
    return;
end

% escolhe a comunicacao sincrona
vrep.simxSynchronous(clientID,true);

% pega um handle para a massa superior (chamada de topPlate). Ele sera
% usado para aplicarmos a forca.
[err,h]=vrep.simxGetObjectHandle(clientID,'SpringDamper_topPlate',vrep.simx_opmode_oneshot_wait);

% pega a posicao inicial da massa
[res,posTopPlate]=vrep.simxGetObjectPosition(clientID,h,-1,vrep.simx_opmode_oneshot_wait);

% coloca a posicao (tres coordenadas, mas apenas a z vai interessar) na
% variavel position.
position = posTopPlate;

fprintf('tempo = %.3f posicao = [%.5f,%.5f,%.5f]\n',timeSim,posTopPlate(1),posTopPlate(2),posTopPlate(3));

%aplica a forma de -50 N na massa (a forca eh aplicada no centro de massa)
[res retInts retFloats retStrings retBuffer]=vrep.simxCallScriptFunction(clientID,'myFunctions',vrep.sim_scripttype_childscript,'addForceTo',[h],[0.0,0.0,0,0,0,-50],[],[],vrep.simx_opmode_blocking);

% espera sincronizar
vrep.simxSynchronousTrigger(clientID);

% ajusta o passo de tempo (deve ser o mesmo que esta ajustado no v-rep)
timeStepMax = 0.020;
timeStep = 0.0010;

%inicializa o tempo da simulacao.
timeSim=0;
% laco principal (fica ate nao haver mudanca significativa na posicao da
% massa)

while timeSim < 2.0
    
    % faz a leitura da posicao da massa
    [res,posTopPlate]=vrep.simxGetObjectPosition(clientID,h,-1,vrep.simx_opmode_oneshot_wait);
    
    %imprime na tela
    fprintf('tempo = %.3f posicao = [%.5f,%.5f,%.5f]\n',timeSim(1,end)+timeStep,posTopPlate(1),posTopPlate(2),posTopPlate(3));
    
    %guarde a nova posicao posTopPlate aqui...        
    
    timeSim=[timeSim timeSim(1,end)+timeStep];
    
    if timeSim(end) < timeStepMax
        [res retInts retFloats retStrings retBuffer]=vrep.simxCallScriptFunction(clientID,'myFunctions',vrep.sim_scripttype_childscript,'addForceTo',[h],[0.0,0.0,0,0,0,-50],[],[],vrep.simx_opmode_blocking);
        fprintf('apliquei força extra t=%.4f\n',timeSim(end));
    end
    % espera sincronizar
    vrep.simxSynchronousTrigger(clientID);
    
    % se a posicao z da massa no instante atual nao for muito diferente da posicao z do
    % instante de tempo anterior, poderia terminar a simulacao.Acrescente 
    % aqui uma condicao para terminar (nao eh obrigatorio)
    position = [position;posTopPlate];
end

% chama o destrutor
vrep.delete(); % call the destructor!

fprintf('Simulacao terminada!\n');
plot(timeSim,position(:,3));

save('data_001msA.mat','timeSim','position');

return;
t0=0.14;
p0=0.1670;
t1=0.34;
p1=0.1463;
r = 0.14073;
wd=2*pi*inv(t1-t0);

p0_a = p0-r;
p1_a = p1-r;
ld = inv(2*pi)*log(p0_a/p1_a);
ra=roots([1+ld, 0, -ld^2]);
xi=ra(1)
wn=wd/sqrt(1-xi*xi)


t0=0.16;
p0=0.1650;
t1=0.38;
p1=0.145;
r = 0.1383;
wd=2*pi*inv(t1-t0);

p0_a = p0-r;
p1_a = p1-r;
ld = inv(2*pi)*log(p0_a/p1_a);
ra=roots([1+ld, 0, -ld^2]);
xia=ra(1)
wna=wd/sqrt(1-xia*xia)

m=(0.05*wna^2)/(wn^2-wna^2)
k=wn*wn*m
b = 2*xi*sqrt(m*k)