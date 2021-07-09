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

% espera sincronizar
vrep.simxSynchronousTrigger(clientID);

% ajusta o passo de tempo (deve ser o mesmo que esta ajustado no v-rep)
timeStep = 0.01;

%inicializa o tempo da simulacao.
timeSim=0;
% laco principal (fica ate nao haver mudanca significativa na posicao da
% massa)
omega=25;
fprintf('posicao inicial = %.7f\n',position(1,end));
while timeSim(1,end) < 3
    % aplica a forca senoidal
    [res retInts retFloats retStrings retBuffer]=vrep.simxCallScriptFunction(clientID,'myFunctions',vrep.sim_scripttype_childscript,'addForceTo',[h],[0,0,0,0,0,-1*sin(omega*timeSim(1,end))],[],[],vrep.simx_opmode_blocking);

    % espera sincronizar
    vrep.simxSynchronousTrigger(clientID);
    timeSim=[timeSim timeSim(1,end)+timeStep];
    
    % faz a leitura da posicao da massa
    [res,posTopPlate]=vrep.simxGetObjectPosition(clientID,h,-1,vrep.simx_opmode_oneshot_wait);
    
    %guarde a nova posicao posTopPlate aqui...
    
    %imprime na tela
    fprintf('t=%.3f -> pos = [%.5f,%.5f,%.5f]\n',timeSim(1,end),posTopPlate(1),posTopPlate(2),posTopPlate(3));
    
end

% chama o destrutor
vrep.delete(); % call the destructor!

fprintf('Simulacao terminada!\n');

