import matplotlib.pyplot as plt
import numpy as np
try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteAnp.pi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

# por seguranca, fecha todas as conexoes abertas
sim.simxFinish(-1)

# se conecta ao V-rep usando a porta 19997 (mesmo computador)
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if clientID == -1:
    print('aconteceu algum problema na conexao com o servidor da remoteAPI!')

# escolhe a comunicacao sincrona
sim.simxSynchronous(clientID, True)

# pega um handle para a massa superior (chamada de topPlate). Ele sera
# usado para aplicarmos a forca.
err, h = sim.simxGetObjectHandle(clientID, 'SpringDamper_topPlate', sim.simx_opmode_oneshot_wait)

# pega a posicao inicial da massa
res, posTopPlate = sim.simxGetObjectPosition(clientID, h, -1, sim.simx_opmode_oneshot_wait)

# coloca a posicao (tres coordenadas, mas apenas a z vai interessar) na
# variavel position.
position = posTopPlate

# inicializa o tempo da simulacao.
timeSim = np.zeros(1)

PROGRESS = 'tempo = {time:.3f} posicao = [{pos0:.5f}, {pos1:.5f}, {pos2:.5f}]'

# print(PROGRESS.format(time=timeSim[0],
#                       pos0=posTopPlate[0],
#                       pos1=posTopPlate[1],
#                       pos2=posTopPlate[2]))

# aplica a forma de -50 N na massa (a forca eh aplicada no centro de massa)
(res,
 retInts,
 retFloats,
 retStrings,
 retBuffer) = sim.simxCallScriptFunction(clientID,
                                         'myFunctions',
                                         sim.sim_scripttype_childscript,
                                         'addForceTo',
                                         [h],
                                         [0, 0, 0, 0, 0, -50],
                                         [],
                                         bytearray(),
                                         sim.simx_opmode_blocking)

# espera sincronizar
sim.simxSynchronousTrigger(clientID)

# ajusta o passo de tempo (deve ser o mesmo que esta ajustado no v-rep)
timeStepMax = 0.020
timeStep = 0.010

# laco principal (fica ate nao haver mudanca significativa na posicao da
# massa)
while timeSim[-1] < 2.0:

    # faz a leitura da posicao da massa
    [res,posTopPlate] = sim.simxGetObjectPosition(clientID,h,-1,sim.simx_opmode_oneshot_wait)

    # imprime na tela
    # print(PROGRESS.format(time=timeSim[-1] + timeStep,
    #                       pos0=posTopPlate[0],
    #                       pos1=posTopPlate[1],
    #                       pos2=posTopPlate[2]))

    # guarde a nova posicao posTopPlate aqui...        
    timeSim = np.append(timeSim, timeSim[-1] + timeStep)

    if timeSim[-1] < timeStepMax:
        (res,
         retInts,
         retFloats,
         retStrings,
         retBuffer) = sim.simxCallScriptFunction(clientID,
                                                 'myFunctions',
                                                 sim.sim_scripttype_childscript,
                                                 'addForceTo',
                                                 [h],
                                                 [0, 0, 0, 0, 0, -50],
                                                 [],
                                                 bytearray(),
                                                 sim.simx_opmode_blocking)
        print(f'apliquei força extra t={timeSim[-1]:.4f}')

    # espera sincronizar
    sim.simxSynchronousTrigger(clientID)
    
    # se a posicao z da massa no instante atual nao for muito diferente da posicao z do
    # instante de tempo anterior, poderia terminar a simulacao.Acrescente 
    # aqui uma condicao para terminar (nao eh obrigatorio)
    position = np.append(np.atleast_2d(position), np.atleast_2d(posTopPlate), axis=0)

print('Simulacao terminada!')
plt.plot(timeSim, position[:,2])
plt.show()

# save('data_001msA.mat','timeSim','position')

t0 = 0.14
p0 = 0.1670
t1 = 0.34
p1 = 0.1463
r = 0.14073
wd = 2*np.pi/(t1-t0)

p0_a = p0-r
p1_a = p1-r
ld = np.log(p0_a/p1_a)/(2*np.pi)
ra = np.roots([1+ld, 0, -ld**2])
xi = ra[0]
wn = wd/np.sqrt(1-xi*xi)

t0 = 0.16
p0 = 0.1650
t1 = 0.38
p1 = 0.145
r = 0.1383
wd = 2*np.pi/(t1-t0)

p0_a = p0 - r
p1_a = p1 - r
ld = np.log(p0_a/p1_a)/(2*np.pi)
ra = np.roots([1+ld, 0, -ld**2])
xia = ra[0]
wna = wd/np.sqrt(1-xia*xia)

m = (0.05*wna**2)/(wn**2-wna**2)
k = wn*wn*m
b = 2*xi*np.sqrt(m*k)
