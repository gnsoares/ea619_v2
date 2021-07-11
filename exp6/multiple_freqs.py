import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import time

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

# ajusta o passo de tempo (deve ser o mesmo que esta ajustado no v-rep)
timeStep = 0.01

# inicializa o tempo da simulacao.
timeSim = np.arange(0, 3 + timeStep, timeStep)

#omega = np.logspace(0.9, 1.7, 100)
# 200g
#omega = np.linspace(25, 33, 8)
#omega = np.append(omega, np.linspace(30.5, 31.5, 4))
# 300g
omega = np.linspace(23, 30, 8)
omega = np.append(omega, np.linspace(24.5, 25.5, 4))

#
position = {}

# print(f'posicao inicial = {position[-1]:.7f}');

for omega_i in omega:

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

    # espera sincronizar
    sim.simxSynchronousTrigger(clientID)

    # pega a posicao inicial da massa
    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)
    time.sleep(2.5)

    print(f'simulation for w = {omega_i}')

    # coloca a posicao (tres coordenadas, mas apenas a z vai interessar) na
    # variavel position.
    position[omega_i] = []

    for time_i in timeSim:
        # aplica a forca senoidal
        sim.simxCallScriptFunction(clientID,
                                   'myFunctions',
                                   sim.sim_scripttype_childscript,
                                   'addForceTo',
                                   [h],
                                   [0, 0, 0, 0, 0, -1*np.sin(omega_i*time_i)],
                                   [],
                                   bytearray(),
                                   sim.simx_opmode_blocking)

        # espera sincronizar
        sim.simxSynchronousTrigger(clientID)

        # faz a leitura da posicao da massa
        posTopPlate = sim.simxGetObjectPosition(clientID, h, -1, sim.simx_opmode_oneshot_wait)[1]

        # guarde a nova posicao posTopPlate aqui...
        position[omega_i] = np.append(position[omega_i], posTopPlate[2])

    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
    is_running = True
    while is_running:
        error_code, ping_time = sim.simxGetPingTime(clientID)
        error_code, server_state = sim.simxGetInMessageInfo(clientID, sim.simx_headeroffset_server_state)
        is_running = server_state & 1

    # imprime na tela
    # print(f't={time:.3f} -> pos = [{posTopPlate[0]:.5f},{posTopPlate[1]:.5f},{posTopPlate[2]:.5f}]')

df = pd.DataFrame({
    'time': timeSim,
    **{f'pos_{omega_i}': position[omega_i] for omega_i in omega},
})
df.to_csv('sim_out_300g_new.csv', index=False)

print('Simulacao terminada!')
