import smbus2 as smbus
import RPi.GPIO as GPIO
from flask import Flask, render_template
from threading import Thread
import time
import math
import csv
import Adafruit_SSD1306
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont


# caracterisiticas del robot
class Robot:
    metodo_orientacion = 1
    coord_ini = (190, 190, 0)  # mm
    radio_robot = 150  # mm
    n_sensores = 14
    rango_sensor = 160  # mm
    radio_sensor = 110.2  # mm
    max_velocidad = 30  # mm/s
    c_atraccion = radio_robot/12
    max_velocidad_angular = math.pi/20  # rad/s
    c_atraccion_angular = 5*math.pi/180  # rad
    c_repulsion = 4000
    c_filtro_ema = 0.4
    dist_final = 2  # mm
    ang_final = 0.5*math.pi/180  # rad
    dist_estante = 1  # mm
    radio_rueda = 48  # mm
    l = 57  # mm
    w = 79.6  # mm


# Definir pines fin de carrera
GPIO_FCX = [5, 11]
GPIO_FCY = [13, 6]
# Definir tiempo de espera entre fases
t_espera = 0.003
# Definir pines de motores a pasos
GPIO_StepX = [14, 15, 18, 23]
GPIO_StepY = [24, 25, 8, 1]

# set GPIO Pins
GPIO_TRIGGER = 4
GPIO_ECHO = 17
GPIO_SELECT = [21, 20, 16, 12]

# Definir numeracion de pines
GPIO.setmode(GPIO.BCM)
# Deshabilitar avisos de GPIO
GPIO.setwarnings(False)

# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
for i in GPIO_SELECT:
    GPIO.setup(i, GPIO.OUT)
dist_medidas = [0] * Robot.n_sensores

# Definir pines de fin de carrera como entrada
GPIO.setup(7, GPIO.IN)
GPIO.setup(GPIO_FCX[0], GPIO.IN, GPIO.PUD_DOWN)
GPIO.setup(GPIO_FCX[1], GPIO.IN, GPIO.PUD_DOWN)
GPIO.setup(GPIO_FCY[0], GPIO.IN, GPIO.PUD_DOWN)
GPIO.setup(GPIO_FCY[1], GPIO.IN, GPIO.PUD_DOWN)
# Definir pines de motores a pasos como salidas
for pin in GPIO_StepX:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)
for pin in GPIO_StepY:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)
# Definir secuencia de fases motores
Seq = [[0]*4]*4
Seq[0] = [0, 0, 0, 1]
Seq[1] = [0, 0, 1, 0]
Seq[2] = [0, 1, 0, 0]
Seq[3] = [1, 0, 0, 0]

# Configurar display
# RST no se usa
RST = None
# 128x32 display con I2C
disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)
# Inicializacion libreria
disp.begin()
# Clear display.
disp.clear()
disp.display()
width = disp.width
height = disp.height
image = Image.new('1', (width, height))
draw = ImageDraw.Draw(image)
# Constantes
padding = -1
top = padding
bottom = height-padding
x = 0
# Font
font = ImageFont.truetype('/home/pi/Documents/Python/Arimo-Bold.ttf', 11)
# Borrar imagen
draw.rectangle((0, 0, width, height), outline=0, fill=0)
# Escirbir Texto
draw.text((x+10, top+10), "ESPERANDO PEDIDO...",  font=font, fill=255)
# Display
disp.image(image)
disp.display()

# I2C addresses para los Atmega de cada motor
addrMotor = [0x8, 0x9, 0x10, 0x11]
# Indica cual I2C bus del RPI /dev/ic2-1
bus = smbus.SMBus(1)

app = Flask(__name__)


# funcion para extraer informacion de documentos
def leer_docs():
    # límites del entrno
    with open('docs/limites.txt', 'r') as f:
        limites = [[float(num) for num in line.split(' ')] for line in f]
        limites = limites[0]
    # vectores y puntos de obstaculos conocidos
    with open('docs/vecsObstaculos.txt', 'r') as f:
        vecs_obs = [[float(num) for num in line.split(' ')] for line in f]
    with open('docs/pointsObstaculos.txt', 'r') as f:
        points_obs = [[float(num) for num in line.split(' ')] for line in f]
    # nodos del grafo de visibilidad
    with open('docs/nodos.txt', 'r') as f:
        nodos = [[float(num) for num in line.split(' ')] for line in f]
    # matriz de pesos del grafo de visibilidad
    with open('docs/matPesos.txt', 'r') as f:
        mat_pesos = [[float(num) for num in line.split(' ')] for line in f]
    # posiciones previas a agarrar el producto
    with open('docs/posPrev.txt', 'r') as f:
        pos_prev = [[float(num) for num in line.split(' ')] for line in f]
    # posicion para agarrar el producto
    with open('docs/posProd.txt', 'r') as f:
        pos_prod = [[float(num) for num in line.split(' ')] for line in f]
    items = []
    with open('docs/productos.csv', mode='r') as file:
        reader = csv.DictReader(file)
        for l in reader:
            items.append({})
            pose = []
            for key, value in l.items():
                if key == 'POSE_X':
                    pose.append(float(value))
                elif key == 'POSE_Y':
                    pose.append(float(value))
                elif key == 'ORIENTACION':
                    pose.append(float(value))
                    items[-1]['posicion'] = pose
                elif key == 'ID':
                    items[-1]['id'] = int(value)
                elif key == 'NOMBRE':
                    items[-1]['name'] = value
                elif key == 'PRECIO':
                    items[-1]['price'] = float(value)
                elif key == 'NIVEL':
                    items[-1]['nivel'] = int(value)
                elif key == 'PASILLO':
                    items[-1]['pasillo'] = int(value)
                    items[-1]['state'] = False
    return limites, vecs_obs, points_obs, nodos, mat_pesos, pos_prev, pos_prod, items


# Funcion para apagar todas las bobinas del motor
def apagar_motores():
    for pin in GPIO_StepX:
        GPIO.output(pin, False)
    for pin in GPIO_StepY:
        GPIO.output(pin, False)


# Funcion movimiento a posicion inical en X
def pos_ini_x():
    n = 0
    while not GPIO.input(GPIO_FCX[0]):
        for pin in range(0, 4):
            GPIO.output(GPIO_StepX[pin], Seq[n % 4][pin])
        time.sleep(t_espera)
        n = n - 1
    apagar_motores()


# Funcion movimiento a posicion final en X
def pos_fin_x():
    n = 0
    while not GPIO.input(GPIO_FCX[1]):
        for pin in range(0, 4):
            GPIO.output(GPIO_StepX[pin], Seq[n % 4][pin])
        time.sleep(t_espera)
        n = n + 1
    apagar_motores()


# Funcion movimiento a posicion inicial en Y
def pos_ini_y():
    n = 0
    while not GPIO.input(GPIO_FCY[0]):
        for pin in range(0, 4):
            GPIO.output(GPIO_StepY[pin], Seq[n % 4][pin])
        time.sleep(t_espera)
        n = n - 1
    apagar_motores()


# Funcion movimiento a posicion final en Y
def pos_fin_y():
    n = 0
    while not GPIO.input(GPIO_FCY[1]):
        for pin in range(0, 4):
            GPIO.output(GPIO_StepY[pin], Seq[n % 4][pin])
        time.sleep(t_espera)
        n = n + 1
    apagar_motores()


# Funcion secuencia mecanismo de agarre
def secuencia():
    print('-> Mecanismo en movimiento!')
    # Borrar imagen
    draw.rectangle((0, 0, width, height), outline=0, fill=0)
    # Escirbir Texto
    draw.text((x + 10, top + 5), "MECANISMO EN", font=font, fill=255)
    draw.text((x + 10, top + 15), "MOVIMIENTO", font=font, fill=255)
    # Display
    disp.image(image)
    disp.display()
    # Display
    disp.image(image)
    disp.display()

    pos_fin_x()
    pos_fin_y()
    pos_ini_x()
    pos_ini_y()
    apagar_motores()


# costo de pedido para app web
costo_total = 0
# condicion de programa del robot corriendo
running = 0
# calculo vectores de direccion de sensores
vecs_sensores = [[-math.cos(2 * math.pi / Robot.n_sensores * -i), -math.sin(2 * math.pi / Robot.n_sensores * -i)] for i in range(0, Robot.n_sensores)]
giro_ruedas = [0] * 4
# obtener variables precalculadas de documentos
limites, vecs_obs, points_obs, nodos, mat_pesos, pos_prev, pos_prod, items = leer_docs()
# Secuencia a posicion inicial mecanismo
pos_ini_y()
pos_ini_x()
apagar_motores()

# pagina principal
@app.route('/')
@app.route('/home')
def home_page():
    return render_template('home.html')


# pagina de mercado
@app.route('/market')
def market_page():
    return render_template('market.html',items=items)


# pagina de redireccion para añadir/remover un producto del mercado
@app.route("/mercado/<item>/<action>")
def action(item, action):
    global items, costo_total
    item = int(item)-1
    # se cambia estado de compra del producto segun la accion añadir/remover y se actualiza el costo total
    if action == 'add':
        items[item]['state'] = True
        costo_total += items[item]['price']
    else:
        items[item]['state'] = False
        costo_total -= items[item]['price']
    return render_template('market.html', items=items)


# pagina para remover producto de carrito de compras
@app.route("/carrito/<item>/<action>")
def remover(item, action):
    global items, costo_total
    item = int(item)-1
    # se actualiza estado de costo de producto y precio de pedido
    if action == 'remove':
        items[item]['state'] = False
        costo_total -= items[item]['price']
    return render_template('compras.html', items=items, costo_total=costo_total)


# pagina para boton de acerca de
@app.route('/acerca')
def acerca_page():
    return render_template('acerca_de.html')


# pagina de carrito o compras hechas
@app.route('/compras')
def compras_page():
    global items, costo_total
    return render_template('compras.html', items=items, costo_total=costo_total)


# pagina de checkout, espera mientras robot se mueve
@app.route('/checkout')
def checkout_page():
    global running
    # inicializacion de thread para programa del robot
    thread = Thread(target=programa)
    # running=0 robot no moviendos, running=1 robot moviendose, running=2 robot termino secuencia
    if running == 0:
        # programa principal del robot
        thread.start()
        # redireccion a pagina de checkout
        return render_template('running.html')
    elif running == 1:
        # redireccion a pagina de checkout
        return render_template('running.html')
    elif running == 2:
        running = 0
        # redireccion a pagina de inicio
        return render_template('home.html')


# generacion de metas segun productos pedidos
def gen_goals(prod_pedidos, pos_prev):
    goals = []
    for pedidos in prod_pedidos:
        goals.append(pos_prev[pedidos][0:2])
    return goals


# módulo del producto cruz 2D
def mod_prod_cruz(vec1, vec2):
    mod = vec1[0]*vec2[1]-vec1[1]*vec2[0]
    return mod


# deteccion de colision
def vert_colision(nodo1, nodo2, vecs_obs, points_obs):
    # adaptacion de algoritmo de Ronald Goldman para deteccion de interseccion de dos segmentos de linea
    # vector de arista de grafo a analizar
    vec_vertice = [nodo2[0]-nodo1[0], nodo2[1]-nodo1[1]]
    # se analiza colisiones en todos los obstaculos
    for i in range(0, int(len(vecs_obs)/6)):
        # generacion de vectores de aristas de cada obstaculos
        vecs = vecs_obs[i*6:i*6+6]
        # generacion de coordenadas iniciales de vector de cada obstaculo
        verts = points_obs[i*6:i*6+6]
        # verificacion de arista de grafo no cruza por dentro de obstaculos
        count_check = 0
        for j in range(0, len(vecs)):
            if ((vecs[j][0] == nodo1[0] and vecs[j][1] == nodo1[1]) or (vecs[j][0] == nodo2[0] and vecs[j][1] == nodo2[1])) and (nodo1[0] != nodo2[0] and nodo1[1] != nodo2[1]):
                count_check += 1
            # algoritmo de Ronald Goldman para deteccion de interseccion entre segmentos de linea
            a = [verts[j][0]-nodo1[0], verts[j][1]-nodo1[1]]
            b = mod_prod_cruz(vec_vertice, vecs[j])
            if b != 0:
                t = mod_prod_cruz(a, vecs[j])/b
                u = mod_prod_cruz(a, vec_vertice)/b
                if 0 < t < 1 and 0 < u < 1:
                    check = True
                    return check
        if count_check >= 2:
            check = True
            return check
    check = False
    return check


# generacion de nuevo grafo de visibilidad
def nuevo_grafo(goals, nodos, mat_pesos, vecs_obs, points_obs):
    # crear nueva lista de nodos
    nodos_new = [[Robot.coord_ini[0], Robot.coord_ini[1]]]
    for i in goals:
        nodos_new.append(i)
    for i in nodos:
        nodos_new.append(i)
    n_nodos_new = len(nodos_new)
    n_nodos_agregados = len(goals)+1
    # crear nueva matriz de pesos de grafo
    mat_pesos_new = [[0.0 for j in range(0, n_nodos_new)] for i in range(0, n_nodos_new)]
    # agregar matriz de pesos base a la nueva matriz
    for i in range(n_nodos_agregados, n_nodos_new):
        for j in range(n_nodos_agregados, n_nodos_new):
            mat_pesos_new[i][j] = mat_pesos[i-n_nodos_agregados][j-n_nodos_agregados]
    # analizar conectividad de nuevos nodos y agregar a matriz de pesos
    for i in range(0, n_nodos_agregados):
        for j in range(i+1, n_nodos_new):
            if not vert_colision(nodos_new[i], nodos_new[j], vecs_obs, points_obs):
                mat_pesos_new[i][j] = math.hypot(nodos_new[i][0]-nodos_new[j][0], nodos_new[i][1]-nodos_new[j][1])
                mat_pesos_new[j][i] = mat_pesos_new[i][j]
    return nodos_new, mat_pesos_new


# algoritmo de busqueda en grafos A*
def a_star(mat_pesos, nodos, ini, fin):
    n_nodos = len(nodos)
    # calculo de heuristica como la distancia euclidiana hasta la meta
    heuristica = [math.hypot(nodos[i][0]-nodos[fin][0], nodos[i][1]-nodos[fin][1]) for i in range(0, n_nodos)]
    # inicializacion de nodos revisados y no revisados
    nodos_revisados = []
    nodos_no_revisados = [ini]
    # inicializacion de lista de costos minimos para cada nodo
    min_costos = [math.inf]*n_nodos
    min_costos[ini] = 0.0
    # inicializacion de arbol del grafo
    arbol = [None]*n_nodos
    # inicializacion de costos estimados para cada nodo
    costo_estimado = [math.inf]*n_nodos
    # bucle se ejecuta hasta que no haya nodos vecinos a arbol para analizar o hasta que se llego a la meta
    while len(nodos_no_revisados) > 0:
        # se elige nodo en la lista nodos no revisados
        current = nodos_no_revisados[0]
        # se agrega nodo a lista de revisados
        nodos_revisados.append(current)
        # se elimina nodo de lista de no revisados
        nodos_no_revisados.remove(current)
        # si el nodo es igual a la meta se termina bucle
        if current == fin:
            break
        # se analiza todos los nodos
        for vecino in range(0, n_nodos):
            # se chequea cual nodo es vecino y si no se encuentra en lista de revisados
            if (mat_pesos[current][vecino] != 0 or (nodos[current][0] == nodos[vecino][0] and nodos[current][1] == nodos[vecino][1])) and vecino not in nodos_revisados:
                # se almacena el costo temporal para ir a nodo vecino
                costo_temporal = min_costos[current]+mat_pesos[current][vecino]
                # si este costo es menor al costo almacenado previamente, se reemplaza valores
                if costo_temporal < min_costos[vecino]:
                    min_costos[vecino] = costo_temporal
                    arbol[vecino] = current
                    # se agrega nodo vecino a la lista de no revisados en orden segun el costo estimado
                    costo_estimado[vecino] = min_costos[vecino]+heuristica[vecino]
                    n_nodos_no_revisados = len(nodos_no_revisados)
                    pos = 0
                    while pos < n_nodos_no_revisados and costo_estimado[vecino] > costo_estimado[nodos_no_revisados[pos]]:
                        pos += 1
                    nodos_no_revisados.insert(pos, vecino)
    # recursividad para hallar el camino de nodo inicial a final segun el arbol
    camino = [fin]
    while camino[0] != ini:
        camino.insert(0, arbol[camino[0]])
    camino.remove(ini)
    costo_total = min_costos[fin]
    return camino, costo_total


# generacion de trayectoria
def trayectoria(goals, mat_pesos_new, nodos_new):
    n_goals = len(goals)
    # nodo inicial de trayectoria
    ini = 0
    # camino comienza en nodo 1
    camino = [0]
    # bucle analiza todas las metas para generar la trayectoria
    goals_orden = []
    goals_no_revisados = [i for i in range(1, n_goals+1)]
    while len(goals_no_revisados) > 0:
        # meta mas cercana comienza con costo infinito
        costo = math.inf
        # se analiza todas las metas para obtener el costo más bajo
        for i in range(0, n_goals):
            # algoritmo de busqueda en grafos A*
            camino_temp, costo_temp = a_star(mat_pesos_new, nodos_new, ini, goals_no_revisados[i])
            # si el costo calculado es menor al costo previo almacenado se reemplaza como nueva mejor meta
            if costo_temp < costo:
                costo = costo_temp
                goal_temp = goals_no_revisados[i]
                camino_temp_aux = camino_temp
        # se agrega el mejor camino encontrado al camino general
        for i in camino_temp_aux:
            camino.append(i)
        # se elimina la meta de las metas no revisadas
        goals_no_revisados.remove(goal_temp)
        # se agrega la meta en la lista del orden de las metas
        goals_orden.append(goal_temp)
        # nodo incial se convierte en la meta seleccionada
        ini = goal_temp
        n_goals -= 1
    # cálculo del camino para retornar a las coordenadas de la posicion inicial
    camino_temp, costo_temp = a_star(mat_pesos_new, nodos_new, ini, 0)
    for i in camino_temp:
        camino.append(i)
    goals_orden.append(0)
    return camino, goals_orden


# actualizacion de posicion por odometria
def odometria(pose, v_control):
    giro_ruedas[0] = read_i2c(addrMotor[0])
    giro_ruedas[1] = read_i2c(addrMotor[1])
    giro_ruedas[2] = read_i2c(addrMotor[2])
    giro_ruedas[3] = read_i2c(addrMotor[3])
    giro_robot = Robot.radio_rueda/4*(-giro_ruedas[0]+giro_ruedas[1]+giro_ruedas[2]-giro_ruedas[3])/(Robot.l+Robot.w+1.738702903)
    a = giro_ruedas[0]+giro_ruedas[1]+giro_ruedas[2]+giro_ruedas[3]
    b = -giro_ruedas[0]+giro_ruedas[1]-giro_ruedas[2]+giro_ruedas[3]
    if giro_robot == 0:
        d_x = Robot.radio_rueda/4*a*1
        d_y = Robot.radio_rueda/4*b*0.95
    else:
        d_x = Robot.radio_rueda/4*(math.sin(giro_robot)*a+(math.cos(giro_robot)-1)*b)/giro_robot*1
        d_y = Robot.radio_rueda/4*((1-math.cos(giro_robot))*a+math.sin(giro_robot)*b)/giro_robot*0.95
    pose[0] = pose[0]+math.cos(pose[2])*d_x-math.sin(pose[2])*d_y
    pose[1] = pose[1]+math.sin(pose[2])*d_x+math.cos(pose[2])*d_y
    pose[2] = pose[2]+giro_robot
    if pose[2] > math.pi:
        pose[2] = pose[2] - 2*math.pi
    pose_print = [pose[0], pose[1], pose[2]*180/math.pi]
    print("pose " + str([round(i, 2) for i in pose_print]))
    return pose


# velocidad de giro
def atraccion_giro(orientacion_deseada, orientacion_actual):
    # calcular error en orientacion
    error_orientacion = orientacion_deseada-orientacion_actual
    if abs(error_orientacion) > math.pi:
        s = error_orientacion/abs(error_orientacion)
        giro = -error_orientacion+s*math.pi
    else:
        giro = error_orientacion
    # calculo modulo velocidad de giro
    vel_angular = Robot.max_velocidad_angular * (1 - math.exp(-abs(giro) / Robot.c_atraccion_angular))
    # determinacion de sentido de giro
    if giro >= 0:
        return vel_angular
    else:
        return -vel_angular


# velocidad lineal
def atraccion(goal, pose, orientacion_deseada):
    # calculo velocidad lineal de atraccion
    v_atraccion = Robot.max_velocidad * (1 - math.exp(-math.hypot(goal[1] - pose[1], goal[0] - pose[0]) / Robot.c_atraccion))
    # angulo hacia meta
    theta = math.atan2(goal[1]-pose[1], goal[0]-pose[0])
    # calculo velocidad de giro para orientacion deseada
    vel_giro = atraccion_giro(orientacion_deseada, pose[2])
    return [math.cos(theta)*v_atraccion, math.sin(theta)*v_atraccion, vel_giro]


# velocidad sin considerar repulsion
def calc_velocidad_no_rep(meta, pose):
    # calculo velocidad de control = atraccion
    v_control = atraccion(meta[0:2], pose, meta[2])
    # verificar que velocidad de control no supere velocidad maxima del robot
    mod_v_control = math.hypot(v_control[0], v_control[1])
    if mod_v_control > Robot.max_velocidad:
        v_control = [Robot.max_velocidad * v_control[0] / mod_v_control,
                     Robot.max_velocidad * v_control[1] / mod_v_control, v_control[2]]
    return v_control


# filtro EMA
def ema_filtro(valores_filtrados, valores):
    # filtro EMA - Exponential Moving Average
    for i in range(0, Robot.n_sensores):
        valores_filtrados[i] = Robot.c_filtro_ema * valores[i] + (1 - Robot.c_filtro_ema) * valores_filtrados[i]
    return valores_filtrados


def sensor_select(sensor):
    # set MUX selection
    if sensor == 1:
        GPIO.output(GPIO_SELECT[0], False)
        GPIO.output(GPIO_SELECT[1], False)
        GPIO.output(GPIO_SELECT[2], False)
        GPIO.output(GPIO_SELECT[3], False)
    elif sensor == 2:
        GPIO.output(GPIO_SELECT[0], True)
        GPIO.output(GPIO_SELECT[1], False)
        GPIO.output(GPIO_SELECT[2], False)
        GPIO.output(GPIO_SELECT[3], False)
    elif sensor == 3:
        GPIO.output(GPIO_SELECT[0], False)
        GPIO.output(GPIO_SELECT[1], True)
        GPIO.output(GPIO_SELECT[2], False)
        GPIO.output(GPIO_SELECT[3], False)
    elif sensor == 4:
        GPIO.output(GPIO_SELECT[0], True)
        GPIO.output(GPIO_SELECT[1], True)
        GPIO.output(GPIO_SELECT[2], False)
        GPIO.output(GPIO_SELECT[3], False)
    elif sensor == 5:
        GPIO.output(GPIO_SELECT[0], False)
        GPIO.output(GPIO_SELECT[1], False)
        GPIO.output(GPIO_SELECT[2], True)
        GPIO.output(GPIO_SELECT[3], False)
    elif sensor == 6:
        GPIO.output(GPIO_SELECT[0], True)
        GPIO.output(GPIO_SELECT[1], False)
        GPIO.output(GPIO_SELECT[2], True)
        GPIO.output(GPIO_SELECT[3], False)
    elif sensor == 7:
        GPIO.output(GPIO_SELECT[0], False)
        GPIO.output(GPIO_SELECT[1], True)
        GPIO.output(GPIO_SELECT[2], True)
        GPIO.output(GPIO_SELECT[3], False)
    elif sensor == 8:
        GPIO.output(GPIO_SELECT[0], True)
        GPIO.output(GPIO_SELECT[1], True)
        GPIO.output(GPIO_SELECT[2], True)
        GPIO.output(GPIO_SELECT[3], False)
    elif sensor == 9:
        GPIO.output(GPIO_SELECT[0], False)
        GPIO.output(GPIO_SELECT[1], False)
        GPIO.output(GPIO_SELECT[2], False)
        GPIO.output(GPIO_SELECT[3], True)
    elif sensor == 10:
        GPIO.output(GPIO_SELECT[0], True)
        GPIO.output(GPIO_SELECT[1], False)
        GPIO.output(GPIO_SELECT[2], False)
        GPIO.output(GPIO_SELECT[3], True)
    elif sensor == 11:
        GPIO.output(GPIO_SELECT[0], False)
        GPIO.output(GPIO_SELECT[1], True)
        GPIO.output(GPIO_SELECT[2], False)
        GPIO.output(GPIO_SELECT[3], True)
    elif sensor == 12:
        GPIO.output(GPIO_SELECT[0], True)
        GPIO.output(GPIO_SELECT[1], True)
        GPIO.output(GPIO_SELECT[2], False)
        GPIO.output(GPIO_SELECT[3], True)
    elif sensor == 13:
        GPIO.output(GPIO_SELECT[0], False)
        GPIO.output(GPIO_SELECT[1], False)
        GPIO.output(GPIO_SELECT[2], True)
        GPIO.output(GPIO_SELECT[3], True)
    elif sensor == 14:
        GPIO.output(GPIO_SELECT[0], True)
        GPIO.output(GPIO_SELECT[1], False)
        GPIO.output(GPIO_SELECT[2], True)
        GPIO.output(GPIO_SELECT[3], True)


# obtencion distancia sensores
def ultrasonicos(dist_filtradas):
    # lectura de sensores
    for i in range(1, Robot.n_sensores + 1):
        check = False
        while not check:
            distancia = 0
            check = True
            counter = 0
            # set MUX selection
            sensor_select(i)
            # set Trigger to HIGH
            GPIO.output(GPIO_TRIGGER, True)
            # set Trigger after 0.01ms to LOW
            time.sleep(0.00001)
            GPIO.output(GPIO_TRIGGER, False)
            time.sleep(0.000002)  # this time delay to avoid interference between trigger and echo

            StopTime = time.time()

            while GPIO.input(GPIO_ECHO) == 0:
                # pass
                counter += 1
                if counter == 5000:
                    check = False
                    break
            StartTime = time.time()

            # save time of arrival
            while GPIO.input(GPIO_ECHO) == 1:
                StopTime = time.time()

            # time difference between start and arrival
            TimeElapsed = StopTime - StartTime
            if (TimeElapsed < 0):
                check = False
            # multiply with the sonic speed (34300 cm/s)
            # and divide by 2, because there and back
            # distancia = (TimeElapsed * 34300) / 2
            distancia = (TimeElapsed * 343000) / 2  # mm
            # if not check:
            #    print(str(sensor)+" "+str(check))
            # return [check, distance]
            time.sleep(0.002)
        distancia = distancia + Robot.radio_sensor
        if distancia <= Robot.radio_robot:
            distancia = Robot.radio_robot+1
        dist_medidas[i - 1] = distancia
    # dist_medidas = [Robot.rango_sensor - 1] * Robot.n_sensores
    # retornar datos filtrados
    return ema_filtro(dist_filtradas, dist_medidas)


# calculo velocidad desplazamiento
def calc_velocidad(meta, pose, orientacion_deseada, dist_filtradas):
    # calcular velocidad de atraccion
    v_atraccion = atraccion(meta, pose, orientacion_deseada)
    # obtener distancias a obstaculos
    dist_filtradas = ultrasonicos(dist_filtradas)
    # calcular velocidad de repulsion
    v_repulsion = repulsion(dist_filtradas, pose[2])
    # calcular velocidad de control = atraccion + repulsion
    v_control = [v_atraccion[0]+v_repulsion[0], v_atraccion[1]+v_repulsion[1], v_atraccion[2]]
    # verificar que velocidad de control no supere velocidad maxima del robot
    mod_v_control = math.hypot(v_control[0], v_control[1])
    if mod_v_control > Robot.max_velocidad:
        v_control = [Robot.max_velocidad * v_control[0] / mod_v_control, Robot.max_velocidad * v_control[1] / mod_v_control, v_control[2]]
    return v_control, dist_filtradas


# conversion velocidad robot a velocidad ruedas
def velocidad_2_rpm(pose, v_control):
    a = Robot.l + Robot.w
    b = math.sin(pose[2])+math.cos(pose[2])
    c = math.sin(pose[2])-math.cos(pose[2])
    d = 60/(2*math.pi*Robot.radio_rueda)
    # conversion_2_rpm = 60/(2*math.pi)
    return [d*(v_control[2]*(-a)+v_control[0]*b+v_control[1]*c),
            d*(v_control[2]*a+v_control[0]*(-c)+v_control[1]*b),
            d*(v_control[2]*a+v_control[0]*b+v_control[1]*c),
            d*(v_control[2]*(-a)+v_control[0]*(-c)+v_control[1]*b)]


def velocidad(pose, v_control):
    rpm = velocidad_2_rpm(pose, v_control)
    write_i2c(addrMotor[0], rpm[0])
    write_i2c(addrMotor[1], rpm[1])
    write_i2c(addrMotor[2], rpm[2])
    write_i2c(addrMotor[3], rpm[3])


# calculo movimiento con repulsion
def movimiento_con_repulsion(pose, meta_prev, orientacion, dist_filtradas):
    fin = True
    # se inicializa velocidad de control
    v_control = [0, 0, 0]
    velocidad(pose, v_control)
    while fin:
        # se actualiza pose por odometria
        pose = odometria(pose, v_control)
        # condicion de terminacion
        if math.hypot(meta_prev[0] - pose[0], meta_prev[1] - pose[1]) < Robot.dist_estante and abs(orientacion - pose[2]) < Robot.ang_final:
            v_control = [0, 0, 0]
            velocidad(pose, v_control)
            break
        # calculo de velocidad de control
        v_control, dist_filtradas = calc_velocidad(meta_prev[0:2], pose, orientacion, dist_filtradas)
        velocidad(pose, v_control)
    return pose, dist_filtradas


# calculo movimiento sin repulsion
def movimiento_no_repulsion(pose, meta):
    fin = True
    # se inicializa velocidad de control
    v_control = [0, 0, 0]
    velocidad(pose, v_control)
    while fin:
        # se actualiza pose por odometria
        pose = odometria(pose, v_control)
        # condicion de terminacion
        if math.hypot(meta[0] - pose[0], meta[1] - pose[1]) < Robot.dist_estante and abs(meta[2] - pose[2]) < Robot.ang_final:
            v_control = [0, 0, 0]
            velocidad(pose, v_control)
            break
        # calculo de velocidad de control sin considerar repulsion de sensores
        v_control = calc_velocidad_no_rep(meta, pose)
        velocidad(pose, v_control)
    return pose


# movimiento cuando robot debe moverse a estante
def movimiento_estante(meta, meta_prev, pose, dist_filtradas):
    # giro del robot hacia orientacion deseada segun metodo de orientacion
    if Robot.metodo_orientacion != 1:
        print('Orientando')
        pose, dist_filtradas = movimiento_con_repulsion(pose, meta_prev, meta[2], dist_filtradas)
    pose = movimiento_no_repulsion(pose, meta)
    print('--> Tomando Producto <--')
    secuencia()
    if Robot.metodo_orientacion != 1:
        pose, dist_filtradas = movimiento_con_repulsion(pose, meta_prev, pose[2], dist_filtradas)
        pose, dist_filtradas = movimiento_con_repulsion(pose, meta_prev, Robot.coord_ini[2], dist_filtradas)
    return pose


# determinacion si cambiar a siguiente meta
def siguiente_meta(nodos_new, cont_camino, camino, pose, cont_goals, goals_orden, pos_prod, prod_pedidos, dist_filtradas):
    # distancia a meta actual
    dist_goal_actual = math.hypot(nodos_new[camino[cont_camino]][0]-pose[0], nodos_new[camino[cont_camino]][1]-pose[1])
    # si distancia es menor a cierto threshold se cambia al siguiente nodo de la trayectoria
    if dist_goal_actual < Robot.dist_final and cont_camino < len(camino)-1:
        # si el nodo actual es una posicion de un producto pedido se activa secuencia de mecanismo
        if camino[cont_camino] == goals_orden[cont_goals]:
            # secuencia de mecanismo para nivel
            print('--> Mecanismo Nivel '+str(int(pos_prod[prod_pedidos[goals_orden[cont_goals]-1]][3]))+' para producto '+str(prod_pedidos[goals_orden[cont_goals]-1]+1))
            # movimiento de robot cerca al estante
            pose = movimiento_estante(pos_prod[prod_pedidos[goals_orden[cont_goals]-1]][0:3], nodos_new[camino[cont_camino]], pose, dist_filtradas)
            # actualizacion a siguiente meta
            cont_goals += 1
            if cont_goals < len(goals_orden)-1:
                disp_str = "PRODUCTO  " + str(prod_pedidos[goals_orden[cont_goals] - 1] + 1)
            else:
                disp_str = "RETORNANDO"
            # Borrar imagen
            draw.rectangle((0, 0, width, height), outline=0, fill=0)
            # Escirbir Texto
            draw.text((x + 10, top + 10), disp_str, font=font, fill=255)
            # Display
            disp.image(image)
            disp.display()
        # actualizacion a siguiente nodo de trayectoria
        cont_camino += 1
    return cont_camino, cont_goals, pose


# repulsion
def repulsion(dist_sensores, orientacion):
    # calculo velocidad de repulsion
    v_repulsion = [0, 0]
    for i in range(0, Robot.n_sensores):
        if dist_sensores[i] < Robot.rango_sensor:
            # modulo de velocidad de repulsion por cada sensor
            v = Robot.c_repulsion / 2 * (1 / (dist_sensores[i] - Robot.radio_robot)) ** 2
            # direccion de aplicacion de velocidad
            v_repulsion = [v_repulsion[0]+vecs_sensores[i][0]*v, v_repulsion[1]+vecs_sensores[i][1]*v]
    v_repulsion = [math.cos(orientacion)*v_repulsion[0]-math.sin(orientacion)*v_repulsion[1],
                   math.sin(orientacion)*v_repulsion[0]+math.cos(orientacion)*v_repulsion[1]]
    return v_repulsion


def read_i2c(addr):
    ReadCheck = True
    while ReadCheck:
        try:
            time.sleep(0.02)
            data = bus.read_i2c_block_data(addr, 0x00, 9)
        except:
            print('Read')
            time.sleep(0.01)
            continue
        # borrar exceso de valores
        aux = 1
        while aux == 1:
            try:
                data.remove(255)
            except ValueError:
                aux = 0
        ThetaStr = "".join(map(chr, data))
        try:
            Theta = float(ThetaStr)
            return Theta / 1e2
        except ValueError:
            continue
        # time.sleep(0.02)


# Función para convertir string a byte para enio de datos
def convert_str2_byte(string):
    convertidos = []
    for i in string:
        convertidos.append(ord(i))
    return convertidos


def write_i2c(addr, rpm):
    val_string = str(round(rpm, 1))
    val = convert_str2_byte(val_string[::-1])
    write_check = True
    while write_check:
        # bus.write_i2c_block_data(addr, 0x00, val)
        try:
            # time.sleep(0.01)
            bus.write_i2c_block_data(addr, 0x00, val)
            write_check = False
        except:
            print('Write')
            # time.sleep(0.01)
            pass


# programa principal del robot para un nuevo pedido
def nuevo_pedido(prod_pedidos, pos_prev, nodos, mat_pesos, vecs_obs, points_obs, pos_prod):
    prod_pedidos = [i - 1 for i in prod_pedidos]
    # generacion de metas de productos pedidos
    goals = gen_goals(prod_pedidos, pos_prev)
    # generar nueva matriz de pesos con metas de productos pedidos y posicion inical
    nodos_new, mat_pesos_new = nuevo_grafo(goals, nodos, mat_pesos, vecs_obs, points_obs)
    # # generar mejorar trayectoria que pase por todas las metas
    camino, goals_orden = trayectoria(goals, mat_pesos_new, nodos_new)
    # posicion inicial
    pose = [Robot.coord_ini[0], Robot.coord_ini[1], Robot.coord_ini[2]]
    # siguiente nodo en la ruta
    cont_camino = 1
    # contador de metas
    cont_goals = 0
    disp_str = "PRODUCTO  " + str(prod_pedidos[goals_orden[cont_goals] - 1] + 1)
    # Borrar imagen
    draw.rectangle((0, 0, width, height), outline=0, fill=0)
    # Escirbir Texto
    draw.text((x + 10, top + 10), disp_str, font=font, fill=255)
    # Display
    disp.image(image)
    disp.display()

    # inicializar vector para filtrar mediciones
    dist_filtradas = [0] * Robot.n_sensores
    # inicializar velocidad de control
    v_control = [0, 0, 0]
    velocidad(pose, v_control)
    fin = True
    while fin:
        # actualizacion de la pose por odometria
        pose = odometria(pose, v_control)
        # verificacion de terminacion de trayectoria
        if cont_camino == len(camino) - 1 and math.hypot(nodos_new[camino[cont_camino]][0] - pose[0], nodos_new[camino[cont_camino]][1] - pose[1]) < Robot.dist_final:
            if abs(Robot.coord_ini[2] - pose[2]) < Robot.ang_final:
                # Borrar imagen
                draw.rectangle((0, 0, width, height), outline=0, fill=0)
                # Escirbir Texto
                draw.text((x + 10, top + 10), "ESPERANDO PEDIDO...", font=font, fill=255)
                # Display
                disp.image(image)
                disp.display()
                break
        # actualizacion de meta
        cont_camino, cont_goals, pose = siguiente_meta(nodos_new, cont_camino, camino, pose, cont_goals, goals_orden, pos_prod, prod_pedidos, dist_filtradas)
        # cálculo de velocidad de control
        # si Robot.metodo_orientacion = 1 robot gira hacia orientacion de producto
        # si Robot.metodo_orientacion = 0 robot se mueve con orientacion 0 grados
        if Robot.metodo_orientacion == 1 and cont_goals < len(goals_orden) - 1:
            v_control, dist_filtradas = calc_velocidad(nodos_new[camino[cont_camino]], pose, pos_prev[prod_pedidos[goals_orden[cont_goals] - 1]][2], dist_filtradas)
        else:
            v_control, dist_filtradas = calc_velocidad(nodos_new[camino[cont_camino]], pose, Robot.coord_ini[2], dist_filtradas)
        velocidad(pose, v_control)


# programa que se ejecuta desde la pagina web
def programa():
    global items, running
    running = 1
    prod_pedidos = [2, 4]
    for item in items:
        if item['state']:
            item['state'] = False
            prod_pedidos.append(item['id'])
    for item in items:
        item['state'] = False
    print("Pedidos "+str(prod_pedidos))
    # time.sleep(4)
    nuevo_pedido(prod_pedidos, pos_prev, nodos, mat_pesos, vecs_obs, points_obs, pos_prod)
    running = 2

programa()