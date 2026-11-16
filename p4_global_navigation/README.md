# Memoria de Práctica: Navegación Global (TeleTaxi)
### Robótica Móvil — 3º Ingeniería de Robótica Software

---

## 1. Objetivo de la práctica

El objetivo de esta práctica es desarrollar un sistema de **navegación global** para un robot móvil en el entorno TeleTaxi de **Unibotics (JdeRobot Robotics Academy)**. El usuario selecciona un destino sobre el mapa y el robot debe calcular una trayectoria para llegar hasta él evitando los edificios y siguiendo las carreteras disponibles.

La práctica se basa en el concepto de **Gradient Path Planning (GPP)**: generar un campo de costes a partir del destino y utilizarlo para guiar al robot hacia él, teniendo en cuenta la presencia de obstáculos.

El robot proporciona su posición y orientación mediante `HAL.getPose3d()`, mientras que el movimiento se controla con `HAL.setV()` y `HAL.setW()`. El mapa y el destino se obtienen mediante la API de `WebGUI`, y el bucle principal se ejecuta a frecuencia fija utilizando `Frequency.tick()`.

Mi objetivo no era únicamente conseguir que el robot encontrase un camino válido, sino que también fuese capaz de seguirlo correctamente, realizar los giros necesarios y llegar al destino sin quedarse detenido entre distintos puntos de la trayectoria.

---

## 2. Planteamiento inicial

Mi primera idea fue dividir el problema en dos partes bastante diferenciadas:

- **Planificación global:** utilizar el mapa para calcular un camino desde la posición actual del robot hasta el destino.
- **Control del movimiento:** transformar ese camino en una serie de objetivos que el robot pueda seguir mediante velocidades lineales y angulares.

Al principio preferí no mezclar ambas partes. Primero quería conseguir que el planificador generase una trayectoria correcta y después trabajar sobre el movimiento del vehículo.

También tuve que tener en cuenta que el mapa se representa como una matriz de píxeles, mientras que la posición del robot y el destino se expresan en coordenadas del mundo. Por tanto, antes de poder controlar el movimiento necesitaba entender bien cómo se relacionaban ambos sistemas de coordenadas.

La solución fue desarrollándose progresivamente. Empecé con un planificador básico, después añadí un criterio para alejar las trayectorias de los edificios y, finalmente, incorporé el seguimiento mediante waypoints y el controlador angular.

---

## 3. Arquitectura del código

He organizado la solución en un único archivo Python, separando las diferentes partes mediante funciones para que cada una tenga una responsabilidad concreta.

La estructura general es la siguiente:

| Parte | Responsabilidad |
|---|---|
| Configuración | Parámetros del mapa, penalizaciones y controlador |
| Conversión de coordenadas | Transformación entre el mapa y el mundo |
| Distancia a obstáculos | Cálculo de la proximidad a los edificios |
| Penalización | Generación del campo de penalización |
| Planificación global | Cálculo del campo de costes mediante Dijkstra |
| Extracción del camino | Obtención de la trayectoria desde el robot al destino |
| Seguimiento | Conversión de la ruta a waypoints y selección del objetivo |
| Controlador | Cálculo de las velocidades lineal y angular |
| `main()` | Integración de todos los componentes y ejecución continua |

He intentado mantener los parámetros configurables al principio del archivo. De esta manera, puedo modificar el comportamiento del robot sin tener que cambiar la lógica de las funciones.

---

## 4. Planificación global

### 4.1 Representación del mapa

Lo primero que hice fue obtener el mapa utilizando `WebGUI.getMap()`:

```python
MAP_URL = '/resources/exercises/global_navigation/images/cityLargenBin.png'
map_data = WebGUI.getMap(MAP_URL)
```

El mapa tiene unas dimensiones de 400 × 400 píxeles y es una imagen binaria. Al analizar sus valores comprobé que:

- `255` representa las carreteras, es decir, las zonas transitables.
- `0` representa los edificios y obstáculos.

A partir de esta información, el planificador puede distinguir qué posiciones se pueden utilizar para construir una trayectoria y cuáles deben evitarse.

Para calcular los movimientos posibles utilizo los ocho vecinos de cada celda: cuatro movimientos ortogonales y cuatro diagonales.

Los costes son:

- Movimiento horizontal o vertical: 1.
- Movimiento diagonal: \(\sqrt{2}\).

También añadí una comprobación para evitar el *corner cutting*. Si un movimiento diagonal atraviesa la esquina de un edificio, se descarta aunque la celda de destino sea transitable. Para ello compruebo que las dos celdas ortogonales adyacentes también sean carretera.

### 4.2 Primer planteamiento: algoritmo de Dijkstra

Para calcular la trayectoria inicial decidí utilizar el algoritmo de **Dijkstra**.

La idea es propagar los costes desde el destino hacia el resto del mapa. El destino comienza con coste cero y, a partir de ahí, se van procesando las celdas pendientes con menor coste acumulado.

Para gestionar las posiciones pendientes utilizo una cola de prioridad mediante `heapq`, lo que permite procesar primero las posiciones con menor coste.

El resultado es una matriz que almacena el coste mínimo necesario para llegar al destino desde cada celda transitable.

Elegí este algoritmo porque permite trabajar con diferentes costes de movimiento y posteriormente me iba a permitir incorporar otros factores al cálculo de la trayectoria sin tener que cambiar toda la estructura.

### 4.3 Extracción de la trayectoria

Una vez calculado el campo de costes, necesitaba obtener el camino concreto que debía seguir el robot.

Para ello, comienzo en la posición del vehículo y voy seleccionando los vecinos que permiten reducir el coste hasta llegar al destino.

Durante las primeras pruebas observé que seleccionar simplemente cualquier vecino con menor coste podía producir cambios de dirección innecesarios. Por eso añadí un criterio de continuidad: cuando hay varios candidatos con un coste igualmente válido, se favorece el que mantiene una dirección más parecida a la del movimiento anterior.

También compruebo que cada movimiento sea válido y que no se repitan posiciones.

Con esto consigo obtener una lista ordenada de coordenadas `(fila, columna)` que representa el recorrido completo.

---

## 5. Mejora de la planificación: penalización de obstáculos

### 5.1 Problema de la primera versión

Con el planificador inicial ya conseguía generar caminos válidos, pero había un aspecto que quería mejorar: el algoritmo únicamente tenía en cuenta el coste de desplazamiento.

Esto significaba que podía elegir una trayectoria que pasase muy cerca de los edificios, siempre que fuese una de las opciones más cortas.

Aunque el camino no atravesase ningún obstáculo, no me parecía el comportamiento más adecuado. Prefería que el robot mantuviese cierta distancia respecto a ellos, aunque eso supusiese recorrer algunos metros más.

Por este motivo decidí incorporar una penalización en función de la proximidad a los edificios.

### 5.2 Cálculo de distancias a los edificios

Para poder aplicar esta penalización, primero necesitaba conocer la distancia de cada celda al obstáculo más cercano.

Implementé una propagación desde todas las celdas ocupadas del mapa. Inicialmente, las posiciones correspondientes a edificios tienen distancia cero y el resto tienen distancia infinita.

A partir de ahí, se propagan las distancias utilizando los mismos ocho movimientos que en el planificador, con costes ortogonales de 1 y diagonales de \(\sqrt{2}\).

De esta forma obtengo un campo de distancias que indica lo cerca que se encuentra cada posición de un edificio.

### 5.3 Función de penalización

Una vez calculadas las distancias, definí una función de penalización cuadrática:

\[
P(d)=
\begin{cases}
\lambda\left(\frac{R-d}{R}\right)^2 & d<R\\
0 & d\geq R
\end{cases}
\]

Los parámetros que utilicé finalmente fueron:

| Parámetro | Valor |
|---|---:|
| `INFLUENCE_RADIUS` | 6.0 |
| `PENALTY_STRENGTH` | 3.0 |

La función hace que las posiciones próximas a los edificios tengan un coste adicional. Cuanto menor es la distancia al obstáculo, mayor es la penalización.

Cuando la distancia alcanza el radio de influencia, la penalización pasa a ser cero.

Elegí esta función porque aumenta progresivamente el coste al acercarse a un edificio, sin introducir un salto brusco en el límite del radio de influencia.

### 5.4 Incorporación al coste de movimiento

El siguiente paso fue integrar la penalización en Dijkstra.

Para ello, modifiqué el coste de transición entre dos celdas:

\[
C(i,j)=D(i,j)\left(1+\frac{P(i)+P(j)}{2}\right)
\]

Donde \(D(i,j)\) es el coste geométrico del movimiento y \(P(i)\), \(P(j)\) son las penalizaciones de las dos celdas.

Utilizo la media de ambas penalizaciones para tener en cuenta tanto la posición actual como la siguiente.

Esta modificación permite que el planificador no busque únicamente el camino más corto en distancia, sino que tenga en cuenta también la proximidad a los obstáculos.

El resultado es un compromiso entre longitud de la trayectoria y separación respecto a los edificios. En determinadas situaciones, el robot puede recorrer un camino algo más largo si con ello evita pasar demasiado cerca de un obstáculo.

---

## 6. Conversión de coordenadas

Esta fue una de las partes que más tuve que comprobar durante el desarrollo, porque necesitaba conectar correctamente el planificador con el movimiento real del robot.

### 6.1 De coordenadas del mundo al mapa

La posición del robot y el destino se expresan mediante coordenadas `(x, y)`. Sin embargo, para consultar el mapa necesito utilizar índices de matriz `(fila, columna)`.

Para realizar la conversión utilicé `WebGUI.rowColumn()`.

Durante las primeras pruebas observé que el orden devuelto por esta función no coincidía directamente con el orden necesario para indexar la matriz. Al comprobar diferentes posiciones, vi que tenía que intercambiar los componentes:

```python
target_rc = WebGUI.rowColumn(target)

goal = (
    int(target_rc[1]),
    int(target_rc[0])
)
```

Hice varias comprobaciones consultando directamente el valor de los píxeles. En algunos casos, utilizando el orden original obtenía un edificio, mientras que al intercambiar los componentes la posición correspondía correctamente a una carretera.

Una vez identificado este comportamiento, apliqué la misma conversión tanto al destino como a la posición del robot.

### 6.2 Del mapa al mundo

Después de obtener la trayectoria, necesitaba convertir sus puntos a coordenadas del mundo para poder utilizarlos como objetivos del controlador.

A partir de varias pruebas con `WebGUI.rowColumn()`, observé que la relación entre las coordenadas del mundo y los índices de la matriz era aproximadamente lineal:

\[
row \approx 200+0.8y
\]

\[
col \approx 200+0.8x
\]

Por tanto, utilicé la siguiente transformación inversa:

\[
x=\frac{row-200}{0.8}
\]

\[
y=\frac{col-200}{0.8}
\]

La función queda así:

```python
def grid_to_world(row, col):

    x = (row - MAP_CENTER) / PIXELS_PER_METER
    y = (col - MAP_CENTER) / PIXELS_PER_METER

    return x, y
```

Esta transformación es una aproximación al centro de cada celda, por lo que no tiene por qué recuperar exactamente la coordenada original.

Para comprobarla, realicé varias conversiones de ida y vuelta. Al transformar una posición del mundo al mapa y volver a utilizar `WebGUI.rowColumn()` sobre la posición reconstruida, obtenía los mismos índices de la matriz.

Con estas comprobaciones consideré que la conversión era suficientemente precisa para utilizarla en el seguimiento de la trayectoria.

### 6.3 Visualización del camino

También tuve que tener en cuenta que `WebGUI.showPath()` utiliza las coordenadas en orden `(x, y)`, mientras que mi trayectoria está almacenada como `(fila, columna)`.

Por tanto, antes de representar el camino, intercambio los componentes:

```python
path_array = np.array(
    [(col, row) for row, col in path],
    dtype=np.int32
)

WebGUI.showPath(path_array)
```

De esta forma, la trayectoria se dibuja correctamente sobre el mapa.

---

## 7. Seguimiento de la trayectoria mediante waypoints

Una vez que el planificador generaba una ruta válida, el siguiente paso era conseguir que el robot la siguiese.

Mi primera aproximación consistía en utilizar los puntos de la trayectoria como objetivos, pero no quería que el vehículo tuviese que detenerse o corregir su orientación continuamente en cada celda.

Por eso incorporé un sistema de seguimiento basado en **waypoints y distancia de anticipación (*lookahead*)**.

### 7.1 Búsqueda del waypoint más cercano

En cada iteración obtengo la posición actual del robot mediante `HAL.getPose3d()` y busco el punto de la trayectoria que se encuentra más cerca.

La distancia se calcula mediante:

\[
d=\sqrt{(x_r-x_w)^2+(y_r-y_w)^2}
\]

Donde `(x_r, y_r)` es la posición del robot y `(x_w, y_w)` la del waypoint.

Para no tener que recorrer toda la trayectoria en cada iteración, guardo el índice del último waypoint y limito la búsqueda a una ventana de puntos posteriores.

El valor utilizado es:

```python
NEAREST_SEARCH_WINDOW = 40
```

Esto permite que el seguimiento avance por la trayectoria sin tener que buscar constantemente desde el principio.

### 7.2 Selección del waypoint de anticipación

Una vez localizado el waypoint más cercano, selecciono otro punto situado a una distancia de anticipación de 5 metros sobre la trayectoria.

Para hacerlo, voy sumando las distancias entre los puntos consecutivos hasta alcanzar la distancia configurada:

```python
LOOKAHEAD_DISTANCE = 5.0
```

El punto seleccionado se convierte en el objetivo temporal del controlador.

La ventaja de este planteamiento es que el robot no necesita dirigirse a cada celda individualmente. En lugar de eso, intenta alcanzar un punto situado más adelante en el recorrido, consiguiendo un seguimiento más continuo.

---

## 8. Controlador de movimiento

### 8.1 Cálculo del error angular

Una vez seleccionado el waypoint objetivo, necesito calcular hacia dónde debe orientarse el robot.

Primero obtengo el ángulo que une la posición actual del vehículo con el objetivo:

\[
\theta_d=\operatorname{atan2}(y_w-y_r,x_w-x_r)
\]

Después calculo el error respecto a la orientación actual:

\[
e_\theta=\theta_d-\theta_r
\]

Para evitar problemas al cruzar los límites de una vuelta completa, normalizo el error al intervalo \([-\pi,\pi]\).

De esta forma, el robot siempre intenta realizar el giro por el sentido que supone el menor cambio angular.

### 8.2 Comprobación del sentido de giro

Antes de implementar el controlador completo, hice una prueba específica para comprobar cómo respondía el vehículo a las velocidades angulares.

Probé valores positivos y negativos de `HAL.setW()` y observé cómo cambiaba el yaw.

Comprobé que una velocidad angular positiva incrementaba el yaw y una negativa lo disminuía.

Esta prueba me permitió confirmar el sentido de giro y asegurarme de que el controlador aplicaba las correcciones en la dirección adecuada.

### 8.3 Control proporcional angular

Para controlar la orientación utilicé un controlador proporcional:

\[
W=K_P\cdot e_\theta
\]

La velocidad angular depende directamente del error entre la orientación actual y la deseada.

Si el error es grande, se aplica una corrección mayor. A medida que el robot se orienta hacia el objetivo, el error disminuye y también lo hace la velocidad angular.

La ganancia utilizada es:

```python
K_P = 1.0
```

Además, establecí una tolerancia angular de 3 grados para considerar que el robot está suficientemente orientado.

Durante esta fase mantengo la velocidad lineal a cero, de forma que el vehículo primero corrige su orientación y después comienza a avanzar.

### 8.4 Movimiento combinado

Una vez que el robot está orientado hacia el waypoint, permito el avance lineal.

Durante el desplazamiento sigo calculando el error angular para corregir la dirección del vehículo. Si el error es demasiado elevado, se reduce o detiene el avance para evitar que continúe desplazándose en una dirección incorrecta.

Los límites de velocidad que he decidido mantener en la versión final son:

```python
V_MAX = 30
W_MAX = 15
```

También utilizo una tolerancia de 2 metros para considerar que el robot ha alcanzado el destino.

---

## 9. Integración y ajustes finales

Una vez que cada parte funcionaba de forma independiente, integré el planificador y el controlador en el programa principal.

El funcionamiento general es:

1. Se carga el mapa del entorno.
2. Se obtiene la posición actual del robot.
3. Se comprueba si el usuario ha seleccionado un destino.
4. Se convierten la posición inicial y el destino a coordenadas del mapa.
5. Se calcula el campo de distancias a los edificios.
6. Se obtiene el campo de penalización.
7. Se calcula el campo de costes mediante Dijkstra.
8. Se extrae la trayectoria desde el robot hasta el destino.
9. Se convierte la trayectoria a coordenadas del mundo.
10. Se busca el waypoint más cercano.
11. Se selecciona el waypoint de anticipación.
12. Se calcula el error angular y se envían las velocidades al vehículo.
13. Se comprueba si se ha alcanzado el destino.

También tuve en cuenta que el usuario puede seleccionar un nuevo destino durante la ejecución. Cuando detecto un cambio, se vuelve a calcular la trayectoria para adaptarse al nuevo objetivo.

### 9.1 Problemas durante el seguimiento

Durante las primeras pruebas con el movimiento completo observé que el robot llegaba a realizar el recorrido, pero en algunos momentos se producían pequeños parones entre waypoints.

El problema no estaba en la generación de la ruta, sino en la forma de gestionar el seguimiento y las correcciones del vehículo.

Después de revisar el código, simplifiqué parte de la lógica y eliminé comprobaciones y funciones auxiliares que ya no eran necesarias para el funcionamiento final.

Tras estos cambios, el comportamiento mejoró y dejaron de aparecer los parones que había observado anteriormente.

### 9.2 Ajuste de velocidades

Por último, fui modificando los límites de velocidad para conseguir que el robot recorriese la trayectoria con mayor rapidez.

Probé diferentes valores y decidí mantener finalmente:

| Parámetro | Valor |
|---|---:|
| `V_MAX` | 30 |
| `W_MAX` | 15 |

Con estos valores estoy satisfecho con el comportamiento obtenido y he decidido no seguir añadiendo modificaciones a la lógica principal, ya que considero que la solución actual cumple el objetivo de la práctica.

---

## 10. Limitaciones observadas

Aunque el comportamiento final es satisfactorio, hay algunos aspectos que podrían mejorarse:

- **Trayectoria discreta:** el camino se calcula sobre una rejilla, por lo que está formado por segmentos entre celdas. Se podría aplicar un suavizado posterior para obtener recorridos más continuos, siempre comprobando que no atraviesen edificios.
- **Ajuste de velocidades:** actualmente utilizo límites máximos configurados manualmente. Se podría adaptar la velocidad lineal a la curvatura del camino o al error angular.
- **Parámetros de penalización:** los valores del radio de influencia y la intensidad afectan directamente a la trayectoria. Se podrían realizar más pruebas para estudiar diferentes combinaciones.
- **Resolución del mapa:** la precisión de la trayectoria depende de la resolución de la matriz y de la conversión entre coordenadas.

He preferido no incorporar estas mejoras a la versión final para mantener una solución sencilla y estable, en lugar de seguir añadiendo funcionalidades que no son necesarias para cumplir el objetivo principal.

---

## 11. Flujo final del sistema

El funcionamiento completo del programa se puede resumir de la siguiente manera:

```text
          CARGA DEL MAPA
                 |
                 v
       SELECCIÓN DEL DESTINO
                 |
                 v
      CONVERSIÓN DE COORDENADAS
                 |
                 v
       DISTANCIA A OBSTÁCULOS
                 |
                 v
       CAMPO DE PENALIZACIÓN
                 |
                 v
       CAMPO DE COSTES (DIJKSTRA)
                 |
                 v
        EXTRACCIÓN DEL CAMINO
                 |
                 v
       CONVERSIÓN AL MUNDO
                 |
                 v
       BÚSQUEDA DEL WAYPOINT
                 |
                 v
       SELECCIÓN LOOKAHEAD
                 |
                 v
       CÁLCULO DEL ERROR ANGULAR
                 |
                 v
       CONTROL DE VELOCIDADES
                 |
                 v
       ¿DESTINO ALCANZADO?
           /          \
         NO            SÍ
         |              |
         v              v
    CONTINUAR          PARAR
     CONTROL
```

Si el usuario selecciona un nuevo destino durante la ejecución, el sistema vuelve a realizar la planificación para calcular una nueva trayectoria.

---

## 12. Demostración

[![Demostración de Navegación Global](../docs/p4/p4_demo.gif)](../docs/p4/p4_demo.gif)

### 12.1 Resultados

En la demostración se podrá observar:

- La selección de diferentes destinos sobre el mapa.
- La generación de la trayectoria evitando los edificios.
- La separación respecto a los obstáculos gracias a la penalización.
- El seguimiento de la ruta mediante waypoints.
- Los giros y el desplazamiento del vehículo hasta alcanzar el destino.

---

## 13. Conclusiones

El resultado final cumple el objetivo principal de la práctica: el robot es capaz de calcular una trayectoria global desde su posición hasta un destino seleccionado por el usuario y seguirla evitando los edificios.

La parte que más trabajo me ha llevado ha sido conectar correctamente la planificación sobre la matriz con el movimiento real del vehículo. Especialmente, la conversión de coordenadas y el seguimiento mediante waypoints requirieron varias pruebas hasta conseguir un comportamiento adecuado.

También considero importante la mejora introducida mediante la penalización de obstáculos. Con ella, el planificador no se limita a encontrar el camino más corto, sino que tiene en cuenta la proximidad a los edificios y puede elegir recorridos más convenientes.

Por último, el desarrollo del controlador me ha permitido comprobar que tener una buena trayectoria calculada no garantiza por sí solo que el robot vaya a seguirla correctamente. Es necesario tener en cuenta su orientación, seleccionar objetivos intermedios y ajustar las velocidades para conseguir un movimiento estable.

En general, estoy satisfecho con la solución final. He conseguido integrar la planificación global y el control del vehículo en un único programa, y he podido comprobar en simulación que el robot llega a los destinos seleccionados siguiendo las trayectorias generadas.

El aprendizaje principal de esta práctica ha sido entender cómo se conectan la representación del entorno, la planificación de caminos y el control de un robot móvil para conseguir un sistema de navegación completo.