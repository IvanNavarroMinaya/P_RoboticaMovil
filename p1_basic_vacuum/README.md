# Memoria de Práctica: Aspiradora Autónoma Básica
### Robótica Móvil — 3º Ingeniería de Robótica Software

---

## 1. Objetivo de la práctica

El objetivo de esta práctica es desarrollar un robot aspiradora autónomo en Python utilizando el entorno de simulación **JdeRobot Robotics Academy**. El robot dispone de un **láser de 180°** como único sensor, y la meta es que recorra y limpie al menos el **50% del área habitable** de la casa de forma autónoma.

Las restricciones del entorno son importantes: **no se pueden usar `sleep()`**, el bucle principal se ejecuta a frecuencia fija controlada por `Frequency.tick()`, y los únicos actuadores disponibles son `HAL.setV()` para la velocidad lineal y `HAL.setW()` para la velocidad angular.

---

## 2. Planteamiento inicial

Mi primera idea fue bastante simple: el robot avanza en línea recta y, cuando detecta un obstáculo por delante, para y gira en la dirección donde haya más espacio libre. Al ser una aspiradora, la velocidad tenía que ser baja.

El código inicial de partida ya tenía algo parecido, pero con varios errores de base:

- Usaba `laser_data.values(90)` con paréntesis en vez de corchetes, lo cual es un error de sintaxis en Python: los paréntesis llaman a una función, los corchetes indexan una lista.
- La condición de obstáculo era `if not front_distance`, que solo se cumple si la distancia es exactamente 0. Nunca se iba a activar en condiciones normales.
- La velocidad y el giro se recalculaban con `random.uniform()` en cada tick del bucle (a 30 Hz), por lo que el robot temblaba constantemente sin llegar a ningún sitio.

Visto esto, decidí replantear el código desde cero.

---

## 3. Decisión de usar una Máquina de Estados Finita (FSM)

El primer rediseño importante fue estructurar el comportamiento del robot como una **máquina de estados finita (FSM)**. La razón es sencilla: el problema del robot que decide cada tick sin memoria es que no tiene coherencia; toma decisiones contradictorias en milisegundos. Con una FSM, el robot sabe en qué modo está y solo toma una decisión concreta (avanzar, girar, parar) hasta que se cumple una condición de transición clara.

La arquitectura que diseñé organiza cada estado como una función independiente que devuelve el siguiente estado. Esto tiene una ventaja enorme para depurar: si el robot hace algo raro, miro qué estado está activo y reviso solo esa función.

```
HANDLERS = {
    IDLE:    state_idle,
    SPIRAL:  state_spiral,
    FORWARD: state_forward,
    STOP:    state_stop,
    TURN:    state_turn,
}
```

El bucle principal simplemente llama a `HANDLERS[current_state](...)` y actualiza el estado. Limpio y fácil de extender.

---

## 4. Estados de la FSM

### 4.1 IDLE

Estado inicial. El robot permanece parado hasta que recibe lecturas válidas del láser. Es necesario porque el simulador puede tardar unos ciclos en devolver datos del sensor al arrancar. Sin este estado, el robot intentaría ejecutar lógica con una lista vacía y daría error.

En cuanto `distances` tiene contenido, se inicializa la espiral y se transita a `SPIRAL`.

### 4.2 STOP

Cuando el robot detecta un obstáculo, primero para completamente (`v=0, w=0`) en este estado antes de girar. Esto es importante por dos motivos: primero, evita que el robot siga avanzando mientras calcula el ángulo de giro (lo que podría provocar una colisión real). Segundo, es en este estado donde se toma la decisión del ángulo de giro **una sola vez**, y ese valor se guarda en `target_yaw` para que `TURN` lo ejecute fielmente.

Una decisión de diseño relevante aquí: en las primeras versiones, el ángulo de giro se calculaba dentro del propio estado `FORWARD` al detectar el obstáculo. Esto era un problema, porque si en el siguiente tick el láser daba una lectura ligeramente diferente, se recalculaba el ángulo y el robot oscilaba. Separar `STOP` de `TURN` y calcular el ángulo solo una vez solucionó esto.

### 4.3 TURN

Ejecuta el giro hasta alcanzar `target_yaw`. Usa `HAL.getPose3d().yaw` para saber la orientación actual y calcula el error como:

```python
error = normalize_angle(target_yaw - yaw)
```

La función `normalize_angle` es clave aquí: sin ella, si el yaw pasa de π a -π (cruzando el ±180°), el error da un salto de 2π y el robot gira en sentido contrario. Usando `atan2(sin, cos)` se fuerza el resultado al rango [-π, π] y el giro siempre toma el camino más corto.

El sentido del giro se determina por el signo del error: positivo gira a la izquierda (`w > 0`), negativo a la derecha (`w < 0`).

Cuando el error cae por debajo de `YAW_TOL = 0.05 rad` (~3°), el giro se da por completado y se transita a `FORWARD` con el contador `forward_ticks` a cero.

### 4.4 FORWARD

Avance en línea recta a `V_FORWARD = 0.4 m/s`. Tiene dos condiciones de salida:
- Si detecta un obstáculo, transita a `STOP`.
- Si acumula `FORWARD_TICKS = 800` ticks sin chocar, considera que ha explorado esa dirección suficientemente y vuelve a lanzar una nueva espiral desde `reset_spiral`.

El valor de `FORWARD_TICKS` se aumentó de 400 a 800 a lo largo del desarrollo, porque con 400 ticks el robot cambiaba de comportamiento demasiado pronto y no aprovechaba los pasillos largos.

### 4.5 SPIRAL

Estado central de cobertura. El robot describe círculos concéntricos de radio creciente para barrer una zona de forma sistemática antes de explorar libremente.

---

## 5. Diseño del estado SPIRAL: el camino hasta hacerlo funcionar

Este fue el estado que más iteraciones requirió. Cuento el proceso completo porque cada fallo enseñó algo.

### 5.1 Primera versión: velocidad lineal creciente (v aumenta, w fija)

La idea inicial era la más sencilla: mantener `w` constante y aumentar `v` poco a poco. Como `r = v/w`, el radio crece conforme sube `v`. El problema es que a medida que `v` sube, las vueltas son cada vez más rápidas y el robot empieza a dejarse franjas sin limpiar entre vuelta y vuelta. Además, al llegar a `V_SPIRAL_MAX` se quedaba dando vueltas en un círculo fijo sin transitar a otro estado.

Solución al segundo problema: detectar cuando `v` alcanza el tope y transitar a `FORWARD`. Pero el problema de los huecos seguía ahí.

### 5.2 Segunda versión: espiral de Arquímedes (r crece linealmente con el ángulo)

Para que la separación entre vueltas sea constante, hay que hacer crecer `r` linealmente con el ángulo total girado: `r = R0 + k·θ`. Esto da una espiral de Arquímedes real, donde la distancia entre vueltas consecutivas es siempre `2πk`.

Para medir `θ` correctamente sin depender de la frecuencia del bucle, integro el yaw en cada tick:

```python
spiral_angle += normalize_angle(yaw - spiral_last_yaw)
spiral_last_yaw = yaw
```

El resultado en simulación seguía dando una trayectoria visualmente elíptica. Tras analizar la ejecución, identifiqué dos causas:

- **El mapa del simulador está mal escalado visualmente.** La representación 2D del mapa tiene escala diferente en horizontal y vertical (el eje X parece el doble de largo que el Y), lo que hace que cualquier círculo real aparezca como una elipse en pantalla. El robot sí describe círculos; es la visualización la que los distorsiona.

- **El simulador limita la velocidad angular.** Comparando la velocidad angular medida (diferencia de yaw entre ticks) con la ordenada, quedó claro que el simulador no permite superar aproximadamente `W_MAX ≈ 0.5 rad/s`. En las primeras vueltas, yo ordenaba `w = 1.2 rad/s` y el simulador lo recortaba a 0.5, lo que hacía que el radio real fuera `r = v/0.5 = 0.6 m` independientemente del radio teórico calculado. Las tres primeras vueltas "de radios distintos" se dibujaban sobre el mismo anillo de 0.6 m, dejando el centro sin limpiar.

### 5.3 Versión final: círculos concéntricos con límite de w respetado

La solución fue replantear la espiral como **círculos concéntricos explícitos** y respetar el límite de `w` desde el principio:

```python
w = min(V_SPIRAL / spiral_radius, W_SPIRAL_MAX)
v = w * spiral_radius
```

Así, si el radio pedido requiere más velocidad angular de la que el simulador puede dar, se reduce `v` proporcionalmente para mantener `r = v/w` exactamente igual al radio deseado. El robot va más despacio en los círculos pequeños, pero el radio es correcto.

Para detectar la vuelta completa e incrementar el radio, acumulo el ángulo girado y compruebo cuando supera `2π`:

```python
if spiral_angle >= 2 * math.pi:
    spiral_angle -= 2 * math.pi
    spiral_radius += SPIRAL_DR
```

Con `SPIRAL_R0 = 0.3 m` y `SPIRAL_DR = 0.15 m`, la primera vuelta ya cubre el área central sin que el robot salga disparado. El valor de `SPIRAL_DR` se eligió pensando en que sea ligeramente menor que el ancho del robot aspiradora, para que los círculos consecutivos se solapen y no queden franjas sin barrer.

---

## 6. Inflado de obstáculos (inflate_obstacles)

Un problema recurrente en simulación era que el robot intentaba meterse por huecos entre obstáculos que en realidad eran más estrechos que su propio chasis. El láser devuelve 180 valores puntuales, y entre dos rayos consecutivos puede colarse una esquina que el robot no ve como obstáculo.

La solución fue aplicar un **filtro de mínimo** sobre cada rayo antes de pasar los datos a la FSM:

```python
for i in range(length):
    start = max(0, i - INFLATION_MARGIN)
    end = min(length, i + INFLATION_MARGIN + 1)
    inflated[i] = min(distances[start:end])
```

Con `INFLATION_MARGIN = 15`, cada rayo adopta la distancia mínima de los 15 rayos a cada lado. El efecto es que los obstáculos "engordan" virtualmente en la percepción del robot, que los evita con más margen. Esto mejoró notablemente el comportamiento en pasillos estrechos y esquinas.

---

## 7. Sistema antibloqueo

En zonas con muchas esquinas y paredes cercanas, el robot podía entrar en un bucle: chocaba, giraba hacia una dirección libre, volvía a chocar casi inmediatamente con otra pared, y así indefinidamente. Para detectar y romper este ciclo, añadí dos contadores:

- `recent_collisions`: incrementa en cada entrada a `STOP`.
- `ticks_since_collision`: incrementa en cada tick de `FORWARD`. Si supera 40 ticks sin colisión, se resetea `recent_collisions` a 0 (el robot está saliendo bien de la zona).

Cuando `recent_collisions` llega a 5, se ignora la lógica de búsqueda de dirección libre y se fuerza directamente un giro de 180°:

```python
if recent_collisions >= 5:
    turn_angle = math.pi
    recent_collisions = 0
```

Esto actúa como mecanismo de escape de mínimos locales: si el robot se queda atrapado girando entre dos paredes, la media vuelta lo saca hacia donde vino.

---

## 8. Elección de dirección en STOP (choose_turn_angle)

La función busca todos los rayos del láser cuya ventana de ±`FREE_WINDOW` grados esté completamente por encima de `FREE_DIST = 0.7 m`. De los rayos candidatos, elige uno al azar. El ángulo relativo al frente del robot se calcula restando 90 (porque el rayo 90 apunta al frente):

```python
return math.radians(random.choice(candidates) - 90)
```

Si no hay ningún candidato (esquina completamente cerrada), en vez de girar siempre exactamente 180°, se elige un ángulo aleatorio entre 135° y 225°. Esto añade aleatoriedad a la salida de esquinas difíciles, evitando que el robot repita siempre el mismo camino al escapar.

---

## 9. Limitaciones observadas durante la ejecución

### 9.1 Error de visualización del mapa

Durante las pruebas observé que el porcentaje de cobertura máximo alcanzable estaba limitado por un **problema en el propio entorno de simulación**, no en mi algoritmo.

El simulador renderiza la casa en **3D con sombras y perspectiva**, y el mapa 2D de cobertura se genera a partir de esa proyección. Algunas paredes del lado derecho de la casa, al tener sombra proyectada sobre el suelo adyacente, hacen que esa zona aparezca oscura en el mapa 2D. El sistema la interpreta como **suelo ya mapeado o como zona inaccesible**, cuando en realidad es una pared. Esto significa que existen zonas del mapa que el robot nunca podrá limpiar independientemente del algoritmo, ya que el propio entorno las marca incorrectamente.

Esta limitación del entorno hace que alcanzar el **100% de cobertura sea imposible**, pero no afecta al objetivo del 50% que se pide en la práctica.

### 9.2 Zonas estrechas y colisiones residuales

Con `OBSTACLE_DIST = 0.25 m`, el robot se acerca bastante a las paredes antes de frenar. En pasillos muy estrechos esto a veces provoca que el robot roce ligeramente las esquinas. Subir este umbral reducía las rozaduras pero hacía que el robot girase demasiado pronto en zonas abiertas, dejando muchas zonas sin recorrer. El valor de 0.25 m fue el compromiso más equilibrado encontrado.

---

## 10. Flujo final de la FSM

El flujo completo de estados en la versión final es el siguiente:

```
IDLE ──► SPIRAL ──► (obstáculo) ──► STOP ──► TURN ──► FORWARD
                                                          │
                                         (800 ticks libres)│
                                                          ▼
                                                        SPIRAL (nuevo)
                                         
                        (antibloqueo: 5 colisiones seguidas)
                        STOP ──► giro forzado 180° ──► TURN ──► FORWARD
```

El robot comienza siempre con la espiral para cubrir sistemáticamente la zona de arranque. Cuando choca, explora aleatoriamente con `FORWARD` hasta que lleva suficiente tiempo limpio, momento en el que vuelve a la espiral para barrer otra zona de forma metódica.

---

## 11. Demostración en vídeo
[![Demostración Aspiradora](Pegar video)]

---

## 12. Conclusiones

El resultado final cubre consistentemente el objetivo del 50% de la casa. La combinación de la espiral para cobertura inicial, el avance aleatorio para exploración y los mecanismos de escape (elección de dirección libre + antibloqueo por colisiones repetidas) consiguen que el robot no se quede atascado durante períodos prolongados.

El aprendizaje principal de esta práctica ha sido que la lógica de control no es el único factor determinante: los límites del simulador (velocidad angular máxima, escala del mapa) son igual de importantes y hay que medirlos empíricamente para que el diseño teórico funcione en la práctica. En un robot real, esto equivaldría a las limitaciones físicas de los motores y a las imprecisiones de los sensores.