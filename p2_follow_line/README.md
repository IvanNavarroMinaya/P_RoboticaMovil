# Memoria de Práctica: Fórmula 1 Sigue-Líneas
### Robótica Móvil — 3º Ingeniería de Robótica Software

---

## 1. Objetivo de la práctica

El objetivo de esta práctica es programar en Python el controlador de un Fórmula 1 simulado en **Unibotics** (ejercicio *Follow Line* de JdeRobot Robotics Academy) para que siga la **línea roja** pintada en el centro de la pista y complete distintos circuitos en el menor tiempo posible.

El robot dispone de una **cámara frontal** como único sensor, y los únicos actuadores son `HAL.setV()` para la velocidad lineal y `HAL.setW()` para la velocidad angular. El bucle principal se ejecuta a frecuencia fija controlada por `Frequency.tick()`, y la imagen de depuración se muestra con `WebGUI.showImage()`.

Se evalúa tanto la **velocidad** como la **robustez**, con más peso en la segunda. Por eso desde el principio no me planteé solo conseguir la vuelta más rápida, sino un comportamiento que no se descontrole ni se pierda aunque vaya deprisa.

---

## 2. Planteamiento inicial

Después de leer la documentación del ejercicio, mi idea inicial tenía cuatro piezas:

- Un **filtro de color en HSV** para quedarme solo con el rojo de la línea.
- Un **controlador PID** que modulase las velocidades.
- Una **política de recuperación**: si en algún momento el coche pierde la línea por un mal control, baja mucho la velocidad y vuelve a encontrarla.
- **Centrar la cámara**, porque en la imagen de inicio se ve que la línea aparece en el centro de la imagen, pero el coche está desplazado a la derecha: la línea queda sobre sus ruedas izquierdas en vez de en el centro. La cámara está, por tanto, a la izquierda del eje del coche.

Al revisar el diseño completo decidí ampliarlo con tres ideas:

1. **No mirar un solo punto de la imagen**, sino varias filas a distinta distancia, para poder anticipar las curvas.
2. **Separar el control lateral del longitudinal**: el PID decide cuánto girar y la velocidad lineal se adapta según lo que se ve venir, en vez de ir a velocidad constante.
3. Plantear la recuperación como una **máquina de estados** con fases, en lugar de un simple "baja la velocidad y busca".

Como la API solo ofrece `HAL.getImage()`, `HAL.setV()` y `HAL.setW()`, no hay forma de mover la cámara físicamente. Resolví la descentralización **por software** (apartado 5).

---

## 3. Arquitectura del código

Dejé todo en **un único archivo**, organizado en clases, para poder subirlo tal cual al editor de Unibotics. Cada clase tiene una responsabilidad clara:

| Clase | Responsabilidad |
|---|---|
| `LineDetector` | Máscara HSV roja y cálculo del error en tres filas de mirada |
| `PidController` | Control lateral con filtro en la derivada y anti-windup |
| `SpeedPlanner` | Limita aceleración (suave) y deceleración (fuerte) |
| `Driver` | Combina todo y gestiona la máquina de estados de recuperación |
| `main()` | Bucle principal: lee imagen, calcula, envía velocidades |

Todos los valores que se pueden afinar están agrupados al principio del archivo, de modo que para ajustar el comportamiento no hace falta tocar la lógica.

---

## 4. Percepción: detectar la línea roja

### 4.1 Filtro de color HSV

Trabajo en HSV porque el tono (*Hue*) separa el color de la iluminación mejor que BGR. En OpenCV el tono va de 0 a 179 y es un círculo, así que el rojo puro cae justo en el corte: hay rojos cerca de 0 y rojos cerca de 179. Con un único rango solo detectaría la mitad. Por eso uso **dos rangos y los uno**:

```python
mask = cv2.inRange(image_hsv, RED_LO_1, RED_HI_1) | cv2.inRange(image_hsv, RED_LO_2, RED_HI_2)
```

Con `S` mínimo de 120 y `V` mínimo de 70 descarto grises, blancos y zonas oscuras.

### 4.2 Limpieza de la máscara

La máscara cruda tiene ruido, así que aplico una **apertura** (quita píxeles sueltos) y un **cierre** (rellena huecos), ambas con un kernel elíptico de 5×5. Además:

- Pongo a cero todo lo que está por encima del **horizonte** (`HORIZON_FRAC = 0.50`), para evitar falsos positivos en el cielo o las gradas.
- Si queda más de una mancha roja, me quedo solo con la **más grande** (`connectedComponentsWithStats`), lo que descarta reflejos o manchas aisladas.

### 4.3 Tres filas de mirada

En vez de calcular un único centroide de toda la máscara, miro la línea en **tres filas**: cercana, media y lejana (`ROW_FRACS`: 0.90, 0.75 y 0.57 de la altura de la imagen). En cada una calculo el centro de masa horizontal de una pequeña banda de la máscara. Si una banda tiene menos de `MIN_BAND_PIXELS` píxeles rojos, esa fila se considera "no vista".

El error de cada fila se **normaliza a [-1, 1]** dividiendo entre la mitad del ancho, así las ganancias no dependen de la resolución de la cámara:

```python
error = (centroid_x - reference_x) / (width / 2.0)
```

Un error positivo significa que la línea está a la derecha de donde debería estar.

Para el control uso un **error combinado** ponderando las filas con `ROW_WEIGHTS = {near: 0.4, mid: 0.3, far: 0.3}`. La fila cercana pesa más para mantenerme sobre la pista, y las lejanas ayudan a empezar a girar antes de entrar en la curva. Si alguna fila no se ve, el error se calcula solo con las que sí.

### 4.4 Por qué tres centroides y no uno

La decisión más importante de la percepción fue no resumir toda la imagen en un único punto. Lo más sencillo habría sido calcular un solo centroide de toda la máscara roja y usar su desplazamiento como error, pero razoné que eso tenía varios problemas:

- **Mezcla información de zonas muy distintas.** Por la perspectiva, la línea ocupa mucha más imagen cerca del coche que lejos (se ve como un triángulo), así que un centroide global queda dominado por lo que ya tengo delante y apenas "ve" la curva que viene.
- **No distingue entre dos situaciones que necesitan respuestas opuestas.** Un coche descentrado en una recta y un coche centrado que se acerca a una curva pueden dar un error parecido, pero lo correcto es corregir el desplazamiento en el primer caso, y reducir la velocidad y empezar a girar en el segundo.
- **Reacciona tarde.** Solo se "entera" de la curva cuando ya está encima, y a alta velocidad eso es demasiado tarde.

Con tres filas a distancias distintas, cada una aporta algo diferente:

| Fila | Posición | Qué aporta | Inconveniente |
|---|---|---|---|
| `near` | 0.90 | **Dónde estoy ahora** (error lateral). Tiene muchos píxeles, así que es la medida más fiable | Reacciona tarde ante una curva |
| `mid` | 0.75 | Transición suave entre las otras dos y redundancia si alguna falla | Por sí sola aporta poca información nueva |
| `far` | 0.57 | **Hacia dónde va la línea**: permite anticipar | Pocos píxeles y más sensible al ruido: un píxel de error equivale a más distancia real |

**Un mismo dato, dos usos.** Las tres filas se combinan ponderadas en el error que recibe el PID, pero además la fila lejana se usa **por separado** en la velocidad adaptativa (`K_V_FAR`, apartado 7): es la que me avisa de que viene una curva y me permite frenar antes de entrar. Con un único punto no habría podido separar estas dos funciones, que son "corregir donde estoy" y "preparar lo que viene".

**Degradación gradual.** Como el error se calcula con las filas que sí se ven, perder una de ellas no significa perder la línea. Solo cuando ninguna fila ve la línea se pasa al estado de recuperación (apartado 8). Con un único centroide, cualquier fallo de detección habría disparado la recuperación directamente.

**Cada fila tiene su propia columna objetivo.** Como el desplazamiento de la cámara depende de la fila (apartado 5), cada una de las tres filas se compara con su `reference_x` y no con el centro de la imagen.

**Cómo elegí las posiciones y los pesos.** Las filas tienen que estar por debajo del horizonte (`HORIZON_FRAC = 0.50`). Coloqué la lejana (0.57) lo más cerca posible del horizonte sin que la banda tenga demasiado pocos píxeles (`MIN_BAND_PIXELS`), la cercana (0.90) cerca del borde inferior sin llegar a él, y la media entre ambas. Los pesos iniciales (0.5/0.3/0.2) priorizaban la fila cercana. Al subir la velocidad, el coche oscilaba y se abría en las curvas, y comprobé que **mirar más lejos (0.62 → 0.57) y dar más peso a esa fila (0.2 → 0.3)** reducía bastante la oscilación (apartado 11, iteración 2). La contrapartida es que dar más peso a la fila lejana también hereda más ruido, por lo que es un equilibrio que hay que afinar.

Calcular los centroides solo sobre tres bandas finas apenas añade coste frente al resto del procesado de la imagen.

---

## 5. Compensación de la cámara descentrada

Como la cámara está desplazada a la izquierda del eje del coche, cuando el coche está bien centrado sobre la línea, esta **no aparece en el centro de la imagen, sino a su derecha**. Si el PID intentase llevar la línea al centro de la imagen, el coche iría siempre descentrado.

La solución es que cada fila tenga su propia **columna objetivo** (`reference_x`) en vez de usar `width / 2`. Este desplazamiento no es el mismo en todas las filas, y se puede justificar con un modelo de cámara pinhole sobre un suelo plano: un punto del suelo a distancia lateral `d` y profundidad `Z` se proyecta en `u = f·d/Z`, y su fila verifica `v − v_h = f·h/Z` (con `v_h` la fila del horizonte y `h` la altura de la cámara). Combinando ambas:

```
u = d · (v − v_h) / h
```

Es decir, el desplazamiento en píxeles es **lineal con la distancia de la fila al horizonte**: cero en el horizonte y máximo en la parte inferior de la imagen. Lo implemento así:

```python
return width / 2.0 + CAM_OFFSET_FRAC * width * max(0.0, row_index - horizon_y) / (height - horizon_y)
```

`CAM_OFFSET_FRAC = 0.06` es el desplazamiento en la fila inferior como fracción del ancho de la imagen. En la imagen de depuración dibujo estos objetivos en amarillo, lo que permite comprobar visualmente si están bien colocados.

---

## 6. Control lateral: PID

El PID actúa sobre el error combinado, ya normalizado, y su salida es la **velocidad angular en rad/s**:

```
w = −( Kp·e + Ki·∫e dt + Kd·(de/dt) )
```

El signo negativo viene del convenio: si la línea está a la derecha (`e > 0`), el coche debe girar a la derecha, que corresponde a `w < 0`.

Valores finales: `KP = 1.6`, `KI = 0.05`, `KD = 0.35`. Detalles de implementación que me parecieron importantes:

- **Derivada filtrada.** El centroide en píxeles tiene mucho ruido de alta frecuencia. Si derivo directamente, el coche vibra. Mezclo la derivada nueva con la anterior con un filtro paso bajo (`D_ALPHA = 0.4`):
  ```python
  self.derivative_term = alpha * raw_derivative + (1.0 - alpha) * self.derivative_term
  ```
- **Integral limitada** a `±I_LIMIT = 0.5`, y con **anti-windup por integración condicional**: si la salida está saturada y el error sigue empujando en esa dirección, deshago la última integración. Así el término integral no crece sin control mientras el coche no puede girar más.
- **Saturación** de la salida a `W_MAX = 2.0 rad/s`.
- **`dt` real**, medido con `time.time()` en cada frame y no supuesto, para que integral y derivada sean correctas.
- Tengo `KI` muy pequeño a propósito: en este problema el PID es casi un PD.

---

## 7. Velocidad adaptativa

Si el coche va siempre a la misma velocidad, tiene que elegir entre ir despacio en todo el circuito o ir rápido y salirse en las curvas. Para evitarlo, la velocidad objetivo se calcula a partir de lo que veo:

```python
target_velocity = V_MAX - K_V_FAR * abs(far_error) - K_V_ERR * abs(control_error) - K_V_W * abs(angular_velocity) / W_MAX
target_velocity = clip_value(target_velocity, V_MIN, V_MAX)
```

Cada término penaliza una situación distinta:

- `K_V_FAR`: la línea lejana está desviada, es decir, **viene una curva**. Es lo que permite frenar antes de entrar.
- `K_V_ERR`: el coche está **descentrado** ahora mismo.
- `K_V_W`: el **giro es brusco**.

Después, el `SpeedPlanner` no aplica esa velocidad directamente, sino que limita la variación: **acelera suave** (`ACC = 5 m/s²`) y **frena fuerte** (`DEC = 15 m/s²`). La asimetría es deliberada: frenar tarde es lo que hace que el coche se salga, mientras que acelerar con calma después de una curva no cuesta mucho tiempo y evita desestabilizarlo.

---

## 8. Política de recuperación

La recuperación es una máquina de estados con tres estados:

| Estado | Cuándo se entra | Qué hace |
|---|---|---|
| `FOLLOW` | Línea visible de forma consistente | PID normal y velocidad adaptativa |
| `LOST_SHORT` | Se pierde la línea | Mantiene el último giro con decaimiento exponencial (`HOLD_TAU = 0.5 s`) y baja a `V_RECOVER = 1.5` |
| `SEARCH` | Lleva más de `T_LOST_SHORT = 0.4 s` sin verla | Gira hacia donde la vio por última vez a `W_SEARCH = 0.8` y `V_SEARCH = 1.0`; si en `T_SEARCH_FLIP = 2.5 s` no aparece, prueba el lado contrario |

```
          línea visible N_REACQ frames seguidos
     ┌─────────────────────────────────────────────┐
     ▼                                             │
  FOLLOW ──(pierdo la línea)──► LOST_SHORT ──(> 0.4 s)──► SEARCH
     ▲                              │                       │
     └──────────────────────────────┴───────────────────────┘
```

Detalles que me parecieron importantes:

- **Recordar la dirección del último giro** hace que la búsqueda tenga un punto de partida razonable en lugar de ser aleatoria.
- **Histéresis:** para volver a `FOLLOW` no basta con ver la línea un frame; hace falta verla durante `N_REACQ = 3` frames seguidos. Así se evita oscilar entre estados por un parpadeo de la detección.
- Si la línea reaparece durante la recuperación, el PID vuelve a actuar enseguida, pero a velocidad reducida hasta confirmar el regreso.
- Al volver a ver la línea tras haberla perdido, **reseteo el PID** para no arrastrar integral ni derivada de antes de la pérdida.
- Al volver a `FOLLOW`, la aceleración limitada del `SpeedPlanner` hace que la velocidad se recupere de forma progresiva.

---

## 9. Bucle principal y gestión del tiempo

El simulador web puede ir a una frecuencia distinta de mi bucle (50 Hz). Si proceso dos veces el mismo frame, el `dt` sería casi cero y la derivada del PID se dispararía. Por eso solo proceso un frame cuando es nuevo:

```python
if previous_image is None or not np.array_equal(current_image, previous_image):
```

Entre frames se mantienen las últimas velocidades enviadas. El `dt` se calcula con el tiempo real entre frames procesados y se limita al rango `[0.001, 0.2]` s para evitar valores absurdos.

---

## 10. Herramientas de depuración

Fue clave poder ver qué estaba percibiendo el coche:

- **Imagen de depuración** con `WebGUI.showImage()`: la máscara superpuesta en verde, el horizonte, las tres filas, el objetivo de cada fila (amarillo), la línea detectada (cian) y un texto con estado, `v`, `w` y error.
- **Trazas por consola** (`[TRACE]`) en cada transición de estado, y cada 2 s un resumen con el error medio, el error máximo y el porcentaje de tiempo en cada estado.

Este resumen me sirvió para comparar configuraciones con datos y no solo "a ojo".

---

## 11. Proceso de afinado de parámetros

Fui cambiando pocos parámetros cada vez y anotando el tiempo de vuelta del circuito simple y el comportamiento:

| # | Cambio | Tiempo | Comportamiento |
|---|---|---|---|
| 0 | Valores iniciales: `V_MAX=7`, `V_MIN=2.5`, `ACC=3`, fila lejana 0.62, pesos 0.5/0.3/0.2, `K_V_FAR=6`, `K_V_ERR=3` | ~100 s | Completa el circuito, pero lento |
| 1 | `V_MAX=11`, `V_MIN=3.5`, `ACC=5` | ~55 s | Casi el doble de rápido, pero en las curvas cerradas el coche "tiraba" hacia fuera y oscilaba bastante |
| 2 | Fila lejana 0.62 → 0.57 y pesos a 0.4/0.3/0.3 | ~60 s | Oscila mucho menos, pero sigue abriéndose en las curvas |
| 3 | `K_V_FAR=7`, `K_V_ERR=3.5` | ~65 s | Oscilación casi nula; todavía se abre un poco en las curvas |

Perdí unos segundos entre la iteración 1 y la 3, pero gané mucho en estabilidad. Como la robustez pesa más en la evaluación, me quedé con esta configuración.

### Parámetros finales

| Grupo | Parámetros |
|---|---|
| Percepción | `HORIZON_FRAC=0.50`, filas `0.90/0.75/0.57`, pesos `0.4/0.3/0.3`, `CAM_OFFSET_FRAC=0.06` |
| PID | `KP=1.6`, `KI=0.05`, `KD=0.35`, `D_ALPHA=0.4`, `I_LIMIT=0.5`, `W_MAX=2.0` |
| Velocidad | `V_MAX=11`, `V_MIN=3.5`, `K_V_FAR=7`, `K_V_ERR=3.5`, `K_V_W=2`, `ACC=5`, `DEC=15` |
| Recuperación | `V_RECOVER=1.5`, `V_SEARCH=1.0`, `W_SEARCH=0.8`, `T_LOST_SHORT=0.4`, `T_SEARCH_FLIP=2.5`, `HOLD_TAU=0.5`, `N_REACQ=3` |

### Resultados en los distintos circuitos

Una vez cerrada la configuración, probé el controlador en todos los circuitos disponibles en Unibotics. Estos son los tiempos aproximados de una **vuelta completa**:

| Circuito | Tiempo aproximado de vuelta |
|---|---|
| Simple | ~65 s |
| Montmelo | ~120 s |
| Nurburgring | ~160 s |
| Montreal | ~200 s |
| Monaco | ~320 s |

El controlador completa la vuelta en los cinco circuitos. La diferencia entre tiempos (de unos 65 s a unos 320 s) se debe sobre todo a que cada circuito tiene una longitud y un trazado distintos. Que el mismo código funcione en circuitos tan diferentes es lo que más me importaba, porque en la evaluación la robustez pesa más que la velocidad.

---

## 12. Limitaciones observadas y posibles mejoras

**Se sigue abriendo un poco en las curvas cerradas.** A velocidad alta el coche no es capaz de mantenerse del todo pegado a la curva, aunque no llega a perder la línea ni a oscilar. Mi hipótesis es que `W_MAX = 2.0` puede quedarse corto en las curvas más cerradas y el giro se sature, pero **no lo he medido**, así que lo dejo como hipótesis.

Ideas que valoré y no implementé, para no complicar el código más allá de lo que puedo explicar y mantener:

- **Contador de saturación de `w`** para comprobar la hipótesis anterior, y subir `W_MAX` si fuese necesario.
- **Frenado por curvatura**, penalizando `|e_far − e_near|` (la inclinación de la línea en la imagen), que reacciona antes que el error lejano solo.
- **Techo de velocidad adaptativo**: que cada vez que entre en `SEARCH` el coche reduzca su propio límite de velocidad y lo recupere poco a poco, como red de seguridad en circuitos distintos.
- **Escalado de las ganancias con la velocidad.**

---

## 13. Demostración en vídeo
[![Demostración Fórmula 1 Sigue-Líneas](/memorias/docs/p2/SimpleCircuit_10s.gif)]

---

## 14. Conclusiones

El resultado final es un controlador que sigue la línea de forma estable a alta velocidad: pasé de unos 100 s a unos 62 s por vuelta sin oscilaciones apreciables, con una recuperación que me permite no perder la carrera si el coche se desvía. Además, el mismo código completa los cinco circuitos disponibles (Simple, Montmelo, Nurburgring, Montreal y Monaco), con vueltas de entre unos 65 y unos 320 s según su longitud y trazado.

Lo que más he aprendido de esta práctica:

- **La percepción es la base de todo.** Detalles como los dos rangos de rojo en HSV o el uso de varias filas de mirada decidían si el control podía funcionar bien, antes siquiera de afinar el PID.
- **Un PID solo no basta para ir rápido.** Al subir la velocidad, el problema pasó de "seguir la línea" a "anticipar la curva", y la solución fue separar el control lateral (PID) del longitudinal (velocidad adaptativa).
- **Compensar un defecto del sensor por software** (la cámara descentrada) exige entender la geometría: el desplazamiento no es constante en la imagen, sino que depende de la perspectiva.
- **Afinar de uno en uno y registrar los datos** evita perderse. Con el resumen de estadísticas por consola podía ver si un cambio mejoraba o empeoraba de verdad.
- En un robot real, la velocidad máxima de giro y la calidad del sensor limitarían igual que aquí, y hay que medirlas para que el diseño teórico funcione en la práctica.