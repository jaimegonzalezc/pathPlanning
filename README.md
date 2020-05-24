# Práctica 3 Path Planning

## Integrantes 

  * Ignacio Afuera Díaz
  * José María Berzal Gómez
  * Laura Bujalance Perez  
  * Jaime González Cárcamo
  * Elisa Gutiérrez Párraga
  
## Descripción del proyecto

En este proyecto grupal, vamos a utilizar distintos algoritmos de búsqueda informada y diferentes heurísticas para estimar el coste entre dos puntos, para obtener la ruta mas óptima de un robot. Este robot seguirá un plan definido por unos estados y acciones que haremos.
  
## Funcionamiento básico 

### Plan

En primer lugar debemos generar un plan, en el que por medio de un dominio y un problema, obtendremos una serie de acciones a realizar por nuestro robot. Para ello vamos a utilizar PDDL (Planning Domain Definition Language), que es un lenguaje creado para la representación de tareas de planificación.

En primer lugar vamos a ver el dominio, en el que definimos los predicados y las acciones que el robot puede realizar.
```
(define (domain planetary-explorations)
(:requirements :typing :strips :fluents :durative-actions )
(:types rover point planet direction samples panels speed)
(:predicates 
(is_on ?rover -rover ?point -point)
(is_communicating? ?rover -rover)
(is_drilling? ?rover -rover)
(is_taking_pictures? ?rover -rover)
(is_analizing? ?rover -rover)
(is_recharging? ?rover -rover)
(is_moving? ?rover -rover)
(has_comunicated ?rover -rover ?point -point)
(has_drilled ?rover -rover ?point -point)
(has_taken_picture ?rover -rover ?point -point)
(has_analized ?rover -rover ?point -point)
(has_extended_solar_panels ?rover -rover)
)

(:functions (battery-level ?rover -rover)
(distance ?x ?y -point)
(speed ?rover -rover)
(total_battery_used ?rover))


(:durative-action move 
    :parameters (?x ?y -point ?rover -rover)
    :duration (= ?duration (/ (distance ?x ?y) (speed ?rover)))
    :condition (and (over all (not (has_extended_solar_panels ?rover))) (over all (not (is_recharging? ?rover)))(over all(not (is_taking_pictures? ?rover)))(over all(not (is_drilling? ?rover)))(over all(not(is_communicating? ?rover)))
    (over all(not (is_analizing? ?rover)))(at start(is_on ?rover ?x)) (at start(> (battery-level ?rover) (* (speed ?rover) (/ (distance ?x ?y) (speed ?rover))))))
    :effect (and(at end(decrease (battery-level ?rover) (* (speed ?rover) (/ (distance ?x ?y) (speed ?rover))))) (at end(is_on ?rover ?y))(at end(not(is_moving? ?rover)))(at start (not (is_on ?rover ?x))) (at start (is_moving? ?rover)) (at end (increase (total_battery_used ?rover) 10)))
)

(:durative-action extend-solar-panels
    :parameters (?point -point ?rover -rover)
    :duration (= ?duration 3)
    :condition (and(over all(is_on ?rover ?point))(over all(not(is_moving? ?rover))))
    :effect  (at end(has_extended_solar_panels ?rover))
)
(:durative-action retract-solar-panels
    :parameters (?point -point ?rover -rover)
    :duration (= ?duration 3)
    :condition (and(over all(is_on ?rover ?point))(over all(not(is_moving? ?rover))))
    :effect  (at end(not (has_extended_solar_panels ?rover)))
)

(:durative-action recharge
    :parameters (?rover -rover)  
    :duration (= ?duration 10)
    :condition (and (over all(not (is_moving? ?rover)))(over all (has_extended_solar_panels ?rover)))
    :effect (and( at end(increase(battery-level ?rover) 20)) (at start (is_recharging? ?rover)) (at end(not (is_recharging? ?rover))))
)

(:durative-action take-pictures
    :parameters (?point -point ?rover -rover)
    :duration (= ?duration 5)
    :condition (and(over all(is_on ?rover ?point))(over all(not(is_moving? ?rover))))
    :effect (and (at start(is_taking_pictures? ?rover))(at end (not (is_taking_pictures? ?rover)))(at end (has_taken_picture ?rover ?point)))
)            

(:durative-action drilling
    :parameters (?point -point ?rover -rover)
    :duration (= ?duration 15)
    :condition (and(over all(is_on ?rover ?point))(over all(not(is_moving? ?rover))))
    :effect (and (at start(is_drilling? ?rover))(at end (not (is_drilling? ?rover)))(at end (has_drilled ?rover ?point)))
)

(:durative-action analyize_samples
    :parameters (?rover -rover ?point -point)
    :duration (= ?duration 8)
    :condition (and(over all(is_on ?rover ?point)) (over all(not(is_moving? ?rover))))
    :effect (and (at start (is_analizing? ?rover)) (at end(not(is_analizing? ?rover)))(at end(has_analized ?rover ?point)))
)

(:durative-action earth-comunnication
    :parameters (?rover -rover ?point -point)
    :duration (= ?duration 20)
    :condition (and(over all(is_on ?rover ?point))(over all(not(is_moving? ?rover))))
    :effect (and(at start(is_communicating? ?rover)) (at end (not(is_communicating? ?rover)))(at end (has_comunicated ?rover ?point))))
)
```
Como se puede observar, los predicados definen un estado y las acciones toman esos predicados como precondiciones para modificar estos predicados, y por tanto el estado en el que se encuentra el robot.

También, tenemos funciones, que alamcenan un valor que puede ser modificado por las acciones. Estas funciones almacenan por ejemplo la distancia entre dos puntos o el nivel de la bateria del robot.

Depués, se debe definir un problema, en el que expondremos unas condiciones iniciales (estado inicial) y una serie de predicados finales (estado meta).

```
(define (problem problem-planetary-explorations)
    (:domain planetary-explorations)
    (:objects
        r1 - rover
        P1004 P1322 P0508 P1033 - point
    )
    (:init
        (= (battery-level r1) 20)
        (is_on r1 P1004)
        (=(total_battery_used r1) 0)
        (= (distance P1004 P1322) 5)
        (= (distance P1004 P0508) 10)
        (= (distance P1004 P1033) 15)
        (= (distance P1322 P1004) 5)
        (= (distance P1322 P0508) 5)
        (= (distance P1322 P1033) 10)
        (= (distance P0508 P1004) 10)
        (= (distance P0508 P1322) 5)
        (= (distance P0508 P1033) 5)
        (= (distance P1033 P1004) 30)
        (= (distance P1033 P1322) 20)
        (= (distance P1033 P0508) 5)
        (= (speed r1) 1)
    )
    (:goal (and(is_on r1 P1004)(has_drilled r1 P1004) (has_analized r1 P1033)(has_comunicated r1 P1004)(has_drilled r1 P2213)       (has_analized r1 P0508)(has_taken_picture r1 P1033)))
    (:metric minimize (total_battery_used r1))
)

```

Lo más destacable del problema, es que los objetos point tienen el formato P-XX-YY donde XX es la coordenada x y YY es la coorenada y.

El resultado del plan lo almacenaremos en el fichero ```planning.txt``` para utilizarlo en nuestro proyecto y calcular las rutas más óptimas.

``` 
0.001: (DRILLING P1004 R1) [15.0000]
0.002: (EARTH-COMUNNICATION R1 P1004) [20.0000]
20.003: (MOVE P1004 P1322 R1) [5.0000]
25.004: (DRILLING P1322 R1) [15.0000]
40.005: (MOVE P1322 P0508 R1) [5.0000]
45.006: (ANALYIZE_SAMPLES R1 P0508) [8.0000]
53.007: (MOVE P0508 P1033 R1) [5.0000]
58.008: (ANALYIZE_SAMPLES R1 P1033) [8.0000]
58.009: (TAKE-PICTURES P1033 R1) [5.0000]
58.010: (EXTEND-SOLAR-PANELS P1033 R1) [3.0000]
61.011: (RECHARGE R1) [10.0000]
71.012: (RETRACT-SOLAR-PANELS P1033 R1) [3.0000]
74.013: (MOVE P1033 P0508 R1) [5.0000]
79.014: (MOVE P0508 P1004 R1) [10.0000]
```
