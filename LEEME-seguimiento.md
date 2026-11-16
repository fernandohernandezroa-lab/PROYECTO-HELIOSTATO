# Heliostato v5: seguimiento solar experimental

Abre `heliostato_v5/heliostato_v5.ino` en Arduino IDE. La carpeta y el archivo deben conservar el mismo nombre. Selecciona tu modelo real de ESP32 y realiza tú la compilación y las pruebas. Esta entrega no se ha cargado en la placa. El intento de compilación local no pudo iniciar el compilador ESP32 instalado por un error de DLL de Windows; no hay compilación de firmware confirmada.

## Uso

1. Nivela físicamente el espejo y establece el origen. Pulsa Habilitar.
2. En Seguimiento solar, abre Lugar y receptor y completa la configuración.
3. Revisa Ejes y calibración. Confirma los sentidos y el modelo de inclinaciones combinadas cuando los hayas comprobado físicamente.
4. Selecciona inicio al activar o una fecha y hora dentro de las próximas 24 horas. La fecha ingresada utiliza la zona horaria del teléfono/computador, indicada en pantalla. La tabla se muestra en America/Santiago. El reloj del dispositivo debe ser correcto.
5. La duración inicial es de 60 minutos. Puede configurarse de 5 a 180 minutos, en múltiplos de cinco. El intervalo siempre es de cinco minutos.
6. Pulsa Revisar recorrido. Si cualquier punto queda fuera del rango, no se permite ejecutar el plan; no se recortan los ángulos para forzarlo a caber.
7. Pulsa Activar seguimiento. La opción Al activar deja diez segundos para transferir el plan. Para una hora hay 13 posiciones: minutos 0, 5, 10…60.
8. Mantén la página de control abierta, el dispositivo despierto y conectado a la red del ESP32. El plan se ejecuta en el ESP32 con un reloj monotónico, pero conserva la parada por falta de comunicación del código original. No es seguimiento desatendido.
9. Detener cancela el plan y conserva el torque. Habilitar no reanuda una prueba cancelada: hay que programarla de nuevo. Al concluir se conserva la última posición; no se vuelve automáticamente al origen.

Las horas de la tabla indican cuándo se inicia cada ajuste, no cuándo termina. Se conserva la velocidad de 10 pulsos/s y el movimiento secuencial de ambos motores. La primera colocación puede demorar más de un minuto. El último ajuste se inicia en el minuto 60 y el programa declara la prueba terminada cuando ese movimiento concluye. Durante el desplazamiento y entre ajustes existe error de apuntado. No se ha implementado una precolocación antes del inicio del periodo.

## Datos configurables

| Campo | Significado y valor inicial |
| --- | --- |
| Latitud / longitud | -39.82 / -73.24, valores aproximados de Valdivia, no un levantamiento del edificio 14K. Reemplazar por el punto de instalación. |
| Rumbo del frente | 270°, oeste provisional. Es la dirección desde la estructura hacia quien toma la foto frontal. En esta instalación mira al receptor. |
| Azimut al receptor | 270°, oeste provisional. Dirección desde el centro del espejo al punto deseado en la pared. |
| Distancia horizontal | 6 m aproximados. No es la distancia inclinada al receptor. |
| Diferencia de altura | Obligatoria, sin valor inventado: altura del punto de luz menos altura del centro del espejo. Positiva si está por encima. |
| Motor 1 positivo | Baja el borde derecho visto en la foto frontal. Configurable. |
| Motor 2 positivo | Baja el borde trasero, más alejado de quien toma la foto. Configurable. |

Todos los azimuts se miden desde el norte verdadero, en sentido horario: N 0°, E 90°, S 180°, O 270°. Una brújula magnética requiere corregir la declinación local. Con el frente al oeste, el borde derecho visto en la foto frontal queda hacia el sur y la parte trasera hacia el este.

Los campos se transfieren al activar el plan; cambiarlos durante una prueba no modifica el plan ya recibido. La configuración del formulario no se guarda en memoria permanente. Un reinicio del ESP32 cancela el plan y exige establecer nuevamente el origen.

## Modelo y límites

El cálculo obtiene la dirección hacia el Sol `s` y la dirección hacia el receptor `t`. La normal buscada es `n = normalizar(s + t)`, la bisectriz que satisface la reflexión especular. La tabla distingue azimut/elevación del Sol y azimut/elevación de la normal. En el origen la normal es vertical y su elevación es 90°; su azimut es indeterminado si es exactamente vertical.

Para traducir la normal a inclinaciones se utiliza un **modelo provisional de pendientes de un plano**, no una cinemática medida de la cruceta. Sea `f` la dirección horizontal del frente y `r` la derecha vista en la foto:

```
inclinacion1 = atan2(n · r, n_vertical)
inclinacion2 = -atan2(n · f, n_vertical)
```

Los signos se invierten si se cambia la configuración de sentidos. Las inclinaciones pasan después por la tabla experimental original. Esta relación es válida si los grados de la tabla representan las inclinaciones de las secciones derecha–izquierda y frente–atrás. Si son ángulos de un mecanismo con ejes de rotación encadenados o existe acoplamiento por cables, se debe sustituir `orientarEspejo()` por la cinemática real o una calibración bidimensional. La foto y las mediciones de cada eje por separado no confirman el comportamiento con ambos inclinados. No se garantiza apuntado físico con ese modelo sin comprobarlo.

Se conserva la tabla media para ambos motores, los pines, 800 pulsos/vuelta del driver, 10 pulsos/s, límites absolutos de ±750 pulsos, el control manual, los destinos angulares y los retornos al origen. Aunque la mecánica supere 30°, el seguimiento admite solamente ±30° por eje, el rango de la tabla actual. El sentido no medido se estima por simetría. No hay encoder, finales de carrera ni corrección por visión.

El error informado de aproximadamente 0,5° en el espejo puede producir un error del orden de 1° en el rayo en una sección de reflexión, unos 10 cm a 6 m en condiciones favorables. Es una estimación geométrica, no una precisión garantizada; se añaden los errores de orientación, holgura, simetría, acoplamiento y el cambio solar durante los cinco minutos entre ajustes.

Se rechazan posiciones con elevación solar inferior a 5° y geometrías no válidas. No se calcula refracción atmosférica, obstáculos, sombra del edificio, deformación del espejo ni movimiento del punto central del espejo. El lugar y el receptor se consideran fijos. El plan completo se valida antes de emitir pulsos. También se detiene si un ajuste llega tarde más de dos segundos o el movimiento anterior no ha terminado.

## Ajustes al control existente

- La parada cancela también el seguimiento y las órdenes manuales se bloquean durante el plan.
- El watchdog queda asociado a la IP y a una sesión de pestaña: otra pestaña observadora no mantiene vivo el control que se perdió. Esto no sustituye autenticación de red.
- Cuando un movimiento manual alcanza su destino, se cierra el estado jog para evitar una caducidad innecesaria después de llegar.
- Las flechas indican el borde que baja, visto desde la foto frontal: derecha/izquierda controlan M1; arriba/abajo controlan borde trasero/delantero de M2. Los botones tienen etiquetas accesibles.
- La interfaz utiliza una hoja de estilos sencilla; la geometría y la calibración quedan en secciones desplegables.

El código conserva la red Wi-Fi y la clave que tenía tu versión original. No cambies los pines ni la configuración física de los drivers sin revisar la calibración.

## Fuente del cálculo solar

Algoritmo aproximado del [U.S. Naval Observatory](https://aa.usno.navy.mil/faq/sun_approx), con cálculo en UTC y conversión a coordenadas horizontales locales. Se ejecuta en el navegador sin depender de Internet. El firmware recibe posiciones de motor ya calculadas y valida sus límites; no verifica por sí mismo las coordenadas del lugar ni vuelve a calcular la astronomía.

La comprobación final de compilación, signos, inclinaciones combinadas y funcionamiento físico queda a cargo del usuario, según lo solicitado. Esta versión es experimental.

