# act1_exp3
## Asignatura: 
"IC422 Sistemas Embebidos"
## Actividad: 
Actividad Experimental Nº1: Manejo de Puertos, Temporizadores e Interrupciones en Microcontroladores ATmega2560 y ATmega328p.
## Consigna:
**Experiencia 3 (contador de packs de yerba)**

En una planta elaboradora de yerba mate, los packs de paquetes van por una cinta transportadora desde el sector de embalaje al depósito. Con el fin de controlar la producción, se requiere contar la cantidad de packs y en caso de superarse cierto valor umbral (configurable), debe indicarse esto mediante una alarma sonora accionada a través de un relé. La alarma debe sonar durante 10 segundos. En base a lo mencionado, diseñar un sistema embebido que permita realizar la tarea indicada, siendo el diagrama de bloques de su circuito el mostrado en la Fig. 3. El sistema de esta figura posee tres pulsadores y un display conformado por tres dígitos, la función de cada uno de estos deberá ser la siguiente:
- P1: En el modo configuración del sistema, accionado este pulsador el usuario puede configurar el valor umbral. El mismo puede estar entre 50 y 400. Durante el transcurso del conteo de packs, el accionamiento de P1 resetea la cantidad contada.
- P2: En el modo configuración del sistema, accionando este pulsador el usuario puede habilitar el conteo. Durante el transcurso del conteo de packs, el accionamiento de este pulsador permite detener la cuenta. Si la cuenta está detenida, el accionamiento de P2 reanuda el conteo.
- P3: Durante el modo configuración del sistema, el accionamiento de este pulsador no tiene efecto en el conteo. Una vez habilitado el conteo mediante P2, con cada flanco descendente generado a través de P3, el conteo se incrementa en una unidad (P3 simula ser el detector de packs).
- Display: En el modo configuración, el display permite mostrar el incremento del valor umbral cuando es configurado por el usuario mediante P1. Durante el conteo, el display indica la cantidad de packs contados. Si se alcanza el valor de umbral configurado, el conteo no se resetea y sigue incrementándose. Cuando llega a 999, vuelve a contar de 000.
