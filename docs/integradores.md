---
##  Documentación: Cuándo Usar Cada Integrador

La elección del integrador numérico es crítica, ya que determina la **estabilidad** y la **conservación de energía** de la simulación de física. La decisión se basa en el compromiso entre simplicidad, velocidad de cálculo y robustez.
---

## 1. Integrador de Euler Explícito (**`EULER_EXPLICIT`**)

Este método utiliza las propiedades del tiempo actual ($t$) para calcular el estado futuro ($t+\Delta t$). Es el más simple de implementar.

- **Lógica:** La posición se actualiza usando la velocidad antigua ($\mathbf{v}_{\text{old}}$), lo que introduce el error de que el cambio de aceleración no se tiene en cuenta inmediatamente.
- **Fórmula Clave:**
  $$\mathbf{x}_{t+\Delta t} = \mathbf{x}_t + \mathbf{v}_t \cdot \Delta t$$
  $$\mathbf{v}_{t+\Delta t} = \mathbf{v}_t + \mathbf{a}_t \cdot \Delta t$$
- **Estabilidad:** **Baja (Inestable).** La energía total del sistema tiende a **aumentar** con el tiempo, lo que provoca que las simulaciones se "inflen" o **exploten** (valores $\mathbf{x} \to \infty$).
- **Cuándo Usarlo:**
  - **NO RECOMENDADO** para la simulación de cuerpos rígidos o colisiones.
  - Solo se puede usar de forma segura si el paso de tiempo ($\Delta t$) es **extremadamente pequeño** y las fuerzas no son excesivas.

---

## 2. Integrador de Euler Semi-Implícito (**`EULER_SEMI_IMPLICIT`**)

También conocido como _Euler-Cromer_, este método es una ligera reordenación del Euler Explícito que ofrece una estabilidad dramáticamente mejorada.

- **Lógica:** La **velocidad se calcula primero** con la aceleración actual ($\mathbf{a}_t$), y luego esa **nueva velocidad** ($\mathbf{v}_{t+\Delta t}$) se usa para calcular la nueva posición. Esto introduce una pequeña amortiguación artificial.
- **Fórmulas Clave (Orden Importa):**
  $$\mathbf{v}_{t+\Delta t} = \mathbf{v}_t + \mathbf{a}_t \cdot \Delta t$$
  $$\mathbf{x}_{t+\Delta t} = \mathbf{x}_t + \mathbf{v}_{t+\Delta t} \cdot \Delta t$$
- **Estabilidad:** **Alta (Estable).** El método tiende a **disipar** (perder) energía muy lentamente con el tiempo, lo que se manifiesta como una pequeña amortiguación. Esto es generalmente deseable en los juegos.
- **Cuándo Usarlo:**
  - **ESTÁNDAR DE LA INDUSTRIA (Juegos):** Es el integrador preferido en la mayoría de los motores de física de tiempo real debido a su excelente equilibrio entre simplicidad, velocidad y estabilidad.

---

## 3. Integrador de Verlet por Posición (**`VERLET_POSITION`**)

El método de Verlet se distingue por no rastrear la velocidad directamente, sino que se basa en el historial de posiciones ($\mathbf{x}_t$ y $\mathbf{x}_{t-\Delta t}$).

- **Lógica:** Calcula la nueva posición ($\mathbf{x}_{t+\Delta t}$) a partir de las dos posiciones anteriores y la aceleración. La velocidad ($\mathbf{v}$) se infiere solo cuando es necesaria.
- **Fórmula Clave:**
  $$\mathbf{x}_{t+\Delta t} = 2\mathbf{x}_t - \mathbf{x}_{t-\Delta t} + \mathbf{a}_t \cdot \Delta t^2$$
- **Estabilidad:** **Excelente (Conservador de Energía).** La **energía total** del sistema se mantiene prácticamente constante (simetría temporal). No disipa ni amplifica la energía.
- **Cuándo Usarlo:**
  - **Simulaciones de Precisión:** Es ideal para sistemas donde la conservación de la energía a largo plazo es crítica, como **dinámica molecular, simulación de resortes, cuerdas o trayectorias orbitales**. Es robusto, pero requiere almacenar un estado extra (`posicion_previa`).
