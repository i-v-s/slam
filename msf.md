# Устойчивый модульный алгоритм объедениения данных от нескольких датчиков для навигации летательных аппаратов

Перевод работы [A Robust and Modular Multi-Sensor Fusion Approach Applied to MAV Navigation](https://github.com/ethz-asl/ethzasl_msf/raw/master/2013_IROS_lynen_modular_sensor_fusion.pdf)

## О работе

### Авторы

```
Simon Lynen 1 , Markus W. Achtelik 1 , Stephan Weiss 2 , Margarita Chli 1 and Roland Siegwart 1
```

## Краткое описание

Давно известно, что использование информации нескольких датчиков
для навигации роботов увеличивает устойчивость и точность.
Однако, точная калибровка группы датчиков перед использованием .
Поэтому, для упрощения, большинство систем не используют всю
доступную информацию от датчиков. Например, на задачах, требующих
перемещения робота из помещения наружу, не используются данные GPS,
которые доступны только на открытом пространстве, а вместо них
навигация основывается только на датчиках (например, визуальных и лазерных),
которые доступны на протяжении задачи. Naturally, this comes at
the expense of robustness and accuracy in real deployment.
This paper presents a generic framework, dubbed Multi-
Sensor-Fusion Extended Kalman Filter (MSF-EKF), able to
process delayed, relative and absolute measurements from a
theoretically unlimited number of different sensors and sensor
types, while allowing self-calibration of the sensor-suite online.
The modularity of MSF-EKF allows seamless handling of
additional/lost sensor signals during operation while employing
a state buffering scheme augmented with Iterated EKF (IEKF)
updates to allow for efficient re-linearization of the prediction
to get near optimal linearization points for both absolute and
relative state updates. We demonstrate our approach in outdoor
navigation experiments using a Micro Aerial Vehicle (MAV)
equipped with a GPS receiver as well as visual, inertial, and
pressure sensors.

## Введение

Основываясь на наших предыдущих работах [16, 17],
мы предлагаем обобщённую систему оценки состояния
с открытым исходным кодом на C++, которая содержит:

- поддержку модулей для неограниченного числа датчиков,
измеряющих как относительные так и абсолютные смещения.
- оценку состояния калибровки между датчиками и динамическую
компенсацию задержки измерения
- Релинеаризацию ограничений как от внутренних так и от внешних
источников информации в фильтре
- эффективное отслеживание взаимной ковариации для относительных
обновлений, позволяющее производить оценку с частотой в несколько КГц. 

После анализа ограничений нашей предыдущей работы,
мы покажем как использовать относительные положения,
полученные от систем локализации с построением карты (SLAM),
которые необходимы при применении визуальных или лазерных одометрических датчиков .
Завершая, мы продемонстриуем MSF-EKF в реальных экспериментах,
с протяжённостью траекторий более 800 метров при скорости до 4 м/с. 

A, B, C...

## Сопряжение нескольких датчиков

Наша система основывается на непрямой формулировке итерируемого
расширенного фильтра Калмана (EKF), когда предсказание состояния
происходит при каждом измерении инерционных датчиков (IMU).
Полное состояние состоит из нескольких *базовых* состояний:

 <div id="e1">$$
\begin{equation}
x_{core}^{T} = [
    {p_{\omega}^{i}}^{T},
    {v_{\omega}^{i}}^{T},
    {q_{\omega}^{i}}^{T},
    b_{\omega}^{T},
    b_{a}^{T}    
]
.\tag{1}\label{eq:one}
\end{equation} 
$$</div>

Они соответствуют относительной позиции $${p_{\omega}^{i}}$$,
скорости $${v_{\omega}^{i}}$$ и
положению (вращению) $${q_{\omega}^{i}}$$
модуля инерционных измерений относительно глобальной
системы координат, определённых в глобальной системе координат.
Далее, мы оценим систематические погрешности акселерометра $$b_{a}$$ и
гироскопа $$b_{\omega}$$. Затем, могут быть добавлены дополнительные датчики,
измеряющие относительно системы координат модуля инерционных измерений.

