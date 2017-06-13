# Устойчивый модульный фильтр, объединяющий измерения множества датчиков, для навигации летательных аппаратов

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

## Обработка относительных измерений

Формулировка уточнений на основе данных от камер,
описанная в разделе III, имеет недостатки при использовании результатов
работы алгоритмов визуальной одометрии: последняя оценка визуального
масштаба используется для масштабирования всего пройденного пути,
не учитывая промежуточных изменений масштаба. 

## Список литературы

1. G. Chowdhary, E. Johnson, D. Magree, D. Wu, and A. Shein. GPS-
Denied Indoor and Outdoor Monocular Vision Aided Navigation and
Control of Unmanned Aircraft. Journal of Field Robotics (JFR), 2013.
2. M. A. Fischler and R. C. Bolles. Random sample consensus: a
paradigm for model fitting with applications to image analysis and
automated cartography. Communications of the ACM, 1981.
3. S. Grzonka, G. Grisetti, and W. Burgard. Towards a navigation system
for autonomous indoor flying. In Proc. of the IEEE Int. Conf. on
Robotics and Automation (ICRA), Kobe, Japan, 2009. IEEE.
4. R. Hartley and A. Zisserman. Multiple View Geometry in Computer
Vision. Cambridge University Press, second edition, 2004.
5. J. A. Hesch, D. G. Kottas, S. L. Bowman, and S. I. Roumeliotis.
Towards consistent vision-aided inertial navigation. In Algorithmic
Foundations of Robotics X. Springer, 2013.
6. V. Indelman, S. Williams, M. Kaess, and F. Dellaert. Factor graph
based incremental smoothing in inertial navigation systems. In
Information Fusion (FUSION), 2012 15th International Conference
on. IEEE, 2012.
7. G. Klein. Visual Tracking for Augmented Reality. PhD thesis,
University of Cambridge, 2006.
8. S. Leutenegger and R. Y. Siegwart. A low-cost and fail-safe inertial
navigation system for airplanes. In Proc. of the IEEE Int. Conf. on
Robotics and Automation (ICRA), Karlsruhe, Germany, 2012. IEEE.
9. M. Li and A. I. Mourikis. High-precision, consistent EKF-based
visual–inertial odometry. The International Journal of Robotics
Research, (IJRR), 2013.
10. F. M. Mirzaei and S. I. Roumeliotis. A kalman filter-based algorithm
for imu-camera calibration: Observability analysis and performance
evaluation. Robotics, IEEE Transactions on, 24(5):1143–1156, 2008.
11. A. Mourikis and S. Roumeliotis. A multi-state constraint Kalman filter
for vision-aided inertial navigation. In Proc. of the IEEE Int. Conf.
on Robotics and Automation (ICRA), 2007
12. A. I. Mourikis, S. I. Roumeliotis, and J. W. Burdick. SC-KF mobile
robot localization: a stochastic cloning Kalman filter for processing
relative-state measurements. Robotics, IEEE Transactions on, 2007.
13. S. Shen, N. Michael, and V. Kumar. Autonomous multi-floor indoor
navigation with a computationally constrained mav. In Proc. of the
IEEE Int. Conf. on Robotics and Automation (ICRA). IEEE, 2011.
14. S. Shen, Y. Mulgaonkar, N. Michael, and V. Kumar. Vision-based
state estimation and trajectory control towards aggressive flight with
a quadrotor. In Robotics Science and Systems, 2013.
15. S. Shen, Y. Mulgaonkar, N. Michael, and V. Kumar. Vision-based state
estimation for autonomous rotorcraft MAVs in complex environments.
In Proc. of the IEEE Int. Conf. on Robotics and Automation (ICRA),
2013.
16. S. Weiss. Vision based navigation for micro helicopters. PhD thesis,
ETH Zurich, 2012.
17. S. Weiss, M. W. Achtelik, S. Lynen, M. Chli, and R. Siegwart. Real-
time onboard visual-inertial state estimation and self-calibration of
MAVs in unknown environments. In Proc. of the IEEE Int. Conf. on
Robotics and Automation (ICRA), 2012.
