
# Быстрее быстрого: примитивы, использующие GPU, для высокоскоростной визуально-инерциальной одометрии 

Перевод работы [Faster than FAST: GPU-Accelerated Frontend for
High-Speed VIO](http://rpg.ifi.uzh.ch/docs/Arxiv20_Nagy.pdf)

## О работе


### Авторы
```
Balazs Nagy, Philipp Foehn, Davide Scaramuzza
```

## Краткое описание

Недавнее появление мощных встраиваемых GPU 
резко улучшило приложения
машинного зрения в реальном времени.
Это позволило выполнять прямо на борту алгоритмы с гораздо
большей частотой кадров, чем обычно, и с меньшей
задержкой.
В работе рассмотрена применимость эффективных низкоуровневых
аппаратно-специфических инструкций GPU для улучшения имеющихся
алгоритмов машинного зрения, используемых при визуально-инерциальной одометрии (VIO).
Поскольку большинство шагов VIO работает с визуальными особенностями, они зависят от данных изображения для
поиска и отслеживания, и оба этих шага отлично подходят для параллелизации. Подавление немаксимумов и последующий выбор особенностей дают заметную добавку в задержку
обработки кадра. Мы сперва пересматриваем задачу
подавления немаксимумов для выбора особенностей на GPU и предлагаем решение, которое выбирает локальный максимум с требуемым пространственным распределением, и одновременно извлекает особенности. Наш второй вклад - улучшенный детектор особенностей FAST, который применяет вышеописанный метод подавления немаксимумов. Наконец, мы сравниваем наш метод с другими современными реализациями на CPU и GPU, где мы превосходим
их в поиске и отслеживании особенностей, получая более 
1000 кадров в секунду на миникомпьютере Jetson TX2.
Также, мы демонстрируем нашу работу в составе
конвеера VIO, достигая частоты оценки состояния около
200 раз в секунду.

Исходный код доступен: [http://github.com/uzh-rpg/vilib](http://github.com/uzh-rpg/vilib)

*[VO]: Visual-Inertial Odometry - визуально-инерциальная одометрия


## Список литературы

1. C. Harris and M. Stephens. A combined corner and edge detector. In
In Proc. of Fourth Alvey Vision Conference, 1988.
2. Jianbo S. and Carlo T. Good features to track. IEEE Conf. Comput. Vis.
Pattern Recog. (CVPR), 1994.
3. E. Rosten and T. Drummond. Machine learning for high-speed corner
detection. In Eur. Conf. Comput. Vis. (ECCV), 2006.
4. E. Rublee, V. Rabaud, K. Konolige, and G. Bradski. Orb: An efficient
alternative to sift or surf. In Int. Conf. Comput. Vis. (ICCV), 2011.
5. David G. Lowe. Distinctive image features from scale-invariant keypoints. Int. J. Comput. Vis., 2004.
6. H. Bay, A. Ess, T. Tuytelaars, and L. Van Gool. Speeded-up robust
features (surf). Comput. Vis. Image. Und., 2008.
7. E. Rosten, R. Porter, and T. Drummond. Faster and better: A machine
learning approach to corner detection. IEEE Trans. Pattern Anal. Mach.
Intell., 2010.
8. K. Omar. Kfast: vectorized x86 cpu implementation of the fast feature
detector, 2006.
9. G. Bradski. The OpenCV Library. Dr. Dobb’s Journal of Software
Tools, 2000.
10. P. Yalamanchili, U. Arshad, Z. Mohammed, P. Garigipati, P. Entschev,
B. Kloppenborg, J. Malcolm, and J. Melonakos. ArrayFire - A high
performance software library for parallel computing with an easy-to-use
API, 2015.
11. A. Neubeck and L. Van Gool. Efficient non-maximum suppression. In
IEEE Int. Conf. Pattern Recog. (ICPR), 2006.
12. Tuan Q. Pham. Non-maximum suppression using fewer than two
comparisons per pixel. In Advanced Concepts for Intelligent Vision
Systems ACIVS, 2010.
13. D. Oro, C. Fernandez, X. Martorell, and J. Hernando. Work-efficient ´
parallel non-maximum suppression for embedded gpu architectures.
In IEEE International Conference on Acoustics, Speech and Signal
Processing (ICASSP), 2016.
14. W. Forstner and E. G ¨ ulch. A fast operator for detection and precise ¨
location of distinct point, corners and centres of circular features. In
Proceedings of the ISPRS Conference on Fast Processing of Photogrammetric Data, 1987.
15. A. I. Mourikis and S. I. Roumeliotis. A multi-state constraint kalman
filter for vision-aided inertial navigation. In IEEE Int. Conf. Robot.
Autom. (ICRA), 2007.
16. S. Leutenegger, S. Lynen, M. Bosse, R. Siegwart, and P. Furgale.
Keyframe-based visual-inertial odometry using nonlinear optimization.
Int. J. Robot. Research, 2015.
17. Raul Mur-Artal, Jos ´ e M. M. Montiel, and Juan D. Tard ´ os. ORB-SLAM: ´
a versatile and accurate monocular SLAM system. IEEE Trans. Robot.,
2015.
18. M. Bloesch, S. Omari, M. Hutter, and R. Siegwart. Robust visual inertial
odometry using a direct EKF-based approach. In IEEE/RSJ Int. Conf.
Intell. Robot. Syst. (IROS), 2015.
19. B. Lucas and T. Kanade. An iterative image registration technique
with an application to stereo vision. In Int. Joint Conf. Artificial Intell.
(IJCAI), 1981.
20. S. Baker and I. Matthews. Lucas-kanade 20 years on: A unifying
framework: Part 1. Int. J. Comput. Vis., 2002.
21. S. Baker, R. Gross, and I. Matthews. Lucas-kanade 20 years on: A
unifying framework: Part 3. Int. J. Comput. Vis., 2003.
22. C. Forster, M. Pizzoli, and D. Scaramuzza. Svo: Fast semi-direct
monocular visual odometry. In IEEE Int. Conf. Robot. Autom. (ICRA),
2014.
23. C. Forster, Z. Zhang, M. Gassner, M. Werlberger, and D. Scaramuzza.
SVO: semidirect visual odometry for monocular and multicamera systems. IEEE Trans. Robot., 2017.
24. C. Zach, D. Gallup, and J. Frahm. Fast gain-adaptive klt tracking on the
gpu. In IEEE Conf. Comput. Vis. Pattern Recog. Workshops (CVPRW),
2008.
25. J.S. Kim, M. Hwangbo, and T. Kanade. Realtime affine-photometric klt
feature tracker on gpu in cuda framework. In Int. Conf. Comput. Vis.
Workshops (ICCVW), 2009.
26. M. Hwangbo, J. Kim, and T. Kanade. Inertial-aided klt feature tracking
for a moving camera. In IEEE/RSJ Int. Conf. Intell. Robot. Syst. (IROS),
2009.
27. M. Burri, J. Nikolic, P. Gohl, T. Schneider, J. Rehder, S. Omari, M.W.
Achtelik, and R. Siegwart. The EuRoC micro aerial vehicle datasets.
Int. J. Robot. Research, 2015.
28. NVIDIA Corporation. CUDA C++ Programming Guide, 2019.
29. D. Nister, O. Naroditsky, and J. Bergen. Visual odometry. In IEEE Conf.
Comput. Vis. Pattern Recog. (CVPR), 2004.
30. D. Scaramuzza, F. Fraundorfer, and R. Siegwart. Real-time monocular
visual odometry for on-road vehicles with 1-point ransac. In IEEE Int.
Conf. Robot. Autom. (ICRA), 2009.
31. F. Fraundorfer and D. Scaramuzza. Visual odometry : Part ii: Matching,
robustness, optimization, and applications. IEEE Robot. Autom. Mag.,
2012.
32. NVIDIA Corporation. Developer Blog - Faster Parallel
Reductions on Kepler, 2014. https://devblogs.nvidia.com/
faster-parallel-reductions-kepler/.
33. H. Liu, M. Chen, G. Zhang, H. Bao, and Y. Bao. Ice-ba: Incremental,
consistent and efficient bundle adjustment for visual-inertial slam. In
IEEE Conf. Comput. Vis. Pattern Recog. (CVPR), 2018.
