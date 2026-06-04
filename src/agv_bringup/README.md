# agv_bringup — Единый запуск системы беспилотного трактора

Пакет содержит главный launch-файл, который запускает **все подсистемы** трактора
в правильном порядке с задержками через `TimerAction`.

## Последовательность запуска

| Время | Подсистема | Пакет |
|-------|-----------|-------|
| t = 0с | Gazebo Classic (headless) + робот | `agv_gazebo` |
| t = 5с | Обработка сенсоров (EKF, лидар) | `agv_sensor_processing` |
| t = 10с | Навигация (Nav2, SLAM, покрытие) | `agv_navigation` |
| t = 13с | Модуль решений (FSM) | `agv_decision` |
| t = 13с | RViz2 (опционально) | `agv_visualization` |

## Быстрый старт

### 1. Собрать все пакеты (в Ubuntu, в терминале VM)

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Запуск всей системы

**Без визуализации** (рекомендуется при нехватке RAM):
```bash
ros2 launch agv_bringup full_system.launch.py
```

**С RViz2** (визуализация трактора, лидара, карты):
```bash
ros2 launch agv_bringup full_system.launch.py rviz:=true
```

**Без планировщика покрытия** (только навигация по точкам):
```bash
ros2 launch agv_bringup full_system.launch.py coverage:=false rviz:=true
```

**Все параметры:**
```bash
ros2 launch agv_bringup full_system.launch.py \
  rviz:=true \
  coverage:=true \
  slam:=true \
  use_sim_time:=true
```

## Параметры командной строки

| Параметр | По умолчанию | Описание |
|----------|-------------|----------|
| `rviz` | `false` | Запустить RViz2 для визуализации |
| `coverage` | `true` | Запустить планировщик покрытия поля |
| `slam` | `true` | Включить SLAM (slam_toolbox) |
| `use_sim_time` | `true` | Использовать время симуляции Gazebo |

## Мониторинг системы (в отдельных терминалах)

```bash
# Проверить список активных нодов
ros2 node list

# Проверить топики
ros2 topic list

# Следить за состоянием FSM
ros2 topic echo /robot_state_str

# Следить за памятью VM
htop
```

## Запуск только RViz2 (если система уже запущена)

```bash
ros2 launch agv_visualization rviz.launch.py
```

## Структура пакета

```
agv_bringup/
├── launch/
│   └── full_system.launch.py   # Единый launch-файл всей системы
├── CMakeLists.txt
└── package.xml
```
