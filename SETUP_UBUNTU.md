# Инструкция по восстановлению окружения проекта

> Этот файл — «страховка» на случай, если ноутбук или виртуальная машина выйдут из строя.
> Здесь по шагам описано: **какую Ubuntu ставить**, **как настроить VirtualBox** и
> **как подключить папку проекта внутрь Ubuntu**, чтобы продолжить работу.

---

## 1. Какую Ubuntu устанавливать

- **Дистрибутив:** Ubuntu **22.04.x LTS Desktop** (конкретно использовалась **22.04.5**).
  - ⚠️ Важно именно **22.04**, а НЕ 24.04 и не 20.04 — потому что проект собран под
    **ROS 2 Humble Hawksbill**, а Humble официально работает только на Ubuntu 22.04.
- **Тип установки:** Desktop (minimal вполне достаточно).
- **Архитектура:** 64-bit (amd64).
- Скачать образ: https://releases.ubuntu.com/22.04/ (файл вида `ubuntu-22.04.x-desktop-amd64.iso`)

---

## 2. Настройки виртуальной машины в VirtualBox

Создаём новую VM и выставляем параметры (они подобраны под слабый ноутбук Samsung 550P,
i7-3610QM, 8 ГБ RAM, встроенная Intel HD 4000):

| Параметр                | Значение                            | Почему так                                          |
|-------------------------|-------------------------------------|-----------------------------------------------------|
| Тип / Версия            | Linux / Ubuntu (64-bit)             |                                                     |
| Оперативная память      | **5120 МБ (5 ГБ)**                   | Больше нельзя — у хоста всего 8 ГБ                   |
| Процессоры (CPU)        | **4 ядра**                          |                                                     |
| Видеопамять             | **64 МБ**                           | 128 МБ вызывало ошибку памяти VirtualBox            |
| 3D ускорение            | **ОТКЛЮЧЕНО**                       | С включённым VM падает с ошибкой памяти             |
| Жёсткий диск            | 50 ГБ, динамический (VDI)            |                                                     |

**Пользователь и имя машины (как было настроено):**
- Имя пользователя: `dan`
- Hostname: `dan-VirtualBox`

> ⚠️ Из-за слабой графики **Gazebo запускается только в headless-режиме** (`gui:=false`,
> работает только `gzserver`, без `gzclient`). Визуализация — через **RViz2**.

---

## 3. Установка Guest Additions

Гостевые дополнения нужны для общих папок и нормального разрешения экрана.
Использовалась версия **7.1.8**.

```bash
# В Ubuntu, в терминале:
sudo apt update
sudo apt install -y build-essential dkms linux-headers-$(uname -r)
```

Затем в меню VirtualBox: **Устройства → Подключить образ диска Дополнений гостевой ОС**
и запустить `autorun.sh` с подключённого диска. После — **перезагрузить** Ubuntu.

Добавить пользователя в группу, которая имеет доступ к общим папкам:
```bash
sudo usermod -aG vboxsf dan
# после этого перелогиниться или перезагрузиться
```

---

## 4. Подключение папки проекта (общая папка Windows ↔ Ubuntu)

Это самое главное для продолжения работы. Проект физически лежит на Windows,
а Ubuntu видит его как папку `~/shared`.

### 4.1. Настроить общую папку в VirtualBox

При выключенной VM: **Настройки VM → Общие папки → добавить новую**:

| Поле                   | Значение                                                   |
|------------------------|------------------------------------------------------------|
| Путь к папке           | папка проекта на Windows (раньше было `D:\ROS2_Project`, сейчас может быть `F:\ROS2_Project`) |
| Имя папки (Folder Name)| `ros2_project`  ← **важно: ровно это имя**                 |
| Авто-подключение       | можно НЕ ставить (мы подключим вручную через fstab)        |
| Только для чтения      | НЕ ставить                                                  |

### 4.2. Автомонтирование через /etc/fstab

```bash
# Создать точку монтирования
mkdir -p ~/shared

# Узнать свой UID и GID (обычно 1000:1000)
id

# Открыть fstab для редактирования
sudo nano /etc/fstab
```

Добавить в конец файла **одну строку** (uid/gid поставить свои из `id`):
```
ros2_project   /home/dan/shared   vboxsf   uid=1000,gid=1000,_netdev   0   0
```
- `ros2_project` — это **Имя папки** из VirtualBox (п. 4.1), а НЕ путь.
- `/home/dan/shared` — куда монтируем.

Сохранить (`Ctrl+O`, `Enter`), выйти (`Ctrl+X`). Затем смонтировать:
```bash
sudo mount -a
ls ~/shared      # должны увидеть CLAUDE.md, SETUP_UBUNTU.md, папку src/
```
Теперь после каждой загрузки Ubuntu папка `~/shared` будет подключаться автоматически.

> Если `mount -a` ругается — проверь, что Guest Additions установлены (п. 3) и
> пользователь в группе `vboxsf`, затем перезагрузи VM.

---

## 5. Установка ROS 2 Humble и зависимостей

Полная инструкция: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Коротко:
```bash
# Локали
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Репозиторий ROS 2
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Сам ROS 2 Humble (полный desktop)
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
```

Дополнительные пакеты проекта:
```bash
sudo apt install -y \
  gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  python3-colcon-common-extensions
```

> Версии, под которые делался проект: **ROS 2 Humble**, **Gazebo Classic 11.10.2**
> (именно Classic, не Harmonic/Fortress).

---

## 6. Создание рабочего пространства и сборка

```bash
# Создать workspace
mkdir -p ~/ros2_ws/src

# Подключить папки проекта (src лежит в ~/shared) в workspace через симлинк
ln -s ~/shared/src/* ~/ros2_ws/src/

# Собрать
cd ~/ros2_ws
colcon build --symlink-install
```

Добавить в `~/.bashrc` (чтобы окружение подхватывалось автоматически):
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 7. Получить код заново с GitHub (если папки на Windows больше нет)

Если ноутбук умер совсем и общей папки нет — код берётся из GitHub-репозитория
(см. файл, куда залит проект). На любой машине:
```bash
git clone <URL_РЕПОЗИТОРИЯ>
```
Затем положить содержимое в общую папку / или прямо в `~/ros2_ws/src/` и собрать (п. 6).

---

## Шпаргалка по полезным командам

```bash
cd ~/shared                              # перейти в папку проекта
cd ~/ros2_ws && colcon build --symlink-install   # собрать пакеты
source ~/ros2_ws/install/setup.bash      # применить изменения
ros2 launch <package> <launchfile>       # запустить систему
free -h                                  # сколько свободно RAM
htop                                     # загрузка CPU/памяти
```
