# Развёртывание проекта в облаке Timeweb Cloud

> Цель: поднять в облаке мощную Ubuntu-машину, на которой будут крутиться
> ROS 2 + Gazebo + Nav2 (на нормальной скорости), удалённый рабочий стол
> (чтобы видеть RViz) и Claude Code (чтобы ассистент работал прямо на сервере).
>
> Платим российской картой. Чтобы не возиться с VPN — берём **европейскую
> локацию** (IP будет «рабочим» для нужных сервисов).

---

## Что в итоге получится

```
        Твой ноутбук (Windows)                Облачный сервер (Ubuntu 22.04, Timeweb)
   ┌───────────────────────────┐         ┌──────────────────────────────────────────┐
   │  Подключение к удалённому  │  RDP →  │  XFCE рабочий стол  →  RViz (видишь сам)   │
   │  рабочему столу (mstsc)    │         │  Gazebo (headless) + Nav2 + EKF + наши узлы│
   │                            │  SSH →  │  Claude Code (ассистент чинит прямо тут)   │
   │  git pull (забрать код)    │ ← git → │  проект с GitHub, colcon build             │
   └───────────────────────────┘         └──────────────────────────────────────────┘
```

---

## Часть 1. Создание сервера в Timeweb Cloud

1. Зарегистрируйся на **https://timeweb.cloud**, пополни баланс (карта РФ).
2. В панели: **Облачные серверы → Создать сервер**.
3. Параметры:
   - **ОС:** Ubuntu **22.04** (обязательно 22.04 — под ROS 2 Humble).
   - **Локация:** ⭐ **Нидерланды** или **Польша** (европейский IP → без VPN).
   - **Конфигурация:** **8 vCPU / 16 ГБ RAM / 40–50 ГБ NVMe**.
     (Минимум — 4 vCPU / 8 ГБ, но 8/16 даст комфорт для Gazebo+Nav2+RViz.)
     ⚠️ GPU **не нужен** — Gazebo у нас headless, RViz рисуется программно на CPU.
   - **Тарификация:** почасовая (платишь только когда сервер включён).
   - **Доступ:** задай **root-пароль** (запиши!) или загрузи SSH-ключ.
     Для новичка проще пароль.
4. Создай сервер. Через 1–2 минуты получишь его **IP-адрес** (например, `5.61.xx.xx`).

> 💰 **Экономия:** когда не работаешь — **выключай** сервер в панели (или делай
> «снапшот» и удаляй). Включённый сервер тарифицируется круглосуточно.

---

## Часть 2. Первое подключение по SSH

На Windows открой **PowerShell** (SSH уже встроен) и подключись:
```powershell
ssh root@ТВОЙ_IP
```
- На вопрос про «fingerprint» ответь `yes`.
- Введи root-пароль (при вводе символы не отображаются — это нормально).

Ты внутри сервера. Дальше все команды выполняются здесь.

---

## Часть 3. Базовая настройка

```bash
# Обновить систему
apt update && apt upgrade -y

# Создать обычного пользователя dan (как на твоей локальной VM) с правами sudo
adduser dan            # задаст пароль — ЗАПИШИ его, он нужен для рабочего стола
usermod -aG sudo dan

# Установить файрвол и базовые утилиты
apt install -y ufw curl wget git nano htop
```

Настроим файрвол. Узнай свой домашний IP (открой на ноуте https://2ip.ru) и подставь его:
```bash
ufw allow OpenSSH
# RDP (рабочий стол) — открываем ТОЛЬКО для твоего домашнего IP (безопасность!)
ufw allow from ТВОЙ_ДОМАШНИЙ_IP to any port 3389 proto tcp
ufw --force enable
ufw status
```
> Если у тебя дома «прыгающий» (динамический) IP — при смене придётся повторить
> команду `ufw allow from ...` с новым IP. Узнать новый — снова на 2ip.ru.

---

## Часть 4. Удалённый рабочий стол (XFCE + xrdp)

Это даст тебе графический рабочий стол Ubuntu, который ты откроешь из Windows.

```bash
# Лёгкий рабочий стол XFCE + сервер xrdp
sudo apt install -y xfce4 xfce4-goodies xorg dbus-x11 x11-xserver-utils
sudo apt install -y xrdp

# Сказать xrdp запускать именно XFCE
echo "xfce4-session" | sudo tee /home/dan/.xsession
sudo chown dan:dan /home/dan/.xsession

# Разрешить xrdp доступ к сертификатам и запустить службу
sudo adduser xrdp ssl-cert
sudo systemctl enable --now xrdp
sudo systemctl restart xrdp
```

### Подключение из Windows
1. Нажми `Win+R`, набери `mstsc`, Enter — откроется «Подключение к удалённому рабочему столу».
2. В поле «Компьютер» введи `ТВОЙ_IP` → «Подключить».
3. В окне входа xrdp: **Session = Xorg**, логин `dan`, пароль пользователя dan (из Части 3).
4. Появится рабочий стол Ubuntu. 🎉

> 🛠 Если при входе выскакивает запрос «Authentication required to create managed
> color device» — выполни (по SSH) фикс polkit:
> ```bash
> sudo bash -c 'cat >/etc/polkit-1/localauthority/50-local.d/45-allow-colord.pkla <<EOF
> [Allow Colord all Users]
> Identity=unix-user:*
> Action=org.freedesktop.color-manager.create-device;org.freedesktop.color-manager.create-profile;org.freedesktop.color-manager.delete-device;org.freedesktop.color-manager.delete-profile;org.freedesktop.color-manager.modify-device;org.freedesktop.color-manager.modify-profile
> ResultAny=no
> ResultInactive=no
> ResultActive=yes
> EOF'
> sudo systemctl restart xrdp
> ```

---

## Часть 5. Установка ROS 2 Humble и зависимостей проекта

Подключись по SSH **как dan** (или в терминале на рабочем столе):
```powershell
ssh dan@ТВОЙ_IP
```

```bash
# Локали
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Репозиторий ROS 2
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS 2 Humble + инструменты
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools

# Зависимости проекта
sudo apt install -y \
  gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  python3-colcon-common-extensions
```

---

## Часть 6. Получить проект с GitHub

Репозиторий приватный, поэтому при клонировании нужен **токен доступа (PAT)**.

1. На ноуте зайди на GitHub → справа вверху аватар → **Settings** →
   внизу слева **Developer settings** → **Personal access tokens** →
   **Tokens (classic)** → **Generate new token (classic)**.
2. Поставь галочку **`repo`**, срок — на твой вкус, сгенерируй, **скопируй токен**
   (он показывается один раз!).
3. На сервере:
```bash
cd ~
git clone https://github.com/svp2ch/ros2-agv-tractor.git
# Username: svp2ch
# Password: ВСТАВЬ_ТОКЕН (не пароль от GitHub!)
```

---

## Часть 7. Сборка рабочего пространства

```bash
mkdir -p ~/ros2_ws/src
ln -s ~/ros2-agv-tractor/src/* ~/ros2_ws/src/

cd ~/ros2_ws
colcon build --symlink-install

# Автозагрузка окружения
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Часть 8. Установка Claude Code

```bash
# Способ 1 (рекомендуется): официальный установщик (без sudo)
curl -fsSL https://claude.ai/install.sh | bash

# Способ 2 (если первый не сработал): через Node.js
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt install -y nodejs
sudo npm install -g @anthropic-ai/claude-code
```

Запуск и вход:
```bash
cd ~/ros2-agv-tractor
claude
```
- При первом запуске попросит войти в аккаунт. Поскольку у нас есть **рабочий стол**,
  проще всего: запусти `claude` **в терминале на рабочем столе XFCE** — откроется
  браузер Firefox прямо на сервере, войдёшь в аккаунт, подтвердишь — и всё.
- Теперь ассистент работает прямо на сервере: видит код, сам собирает и запускает ROS.

> Если вход в аккаунт/доступ к API не проходит (значит локация всё же блокируется) —
> см. Часть 10 «VPN (запасной вариант)».

---

## Часть 9. Запуск симуляции и просмотр в RViz

1. На **рабочем столе** (через RDP) открой терминал.
2. Запусти систему **headless** (без 3D-окна Gazebo):
   ```bash
   ros2 launch agv_bringup full_system.launch.py rviz:=false
   ```
3. В **другом** терминале на рабочем столе запусти RViz:
   ```bash
   source ~/ros2_ws/install/setup.bash
   rviz2 -d ~/ros2_ws/install/agv_visualization/share/agv_visualization/config/agv_rviz.rviz
   ```
   RViz нарисуется силами CPU сервера, а ты увидишь картинку у себя.

> 🛠 Если RViz падает с ошибкой OpenGL — заставь его рисовать программно:
> ```bash
> export LIBGL_ALWAYS_SOFTWARE=1
> rviz2 ...
> ```

### Запись демо для защиты
Чтобы не зависеть от живого запуска на защите — запиши прогон:
```bash
ros2 bag record -a -o ~/demo_run     # запись всех топиков
# ... дай трактору проехать ...  Ctrl+C для остановки
```
Файл `~/demo_run` можно скачать на ноут (`scp`) и проигрывать локально в RViz —
воспроизведение лёгкое, ноут потянет.

---

## Часть 10. VPN на сервере (ЗАПАСНОЙ вариант — только если API не подключается)

Нужно, только если взял РОССИЙСКУЮ локацию или европейская всё равно блокируется.
Поднимаем исходящий VPN на самом сервере (например, через WireGuard-клиент к своему
VPN-провайдеру, или собственный сервер). Это отдельная большая тема — **напиши мне,
если до этого дойдёт**, распишу под твой конкретный VPN. Сначала попробуй без него
(европейская локация обычно работает напрямую).

---

## Часть 11. Контроль расходов

- **Не работаешь — выключай сервер** в панели Timeweb (тарификация идёт, пока включён).
- Чтобы не терять настройки при удалении — сделай **снапшот** (образ) сервера.
- Прикинь бюджет заранее: при почасовой оплате считаешь только часы реальной работы.

---

## Шпаргалка: типичный сеанс работы

```
1. Включить сервер в панели Timeweb.
2. ssh dan@IP            (или сразу RDP через mstsc)
3. cd ~/ros2-agv-tractor && git pull   # подтянуть последние правки
4. claude                # позвать ассистента — чиним/разрабатываем
5. (в RDP) запустить симуляцию + RViz, посмотреть результат
6. git push              # сохранить изменения на GitHub
7. Выключить сервер в панели.
```
