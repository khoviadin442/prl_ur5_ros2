# Хендофф: доводка телеопа на рабочем компе (jerry)

Записка для ассистента, который продолжает настройку на рабочей машине.
Домашняя машина — эталон, там всё работает; рабочая — `khoviadi@jerry`, настраивается сейчас.

## Что это за проект

VR-телеоп руки mantis (dual UR5, левая рука + гриппер Weiss WSG50) с Meta Quest 2:
паблишер на хосте отдаёт позу контроллера в `/vive/pose` + `/vive/buttons`, мост в
контейнере считает Pink diff-IK с барьером коллизий и публикует команды суставов.
Опционально пишутся эпизоды в датасет LeRobot v3 и воспроизводятся стоковым
`lerobot-replay`.

Полное описание — `mantis_teleop/README.md` в этом же репозитории.

## Раскладка на рабочем компе

| Что | Где |
|---|---|
| репозиторий | `~/rabota/prl_ur5_ros2`, ветка `mantis-teleop` |
| исходники телеопа (чистые, из гита) | `~/rabota/prl_ur5_ros2/mantis_teleop/` |
| шара контейнера | `~/rabota/docker_shared` (в контейнере `/home/ros/share`) |
| ROS workspace | `~/rabota/docker_shared/mantis_ws` |
| хостовый prefix для `package://` | `~/rabota/mantis_host_deps/fakeprefix` |
| хостовое ROS-окружение | `~/rabota/SO-100-HTC-vive-teleop/.pixi/envs/default/setup.bash` |
| контейнер | имя `mantis`, образ `prl_ros2:<user>`, запускается `docker-ros2/start_docker.bash` |

## Что уже сделано

1. Репозиторий склонирован, ветка `mantis-teleop`.
2. `./mantis_teleop/setup_workstation.sh` отработал: разложил телеоп в шару, склонировал
   `mantis_ws` (prl_ur5_ros2 + 6 зависимостей), применил патчи, собрал fakeprefix,
   вытащил `ur_description` из контейнера.
3. Старая версия телеопа (месячной давности) с рабочего компа удалена.
4. Хостовое окружение: `pixi` и `adb` поставлены в домашнюю папку (sudo на машине нет),
   в SO-100 добавлены `pinocchio`/`pink`/`qpsolvers`/`scipy`/`coal` и pypi-пакет `openvr`.
   `python -c "import rclpy, openvr, pinocchio, pink"` проходит.
5. Контейнер `mantis` запускается.

## Что осталось

1. **Сборка workspace в контейнере** (на хосте нечем):
   ```bash
   source /opt/ros/jazzy/setup.bash
   cd ~/share/mantis_ws && colcon build --symlink-install && source install/setup.bash
   ```
   Результат остаётся на хосте (шара смонтирована), пересобирать при каждом запуске не нужно.
   Пакеты `allegro_*`, `robotiq*`, `onrobot*` к телеопу отношения не имеют — при поломке
   можно `--packages-skip-regex 'allegro.*|robotiq.*|onrobot.*'`.
2. **Плагин для replay** (нужен только для `run_replay.sh`, живёт внутри контейнера и
   исчезает с ним, т.к. запуск идёт с `--rm`):
   ```bash
   python3 -m pip install --user --break-system-packages -e ~/share/lerobot_robot_mantis
   ```
3. **URDF** — на хосте, при запущенном контейнере:
   ```bash
   cd ~/rabota/prl_ur5_ros2 && ./mantis_teleop/setup_workstation.sh   # ждём "generated (N lines)"
   ```
4. **Пробный старт** в контейнере: `python3 ~/share/teleop_mantis.py` — должен построить
   модель коллизий и встать в ожидание `/joint_states`, без трейсбеков.
5. **Quest**: Developer Mode, `adb devices` = `device`; ALVR (AppImage + APK) либо
   `QUEST_BACKEND=adb`. Проверка раскладки: `~/rabota/docker_shared/run_quest_pub.sh --scan`.
6. **Симуляция** (`ros2 launch prl_ur5_run sim.launch.py`), и только потом железо
   (`real.launch.py`, E-stop под рукой, временно `teleop.scale: 0.5`).

## Известные грабли этой машины

* **sudo нет.** Всё ставится в домашнюю папку: pixi — инсталлятором с pixi.sh, adb —
  распакованным `platform-tools`. Единственное, что требует админа, — udev-правило, если
  `adb devices` покажет `no permissions`. Docker уже установлен и доступен.
* **`pip` / `pip3` не найдены в контейнере.** Проверить `python3 -m pip --version` и
  `python3 -c "import pinocchio, pink, qpsolvers, lerobot"`. Если импорты падают — контейнер
  поднят на старом образе без телеоп-слоёв: `docker rm -f mantis`, затем
  `./start_docker.bash mantis ~/rabota/docker_shared --rebuild` (долго: torch + lerobot).
  В дереве должен лежать наш Dockerfile: `grep -c "pin-pink\|lerobot" docker-ros2/Dockerfile` != 0.
* **`container name /mantis is already in use`** — остался огрызок: `docker rm -f mantis`.
* **`bash: /home/ros/share/mantis_ws/install/setup.bash`** при входе в контейнер — нормально
  до первого `colcon build`.
* **SO-100 pixi окружение.** В форке на GitHub `pixi.toml` был без `openvr` и без
  pinocchio/pink — они добавлялись только локально на домашней машине. На рабочей это
  вылечено через `pixi add` / `pixi add --pypi openvr`. Если окружение пересоздают с нуля,
  проверить, что `pixi run python -c "import openvr"` проходит.
* После `source setup.bash` проверять `which python` — он должен указывать внутрь
  `.pixi/envs/default/bin`, иначе `run_quest_pub.sh` возьмёт системный python и упадёт на
  `import openvr`.

## Правила работы с кодом

* Этот репозиторий содержит **чистые** версии: без `#`-комментариев в Python, с короткими
  однострочными докстрингами, в YAML — только краткие пояснения. Длинная история тюнинга
  живёт на домашней машине в `~/rabota/docker_shared` и в гит не попадает.
* Значения параметров в `config_teleop_mantis.yaml` подобраны на железе и связаны между
  собой (скорости, `out_accel`, `max_joint_lead`). Не менять «на глаз»: откат делается
  набором, а не одной строкой. Исключение для первого пуска — временно `teleop.scale: 0.5`.
* Локальные под-стенд значения, которые стоит проверить перед железом: IP гриппера в
  `mantis_ws/src/wsg50-ros-pkg/wsg_50_driver/config/wsg50_setup.yaml`, позы рук в
  `prl_ur5_robot_configuration/config/standard_setup.yaml`, топики камер в секции `record:`.
* `setup_workstation.sh` ничего не перезаписывает: для обновления файлов из репо нужны
  `--force-files`, для повторного наложения патчей — `--force-patches`.

## Быстрая диагностика

```bash
# хост
ls ~/rabota/docker_shared/mantis_ws/src                     # 7 пакетов
ls ~/rabota/mantis_host_deps/fakeprefix/share               # 4 записи, включая ur_description
ls -l ~/rabota/docker_shared/mantis_ws/mantis.urdf
source ~/rabota/SO-100-HTC-vive-teleop/.pixi/envs/default/setup.bash
which python && python -c "import rclpy, openvr, pinocchio, pink; print('host env OK')"

# контейнер
python3 -c "import pinocchio, pink, qpsolvers, lerobot; print('deps OK')"
ros2 topic hz /vive/pose        # ~250 Гц через ALVR, ~71 Гц через adb
ros2 topic echo /vive/buttons   # 6 полей
```

Блок проводов на гриппере приходит из патченого `wsg_50_simulation/urdf/wsg_50.urdf.xacro`
(`${prefix}_connector_link`) и должен попасть в сгенерированный URDF:

```bash
grep -c 'left_gripper_connector_link' ~/rabota/docker_shared/mantis_ws/mantis.urdf   # != 0
```

Если его там нет — URDF сгенерирован до наложения патчей: удалить `mantis.urdf` и прогнать
`setup_workstation.sh` заново.
