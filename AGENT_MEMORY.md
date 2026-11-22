# Память агента: Blender → Unreal Engine

> Прочитать в начале новой сессии и обновить в конце. Это рабочая память проекта, не инструкция пользователю моделировать самостоятельно. Пользователь — арт-директор; агент выполняет моделирование и подготовку ассетов.

## Текущее состояние

- **Последнее обновление:** 2026-11-21.
- **Референс качества и Blender-метод:** пользователь прислал Sketchfab Volvo S90 Recharge и приложил из другой сессии `car_build.py`, `README_auto.md`, `zoom.py`, `turntable.py`, `silhouette.py`. Все копии сохранены в `references/`; Volvo — методический образец, целевой автомобиль по-прежнему GR86. В `docs/reusable-workflows.md` выписаны лофт-сечения, интегрированные арки/стёкла, raycast-детали, силуэтные тесты, коллекции/риг, zoom-ракурсы, turntable и студийный Cycles. Вспомогательные скрипты требуют адаптации путей и отсутствующих `ref/sil_ref.npy`/объектов.
- **Связанная сессия:** https://arena.ai/agent/01a1fcb6-69e8-7deb-8520-3c4096b31e46 — использовать её наработки вместе с артефактами этой рабочей копии; источник истины по текущему состоянию — файлы репозитория и проверяемые результаты.
- **Проект:** реалистичные игровые автомобили, Blender → UE.
- **Текущий ассет:** Toyota GR86 второго поколения, стандартная версия 2022+.
- **Цель ассета:** управляемая машина для UE5 на ПК; реалистичный экстерьер, учитывать внешнюю и внутреннюю камеры, приоритет внешнему виду; умеренный игровой бюджет; белый кузов; спортивные колёса выбрать агенту.
- **Пользователь предпочитает:** отвечать на вопросы и принимать творческие решения, не выполнять моделирование/уроки самостоятельно.
- **Blender runtime:** native `bpy` wheel импортировать не удалось из-за недостающих системных X/GL библиотек. Обход найден: npm package `@blackbelt-technology/pi-dashboard-blender` с Blender WebAssembly 5.3.0 Alpha; Node 24+ JSPI. Runtime кэшируется вне репозитория, команда и MCP runner сохранены в `scripts/run_blender_wasm.py`. WASM может построить/сохранить `.blend`, но не рендерит и не поддерживает Subsurf.
- **Фактическая Blender-сцена:** `assets/gr86_studio.blend` создана настоящим Blender Python (`bpy`), повторно импортирована и проверена. Содержит 79 объектов, плотную оболочку `Body_Shell` (4.264 × 1.761 × 1.003 м), встроенные faces остекления/окантовки, 4 wheel pivots, камеры/свет и коллекции. `tests/validate_gr86_blend.py` прошёл внутри Blender 5.3 Alpha.
- `assets/gr86_blender_mesh.obj/.mtl` экспортированы из `.blend`; не считать программные превью Cycles. В сцене отдельно моделируются `Body_Shell`, `Front_Bumper`, `Rear_Bumper`, `Hood_Panel`, `Trunk_Lid`, `Roof_Panel`, пары `Front_Fender_L/R`, `Door_L/R`, `Rear_Quarter_L/R`, независимое curved glazing; bonnet/deck panels и двери имеют hinge roots, крыша примыкает через отдельные A/B/C стойки. Недавний defect open render: Hood/Trunk part classification включал только ring indices 10..15, забирая лишь внешнюю часть поперечного профиля и оставляя основную корону в body — открытые детали выглядели как полосы. Исправлено на complete upper profile 10..18; новые Blender assertions требуют Hood width >1.2m / depth >1m, Trunk width >1.2m / depth >.75m. В actual Cycles open-panels render теперь видны цельные bonnet и trunk lids; engine-bay close-up показывает аккумулятор, radiator, block/heads/intake, strut brace. Добавлены arch-lip sheet returns/inner liners, dimensional headlamp projector/DRL and 3D taillights, sculpted hood/door beads; inner door trim reparented with doors. Scene after separating full-width front/rear fascia: 214 objects / 193 mesh; evaluated OBJ 47,652 vertices / 45,818 faces. Renderer shots: `front_3_4`, `side`, `rear_3_4`, `cockpit`, `panels_open`, `engine_bay`; все exterior outputs обновлены после full-span panel split, cockpit remains current for unchanged interior. User chose OEM stance (year/trim still open). Replaced horizontal grille slats with modeled honeycomb mesh geometry; no aftermarket aero added. Body-side door skins received subtle convex crowns and the window openings were reshaped into a swept front A-pillar and tapered fastback quarter glass. Front fascia now uses an actual geometric honeycomb mesh instead of horizontal slats. Front Cycles render updated and checked. Current evaluated OBJ 47,652 vertices / 45,818 faces after honeycomb. Front/rear bumper skins are now complete cross-section mesh islands; grille/headlamps/splitter and rear lamps/plate/diffuser/exhaust are parented to respective fascia. Added thin black bumper joint seams attached to each bumper; OBJ-side door gap locations now match actual side-panel split x=−.75/.75. Validation asserts bumper width/span and seam parenting. Blender WASM validation and 5 OBJ tests pass. Силуэт ещё procedural draft; FBX/UE проверка не сделана.

## Задачи

### Планируемые

1. Сверить силуэт GR86 с ортографическими референсами; улучшить текущую процедурную кузовную оболочку, арки и встроенное остекление.
2. Использовать архитектуру из `references/volvo_s90_car_build.py` (плотный лофт, face-material glazing, raycast детали), но заново задать кривые GR86.
3. Доработать интерьер/детали и физически корректные wheel steering pivots, двери и риг.
4. Подготовить игровой mesh, UV/материалы, LOD и collision meshes под умеренный бюджет.
5. Получить финальные PNG в desktop Blender/Cycles, затем экспортировать и проверить в UE5.

### Начатые

- **Модельный черновик:** OBJ и `.blend` созданы; OBJ проверен структурно, Blender scene проверена в Blender WASM по smoke-тестам. Визуально в Blender не проверялась: WASM не умеет рендерить.
- **Реалистичность:** текущая сетка всё ещё черновая; следующий этап — перенести в GR86 плотный параметрический лофт и встроенное остекление из приложенного Volvo builder, затем добавить референсную проверку.
- **Доступ к Blender:** работает через npm Blender WebAssembly, см. `scripts/run_blender_wasm.py`; desktop Blender/Puter остаются недоступны напрямую.
- **Документация:** `README.md`, `docs/reusable-workflows.md` и этот файл памяти отражают текущие артефакты и ограничения.

### Законченные

- Зафиксирован бриф: GR86 2022+, UE5/ПК, управляемый автомобиль, реалистичный экстерьер, обе камеры, умеренная оптимизация, белый кузов, спортивные колёса на выбор агента.
- Зафиксирована роль пользователя как арт-директора, а не исполнителя моделирования.
- Создана база переиспользуемых процедур с правилом отделять проверенные результаты от предположений.
- Сохранены пользовательские вложения как `references/volvo_s90_car_build.py` и `references/README_auto.md`; скрипт Volvo проходит `py_compile` и использован как архитектурный/методический образец.
- `scripts/import_gr86_in_blender.py` запущен через реальный Blender WASM runtime; создан `assets/gr86_studio.blend` (93 объекта, 187,202 bytes, Blender 5.3.0 Alpha) с материалами, wheel assemblies/pivots, камерами и студийными Area lights.
- `.blend` независимо импортирован обратно в Blender WASM; `tests/validate_gr86_blend.py` прошёл: 4 колеса и их pivot-world позиции совпадают с wheel roots, размер кузова 4.264 × 1.762 × 0.555 м, найдены материалы/свет/камеры.
- Добавлены `scripts/run_blender_wasm.py` и инструкции запуска; Blender WebAssembly рендерить не может, PNG previews остаются программными, не Blender render.
- Созданы генераторы `scripts/generate_gr86_blockout.py` (bpy) и рабочий dependency-free `scripts/generate_gr86_obj.py`.
- Сгенерированы `assets/gr86_blockout.obj` и `.mtl`: текущая версия — 7,446 вершин, 6,650 граней, 81 объектная группа; границы X=4.365 м, Y=1.98 м, Z=1.3025 м. Создан пакет `assets/gr86_blockout_package.zip`.
- Добавлен `scripts/render_obj_png.py`: программный software-renderer на NumPy/Pillow создал три PNG-ракурса модели и архив `assets/renders/gr86_renders.zip`. Это визуализация фактической OBJ-сетки, но не Blender/Cycles render; модель всё ещё требует геометрической доработки.
- Обновлён модельный черновик: добавлены сглаженная продольная интерполяция кузова и перепад профиля капот/корма, вырезы колёсных арок в нижнем контуре, отдельные плоскости остекления и стойки, уплотнённые диски/спицы, зеркала-эллипсоиды, головная оптика с ободком, дверные швы, юбки, вентиляционные детали, сплиттер, диффузор и выхлоп.
- Добавлен `requirements-render.txt` для воспроизводимого запуска программных PNG-превью.
- Добавлены `.gitignore` (исключает `.venv`) и пять автотестов в `tests/test_gr86_obj.py`; все пять пройдены после обновления модели.
- Native `bpy` wheel остаётся заблокирован системными библиотеками, но эквивалентную сборку `.blend` выполнено через Blender WebAssembly 5.3 Alpha; scene smoke-test пройден.

## Набор автотестов и проверок

### Статус

OBJ-ассет и `.blend` созданы. Пять OBJ structural tests и `tests/validate_gr86_blend.py` в реальном Blender WASM 5.3 Alpha проходят. `.blend` проверен повторным импортом; его силуэт визуально в Blender не оценивался (WASM не рендерит). FBX и импорт в UE пока не выполнены. Не отмечать Blender визуальный рендер и UE проверки пройденными без фактического запуска.

### Автоматизированные Blender-проверки и оставшийся план

1. **Структура сцены:** обязательные объекты кузова и 4 колеса существуют; имена уникальны; применены ожидаемые коллекции/группы.
2. **Геометрия:** mesh-объекты не пустые; нет NaN/Inf координат; bounds и габариты положительны и близки к реальным габаритам машины.
3. **Трансформации и pivots:** wheel origins находятся в центрах колёс; левая/правая стороны симметричны относительно центральной плоскости; ось вращения колёс согласована.
4. **Нормали/топология:** нет вырожденных граней и нулевых нормалей; проверить non-manifold там, где ожидается замкнутый объём; исключения для открытых панелей должны быть явными.
5. **Материалы:** у видимых объектов назначены материалы; обязательные категории кузов/стекло/резина/металл существуют.
6. **LOD и коллизии:** если они заявлены готовыми, соответствующие объекты присутствуют и имеют ожидаемые префиксы; collision meshes проще визуальных.
7. **Экспорт:** экспортный набор содержит ожидаемые FBX и текстуры; файлы ненулевого размера; имена и структура каталогов соответствуют соглашению.
8. **Регрессия:** повторный запуск генерации/экспорта не создаёт дубли объектов и даёт одинаковую структуру и ожидаемые границы модели.

### Проверить в UE вручную или автоматизацией UE

- FBX импортируется без ошибок; масштаб, ориентация, smoothing/normals корректны.
- Колёса/подвижные части раздельны и имеют ожидаемые pivots.
- Материалы и текстуры назначаются правильно; нет отсутствующих файлов и чёрных/розовых материалов.
- Collision и LOD создаются/назначаются и работают; модель проверяется с целевой камеры.

## Удачные мысли и решения

- Сначала блок-аут и проверка силуэта в нескольких ракурсах; мелкие детали только после фиксации пропорций.
- Реализм кузова зависит от качества поверхности и отражений; сложный шейдер не исправляет плохую геометрию.
- У управляемого автомобиля колёса и анимируемые панели должны быть самостоятельными объектами с осмысленными pivots.
- Не задавать точный poly/texture budget без конкретной платформы, дистанции камеры и измерений.
- Экспортный пресет считать непроверенным, пока тестовый экспорт реально не проверен в целевой версии UE.
- Пользователь делегировал агенту выбор спортивного дизайна колёс; не задавать этот вопрос снова без причины.
- Для последующего переиспользования записывать в `docs/reusable-workflows.md` только проверенные процедуры с версиями инструментов, результатом и ограничениями.

## Протокол продолжения

1. Прочитать этот файл и `docs/reusable-workflows.md`.
2. Сверить фактические файлы/статус репозитория, прежде чем считать задачу выполненной.
3. Продолжить первую незавершённую задачу из списка «Планируемые»; задавать пользователю только вопросы, которые реально влияют на результат.
4. После каждого существенного этапа перенести задачу между разделами, указать артефакты и реальные результаты тестов.
5. В конце сессии обновить дату и состояния, не записывая предположения как факты.
