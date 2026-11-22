# ArenaAgent-Blender-Learn

Реалистичный параметрический седан в Blender (по мотивам Volvo S90: обмеры и
силуэт сняты с реальной машины, детали авторские). Весь проект процедурный —
нет загруженных сеток и текстур, всё генерируется скриптами внутри Blender.

## Состав репозитория

| Файл | Роль |
|------|------|
| `car_build.py` (~900 строк) | главный генератор: кузов, колёса, салон, детали, материалы, свет, камеры, анимация, сохранение сцены |
| `qa.py` | техническая проверка геометрии (дубли вершин, вырожденные грани, неманифолд, n-gon) |
| `render_stills.py` | перерендер 6 кадров: 5 камер + Cam_Exterior_DoorsOpen (кадр 60, двери открыты) |
| `turntable.py` | облёт 360°, 241 кадр → `turntable.mp4` |
| `silhouette.py` | сверка силуэта с эталоном (заглушка, ждёт `ref/`) |
| `bake_maps.py` | генерация файловых карт normal / roughness / AO в `maps/` (для UE) |
| `export_ue.py` | экспорт в FBX для Unreal Engine (масштаб 100, UCX_ коллизии, UV, имена) |
| `ue_preview_export.py` | файловые карты → предварительные рендеры → экспорт FBX за один запуск |
| `render_gallery.py` | HTTP-сервер галереи промежуточных рендеров (порт 8080) |
| `serve_preview.py` | простой HTTP-сервер превью проекта (порт 4173) |
| `export_obj.py` | экспорт сцены в OBJ/MTL для превью без Blender |
| `render_obj_png.py` | software-рендер OBJ → PNG (NumPy/Pillow, не требует Blender) |
| `tests/test_geometry.py` | автотесты валидации экспортированной геометрии |
| `requirements-render.txt` | зависимости для software рендеринга (numpy, Pillow) |
| `render_blender_cycles.py` | рендер сцены в desktop Blender Cycles → `renders/cycles/` |
| `run_blender_wasm.py` | запуск bpy-скрипта через Blender WebAssembly (npm, Node 24+) |
| `README_auto.md` | эта документация |

## Требования

* Blender 4.2 (проверялось на 4.2.9). Вне Blender скрипты не запускаются.
* В среде Blender нет PIL; все текстуры процедурные по UV.
* Окно рендера отсутствует — только фоновый режим `-b`.

## Сборка и запуск

```bash
# собрать сцену и сохранить car_realistic.blend
blender -b -noaudio -P car_build.py

# проверить геометрию (должны быть везде нули)
blender -b -noaudio car_realistic.blend -P qa.py

# генерировать файловые карты текстур (normal / roughness / AO в maps/)
blender -b -noaudio -P bake_maps.py

# перерендер статичных кадров (после bake — для проверки качества материалов)
blender -b -noaudio car_realistic.blend -P render_stills.py

# облёт 360°
blender -b -noaudio car_realistic.blend -P turntable.py
```

Все пути относительны к текущей директории запуска Blender.

## Параметры модели (габариты)

| Параметр | Значение |
|----------|----------|
| Длина × ширина × высота | 5.00 × 1.872 × 1.456 м |
| Колёсная база | 3.06 м (оси X = 1.60 / −1.46) |
| Колея | 1.584 м |
| Шина | 235/45 R18, радиус 0.353 м |
| Радиус арки / глубина впадины | 0.445 м / 0.13 м |
| Купол носа / кормы | 0.15 / 0.17 м |
| Сечений вдоль X | 182 |
| Точек профиля тела / остекления | 18 / 9 |

Ключевые кривые: `W(x)` полуширина, `ZB(x)` низ, `BELT(x)` подоконник,
`ZR(x)` крыша, `WR(x)` ширина крыши, `TUMBLE(x)` наклон стёкол (0.06…0.16).
Арка — не прорезь, а гладкая впадина `arch_recess()`; торцы замыкаются куполом
по суперэллипсу (показатели ~0.34 / 0.72).

## Материалы (все процедурные, по UV)

| Материал | Суть |
|----------|------|
| `CarPaint_MetallicBlue` | (0.055, 0.115, 0.280), Metallic 0.62, Roughness 0.26, Coat 1.0/0.03; хлопья металлика через Bump+Noise (Scale=2400), переменная Coat Roughness через ValToRGB |
| `Glass_Window` | Transparent + Glossy с мультипликативным смешиванием Fac через Math MULTIPLY_ADD + Fresnel (без Transmission) |
| `Lens_Clear` | Transparent + Glossy по Fresnel (IOR 1.52) — фары и фонари |
| `Rubber_Tire` | Протектор по UV: checker 150/13, волна 5.5, маска зон, зерно боковины |
| `Leather_Seat` | Зерно (scale 9.0) + прострочка, цвет (0.042, 0.038, 0.037), Bump от Noise (Strength=0.14) |
| `Carpet` / `Headliner_Fabric` | тот же шейдер кожи, масштаб 42 / 55 |
| Эмиссия фар / ДХО / габаритов | 0.55 / 0.22 / 0.16 (приглушена, чтобы не светились «лампочками») |

Новые материалы (идеи из `volvo_s90_car_build.py`):
`chrome` (Metallic=1.0, Rough=0.035), `alu` (Metallic=1.0, Rough=0.22),
`steel` (Metallic=1.0, Rough=0.42), `disc` (тормозной диск, Rough=0.55),
`caliper` (суппорт, красный), `liner` (подкрылок, Rough=0.78),
`badge` / `badge_r` (эмблемы), `screen` (эмиссия экрана),
`leather_acc` (акцентная кожа, scale=320), `plastic` (интерьер),
`soft` (мягкий пластик панели, scale=420), `plate` (номерной знак),
`mirror` (зеркало, Rough=0.02), `gunmetal` (диск колёса, Rough=0.30),
`gloss_black` (глянцевый чёрный), `carbon` (углепластик, Checker через UV).

Файловые карты (`bake_maps.py`, сохраняются в `maps/`):
`normal_body.png`, `roughness_body.png`, `ao_body.png`, `normal_tire.png`,
`roughness_tire.png`, `normal_leather.png`, `roughness_leather.png`.
Материалы подключают их через `Image Texture` + `NormalMap`, если файлы
существуют; иначе работают чисто процедурные fallback-ноды. Карты нужны
обязательно для экспорта в UE — процедурные ноды Blender не переносятся
как текстуры.

## Свет и рендер

* Студия: 5 Area-ламп с цветами (тёплый ключевой свет, холодный задний/contre-jour) + циклорама
* Интерьер: Int_RoofLight_F (3.4 Вт), Int_RoofLight_M (3.8 Вт), Int_SeatLight (2.6 Вт), Int_DashLight (1.2 Вт), Int_Ceiling (1.8 Вт) — тёплый белый (1.0, 0.96, 0.90)
* Мир: HDRI `hdri/studio_small_09.hdr` или градиентный фон EASING (ValToRGB) если файла нет
* Камеры: совпадают с настройками из volvo_s90_car_build.py
* Рендер: Cycles CPU, 48 spp, AgX Medium High Contrast, 1440×810;
  max_bounces=8, transmission_bounces=6, glossy_bounces=4,
  caustics=False, sample_clamp_indirect=6.0,
  transparent_max_bounces=48, blur_glossy=2.0 (обязательны — иначе муар)

UV: `uv_top()` для кузова (u = X, v = Y), `uv_cyl()` для колёс (u = угол
вокруг оси Y, v = по ширине).

## Свет, камеры, рендер

* Интерьер: `Int_RoofLight_F 1.9 Вт`, `_M 2.1 Вт`, `Int_SeatLight 1.7 Вт`,
  `Int_DashLight 0.9 Вт`, `Int_Ceiling 0.9 Вт` (Area).
* Студия: циклорама + 6 Area-ламп + HDRI `hdri/studio_small_09.hdr` (при
  отсутствии файла на диске используется сплошной фон).
* Камеры: `Cam_Exterior (6.05, −5.15, 1.66) → (0.05, 0, 0.74)` 62 мм,
  `Cam_Rear (−5.9, −4.7, 1.90)`, `Cam_Side (0.10, −10.2, 1.30)` 68 мм,
  `Cam_Interior (−1.14, −0.36, 1.20)` 19 мм,
  `Cam_Interior2 (0.62, 0.60, 1.185)` 20 мм.
* Рендер: Cycles CPU, 48 spp, AgX Medium High Contrast, 1440×810.
  `transparent_max_bounces = 48` и `blur_glossy = 2.0` обязательны — иначе
  муар на стёклах.

## Анимация

240 кадров / 24 fps: колёса 40 оборотов (LINEAR), руль ±17°, крен кузова
±0.8°, двери открываются ±70° (кадры 45–200). Сохраняется вместе со сценой
в `car_realistic.blend`.

## Проверка качества

После каждой сборки обязательно:

```bash
blender -b -noaudio car_realistic.blend -P qa.py
```

Эталон состояния (раздел 5 PROJECT_MEMORY.md):
дублей 0, вырожденных граней 0, неманифолдных рёбер 0, n-gon 0.
Сверка макро-рендерами: нос, корма, арка, двери, крыша — проблемы видны
только вблизи.

## План дальнейших этапов (раздел 7 памяти)

1. ✅ Карты-файлы: normal / roughness / AO на диске (`bake_maps.py`, 7 карт в `maps/`).
2. UV салона по швам (сиденья/панель без развёртки).
3. Подвеска (колеса не связаны с неровностями дороги — нет Follow Path / Rigid Body).
4. Ремни, кнопки на руле, швы на панели — салон упрощён.

Готово на этом этапе: файловые карты текстур + подключение в материалы, двери
(отдельные меши `Door_*_Left/Right` с анимацией ±70°), задняя полка/шторка
между задними сиденьями и задним стеклом.

## Правила работы

* Общение и комментарии в коде — на русском.
* Порядок владельца: сначала геометрия, потом текстуры.
* Существующую основу не переписывать с нуля — только точечные правки.
* Реалистичность через материалы, а не плоские цвета.
* Обязательные элементы: анимация колёс, анимация руля, интерьер.
* После каждой сборки: без Traceback, EXIT=0, qa.py — нули.
* Не менять `transparent_max_bounces` и `blur_glossy` «для скорости`.

## Экспорт в Unreal Engine

`export_ue.py` готовит сцену к импорту в UE 4/5 и сохраняет FBX в `exports/`:

```bash
# 1. собрать сцену
blender -b -noaudio -P car_build.py

# 2. проверить геометрию (должны быть везде нули)
blender -b -noaudio car_realistic.blend -P qa.py

# 3. предварительные рендеры 5 кадров + экспорт FBX под UE (за один запуск)
blender -b -noaudio car_realistic.blend -P ue_preview_export.py

# (альтернативно по отдельности:)
blender -b -noaudio car_realistic.blend -P render_stills.py
blender -b -noaudio car_realistic.blend -P export_ue.py
```

Результат в `exports/`:

| Файл | Содержимое | Использование в UE |
|------|-----------|-------------------|
| `car_ue.fbx` | статичный меш кузова, колёс, салона, деталей | основной ассет машины |
| `car_ue_anim.fbx` | то же + анимация (колёса 40 об., руль ±17°, крен ±0.8°) | анимированный вариант / preview |
| `car_ue_col.fbx` | только простые коллизии `UCX_*` (коробки по bounding-box) | импортировать как отдельный коллизионный меш |

Настройки экспорта (зашиты в скрипт):

* **Масштаб 100** — метры Blender → сантиметры UE (единица измерения UE)
* **Transforms применены** — location/rotation/scale запечены перед экспортом
* **Ось: Forward = -X, Up = Z** — стандартный конвеер Blender → UE
* **Триангуляция** — все полигоны переводятся в треугольники
* **UV** — авторазвёртка для мешей без развёртки (UE требует хотя бы один канал)
* **Имена** — пробелы, скобки и `+-/` заменены на `_` (требование UE к именам ассетов)
* **Коллизии** — `UCX_<ИмяОбъекта>` по соглашению UE; при импорте UE сам сопоставит коллизию с мешом

Импорт в UE:

1. Content Browser → Import → выбрать `car_ue.fbx`
2. В окне импорта FBX: галочки **Import Mesh**, **Import Textures** (выкл. — материалы процедурные), **Combine Meshes** (выкл., если нужны отдельные части)
3. Коллизии: импортировать `car_ue_col.fbx` отдельно, или галочка **Auto Generate Collision** в UE, если простых коробок достаточно
4. Материалы в UE создадутся с именами из Blender (`CarPaint_MetallicBlue`, `Glass_Window` и т.д.) — замените на свои PBR-материалы при необходимости

Порядок итераций: правка в `car_build.py` → пересборка → `qa.py` (нули) → `export_ue.py` → импорт в UE.

## Галерея промежуточных рендеров

После получения PNG-файров из Blender (`renders/Cam_*.png`) можно запустить
локальную HTTP-галерею для просмотра кадров в браузере:

```bash
python3 render_gallery.py
# открыть http://localhost:8080 (или превью-порт Arena)
```

Галерея отображает 6 карточек:

| Кадр | Описание |
|------|----------|
| `Cam_Exterior.png` | Передний ¾-вид (кадр 1, закрытые двери) |
| `Cam_Rear.png` | Задний ¾-вид (фонари, корма) |
| `Cam_Side.png` | Боковой профиль (клиренс, колёса на Z=0) |
| `Cam_Exterior_DoorsOpen.png` | Кадр 60 анимации — все 4 двери открыты на 70° |
| `Cam_Interior.png` | Салон сзади: сиденья, тоннель, панель, руль |
| `Cam_Interior2.png` | Крупный план руля, панели и центрального экрана |
| `preview_all.png` | Сводная сетка (собирается отдельно) |

Клик по кадру открывает увеличенное изображение в модальном окне.
`preview_all.png` — сводная сетка 3×2 со всеми кадрами на одном листе;
собирается любым удобным способом (например, ImageMagick `montage`) после
получения отдельных PNG.

## Воркфлоу без Blender (software рендеринг)

Когда Blender недоступен, можно всё равно получить PNG-превью геометрии и
проверить её валидность без запуска Blender:

```bash
# 1. Установить зависимости для software рендеринга
pip install -r requirements-render.txt

# 2. Экспортировать сцену в OBJ/MTL (требуется Blender — один раз)
blender -b -noaudio car_realistic.blend -P export_obj.py

# 3. Рендерить превью из OBJ без Blender
python3 render_obj_png.py exports/car_ue.obj

# 4. Запустить сервер превью и открыть в браузере
python3 serve_preview.py
# http://localhost:4173 (или превью-порт Arena)
```

`render_obj_png.py` создаёт 4 ракурса (Front 3/4, Side, Rear 3/4, Cockpit)
в `renders/car_ue_*.png`. Это software-превью, не Blender/Cycles рендер —
оно полезно для быстрой проверки пропорций и топологии, но не заменяет
финальные Cycles-кадры.

`serve_preview.py` раздаёт файлы проекта на порту 4173 с отключённым кэшем,
чтобы обновлённые PNG сразу появлялись в браузере.

## Автотесты геометрии

```bash
python3 -m unittest discover -s tests -v
```

`tests/test_geometry.py` проверяет экспортированный `exports/car_ue.obj`:
существование файлов, отсутствие NaN/Inf, корректность индексов граней,
отсутствие вырожденных граней, габариты модели close к реальным (5.00 × 1.872 × 1.456 м),
соответствие используемых материалов определённым в MTL.
Тест пропускается, если OBJ ещё не сгенерирован.

## Полный пайплайн рендеринга

### Вариант А: software-превью без Blender (работает в этой песочнице)

```bash
pip install -r requirements-render.txt
blender -b -noaudio car_realistic.blend -P export_obj.py   # один раз, нужен Blender
python3 render_obj_png.py exports/car_ue.obj               # PNG без Blender
python3 serve_preview.py                                    # http://localhost:4173
```

### Вариант Б: настоящий Cycles рендер в desktop Blender 4.2

```bash
blender -b -noaudio -P car_build.py                         # сцена + .blend
blender -b -noaudio car_realistic.blend -P qa.py            # геометрия (нули)
blender -b -noaudio car_realistic.blend --python render_blender_cycles.py
                                                            # 6 кадров в renders/cycles/
```

Результат: `renders/cycles/car_ue_{front_3_4,side,rear_3_4,interior,interior2,doors_open}.png`

### Вариант В: Blender WebAssembly (Arena runtime)

```bash
python3 run_blender_wasm.py car_build.py                    # scena через WASM
```

WASM-рантайм (Blender 5.3 alpha) может строить и сохранять `.blend`, но
**не рендерит** изображения. Для финальных PNG используйте Вариант Б.

## Порядок итерации (полный)

1. правка в `car_build.py` (геометрия / материалы / свет)
2. пересборка: `blender -b -noaudio -P car_build.py`
3. проверка геометрии: `blender -b -noaudio car_realistic.blend -P qa.py` (нули)
4. файловые карты: `blender -b -noaudio -P bake_maps.py` (опционально)
5. рендер: Вариант Б (Cycles) или Вариант А (software-превью)
6. экспорт в UE: `blender -b -noaudio car_realistic.blend -P ue_preview_export.py`
7. galerie рендеров: `python3 render_gallery.py` (порт 8080) или `serve_preview.py` (порт 4173)
