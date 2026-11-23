# Реестр объектов 3D-сцены Volvo S90 (`car_realistic.blend`)

- **Всего меш-объектов:** `164` · **Камер и источников света:** `21`
- **Суммарная геометрия:** `172,405` вершин · `173,914` граней · `341,612` треугольников
- **Контроль качества (`qa.py`):** `0` дефектов по всем 8 счётчикам (дубли, вырожденные грани, non-manifold, n-gon, loose, пустые, без UV, точки в именах)
- **Упакованные текстуры в `.blend`:** `Cluster_UI_Tex` (768×256), `Engine_Badge_Tex` (512×256), `Engine_Spec_Tex` (384×128), `Sensus_UI_Tex` (512×640)

## Сводка по подсистемам

| Подсистема | Объектов | Вершин | Треугольников |
|---|---:|---:|---:|
| 1. Несущий монокок, крыша, стойки и остекление | 19 | 27,978 | 55,792 |
| 10. Студия и окружение (Studio) | 1 | 8 | 12 |
| 2. Передок: крылья, съёмный бампер, оптика и силовой каркас | 23 | 29,240 | 58,036 |
| 3. Капот и моторный отсек Volvo Drive-E T6 AWD | 18 | 10,282 | 20,136 |
| 4. Корма: задние крылья, съёмный бампер, C-фонари и силовой каркас | 17 | 25,874 | 51,488 |
| 5. Багажный отсек, крышка, петли, газлифты и оснащение | 19 | 5,480 | 10,532 |
| 6. Двери, остекление дверей, ручки, петли и дверные карты | 24 | 19,948 | 39,480 |
| 7. Ходовая часть: шины, 10-спицевые диски, тормоза и подкрылки | 24 | 45,108 | 89,824 |
| 8. Салон (Interior): торпеда, OLED 12.3"/9", кресла Nappa, ремни и органы управления | 19 | 8,487 | 16,312 |

## 1. Несущий монокок, крыша, стойки и остекление (19 шт.)

| # | Объект | Родитель | Вершин | Трис | Материалы | Габариты X / Y / Z (м) |
|---:|---|---|---:|---:|---|---|
| 1 | `Body` | `—` | 9,238 | 18,472 | CarPaint_MetallicBlue | `2.04 × 1.83 × 0.83` |
| 2 | `Glass` | `Body` | 2,424 | 4,832 | Glass_Window | `2.75 × 1.73 × 0.46` |
| 3 | `Mirror_L` | `Door_Front_Left` | 1,628 | 3,232 | CarPaint_MetallicBlue, Piano_Black, Chrome_Steel, Amber_LED | `0.12 × 0.21 × 0.12` |
| 4 | `Mirror_R` | `Door_Front_Right` | 1,628 | 3,232 | CarPaint_MetallicBlue, Piano_Black, Chrome_Steel, Amber_LED | `0.12 × 0.21 × 0.12` |
| 5 | `Pillar_A_L` | `Body` | 778 | 1,552 | CarPaint_MetallicBlue | `0.87 × 0.30 × 1.15` |
| 6 | `Pillar_A_R` | `Body` | 778 | 1,552 | CarPaint_MetallicBlue | `0.87 × 0.30 × 1.15` |
| 7 | `Pillar_B_L` | `Body` | 730 | 1,456 | Piano_Black | `0.09 × 0.30 × 1.16` |
| 8 | `Pillar_B_R` | `Body` | 730 | 1,456 | Piano_Black | `0.09 × 0.30 × 1.16` |
| 9 | `Pillar_C_L` | `Body` | 698 | 1,392 | CarPaint_MetallicBlue | `0.93 × 0.29 × 1.13` |
| 10 | `Pillar_C_R` | `Body` | 698 | 1,392 | CarPaint_MetallicBlue | `0.93 × 0.29 × 1.13` |
| 11 | `Roof` | `Body` | 2,810 | 5,616 | CarPaint_MetallicBlue | `1.16 × 1.29 × 0.08` |
| 12 | `Roof_Antenna` | `Roof` | 708 | 1,408 | CarPaint_MetallicBlue, Piano_Black | `0.18 × 0.05 × 0.07` |
| 13 | `Roof_Sunroof` | `Roof` | 1,386 | 2,768 | Piano_Black, Glass_Window | `0.68 × 1.04 × 0.03` |
| 14 | `Side_Skirt_L` | `Body` | 1,402 | 2,800 | Bumper_Anthracite | `2.25 × 0.08 × 0.07` |
| 15 | `Side_Skirt_R` | `Body` | 1,402 | 2,800 | Bumper_Anthracite | `2.25 × 0.08 × 0.07` |
| 16 | `Window_Trim_L` | `Body` | 196 | 388 | Chrome_Steel | `2.28 × 0.27 × 0.46` |
| 17 | `Window_Trim_R` | `Body` | 196 | 388 | Chrome_Steel | `2.28 × 0.27 × 0.46` |
| 18 | `Wiper_L` | `Body` | 274 | 528 | Piano_Black, Rubber_Tire | `0.09 × 0.60 × 0.04` |
| 19 | `Wiper_R` | `Body` | 274 | 528 | Piano_Black, Rubber_Tire | `0.08 × 0.64 × 0.04` |

## 10. Студия и окружение (Studio) (1 шт.)

| # | Объект | Родитель | Вершин | Трис | Материалы | Габариты X / Y / Z (м) |
|---:|---|---|---:|---:|---|---|
| 1 | `Cyclorama` | `—` | 8 | 12 | Studio_Floor | `36.00 × 36.00 × 0.04` |

## 2. Передок: крылья, съёмный бампер, оптика и силовой каркас (23 шт.)

| # | Объект | Родитель | Вершин | Трис | Материалы | Габариты X / Y / Z (м) |
|---:|---|---|---:|---:|---|---|
| 1 | `Bumper_Intake_Front` | `Front_Bumper` | 1,148 | 2,272 | Carbon_Trim | `0.17 × 1.35 × 0.15` |
| 2 | `Cowl_Panel` | `Body` | 224 | 432 | Carbon_Trim | `0.21 × 1.56 × 0.06` |
| 3 | `Crash_Beam` | `Body` | 166 | 328 | Alloy_Silver | `0.14 × 1.12 × 0.11` |
| 4 | `Fender_Apron_L` | `Body` | 348 | 684 | Carbon_Trim | `1.16 × 0.21 × 0.41` |
| 5 | `Fender_Apron_R` | `Body` | 348 | 684 | Carbon_Trim | `1.16 × 0.21 × 0.41` |
| 6 | `Fender_Front_L` | `Body` | 7,704 | 15,404 | CarPaint_MetallicBlue | `1.46 × 0.42 × 0.75` |
| 7 | `Fender_Front_R` | `Body` | 7,706 | 15,408 | CarPaint_MetallicBlue | `1.46 × 0.42 × 0.75` |
| 8 | `Fog_Light_L` | `Front_Bumper` | 48 | 92 | DRL_Light | `0.04 × 0.09 × 0.01` |
| 9 | `Fog_Light_R` | `Front_Bumper` | 48 | 92 | DRL_Light | `0.04 × 0.09 × 0.01` |
| 10 | `Front_Bumper` | `Body` | 6,010 | 12,000 | Bumper_Anthracite | `0.50 × 1.65 × 0.50` |
| 11 | `Front_Subframe` | `Body` | 448 | 864 | Carbon_Trim | `1.36 × 1.48 × 0.62` |
| 12 | `Grille_Chrome` | `Front_Bumper` | 854 | 1,588 | Chrome_Steel, Piano_Black | `0.17 × 1.31 × 0.44` |
| 13 | `Grille_Front` | `Front_Bumper` | 364 | 724 | Piano_Black | `0.09 × 0.77 × 0.25` |
| 14 | `Headlight_DRL_L` | `Radiator_Support` | 288 | 560 | DRL_Light | `0.12 × 0.28 × 0.07` |
| 15 | `Headlight_DRL_R` | `Radiator_Support` | 288 | 560 | DRL_Light | `0.12 × 0.28 × 0.07` |
| 16 | `Headlight_L` | `Radiator_Support` | 882 | 1,740 | Piano_Black, Chrome_Steel, Alloy_Silver | `0.22 × 0.36 × 0.17` |
| 17 | `Headlight_Lens_L` | `Radiator_Support` | 230 | 456 | Lens_Clear | `0.18 × 0.34 × 0.18` |
| 18 | `Headlight_Lens_R` | `Radiator_Support` | 230 | 456 | Lens_Clear | `0.18 × 0.34 × 0.18` |
| 19 | `Headlight_R` | `Radiator_Support` | 882 | 1,740 | Piano_Black, Chrome_Steel, Alloy_Silver | `0.22 × 0.36 × 0.17` |
| 20 | `Headlight_Washers` | `Front_Bumper` | 96 | 184 | Bumper_Anthracite | `0.04 × 1.17 × 0.03` |
| 21 | `License_Plate_Front` | `Front_Bumper` | 72 | 132 | White_Studio, Piano_Black, CarPaint_MetallicBlue | `0.03 × 0.49 × 0.10` |
| 22 | `PDC_Sensors_Front` | `Front_Bumper` | 240 | 448 | Piano_Black, Bumper_Anthracite | `0.07 × 1.10 × 0.02` |
| 23 | `Radiator_Support` | `Body` | 616 | 1,188 | Carbon_Trim | `0.21 × 1.41 × 0.53` |

## 3. Капот и моторный отсек Volvo Drive-E T6 AWD (18 шт.)

| # | Объект | Родитель | Вершин | Трис | Материалы | Габариты X / Y / Z (м) |
|---:|---|---|---:|---:|---|---|
| 1 | `Coolant_Tank` | `Body` | 210 | 408 | White_Studio, Coolant_Fluid_Pink, Rubber_Tire | `0.20 × 0.14 × 0.16` |
| 2 | `Engine_Bay` | `Body` | 1,980 | 3,848 | Carbon_Trim | `0.96 × 0.94 × 0.46` |
| 3 | `Engine_Details` | `Body` | 902 | 1,712 | Engine_Badge_Plate, Engine_Spec_Label, Chrome_Steel, Caliper_Red, Washer_Cap_Blue, Warning_Yellow, Piano_Black | `1.02 × 0.93 × 0.14` |
| 4 | `Engine_Metal` | `Body` | 1,996 | 3,896 | Alloy_Silver | `0.84 × 1.22 × 0.52` |
| 5 | `Hood_Hinge_Arm_L` | `Hood_Lid` | 20 | 36 | Carbon_Trim | `0.29 × 0.02 × 0.12` |
| 6 | `Hood_Hinge_Arm_R` | `Hood_Lid` | 20 | 36 | Carbon_Trim | `0.29 × 0.02 × 0.12` |
| 7 | `Hood_Hinge_Mount_L` | `Body` | 38 | 68 | Carbon_Trim | `0.07 × 0.05 × 0.07` |
| 8 | `Hood_Hinge_Mount_R` | `Body` | 38 | 68 | Carbon_Trim | `0.07 × 0.05 × 0.07` |
| 9 | `Hood_Insulation` | `Hood_Lid` | 454 | 880 | Carpet, Piano_Black | `0.84 × 0.84 × 0.12` |
| 10 | `Hood_Lid` | `Body` | 4,448 | 8,864 | CarPaint_MetallicBlue | `1.47 × 1.32 × 0.27` |
| 11 | `Hood_Lift_Bracket_L` | `Hood_Lid` | 8 | 12 | Carbon_Trim | `0.04 × 0.04 × 0.03` |
| 12 | `Hood_Lift_Bracket_R` | `Hood_Lid` | 8 | 12 | Carbon_Trim | `0.04 × 0.04 × 0.03` |
| 13 | `Hood_Lift_Cyl_L` | `Body` | 38 | 72 | Carbon_Trim | `0.38 × 0.03 × 0.35` |
| 14 | `Hood_Lift_Cyl_R` | `Body` | 38 | 72 | Carbon_Trim | `0.38 × 0.03 × 0.35` |
| 15 | `Hood_Lift_Mount_L` | `Body` | 8 | 12 | Carbon_Trim | `0.04 × 0.04 × 0.04` |
| 16 | `Hood_Lift_Mount_R` | `Body` | 8 | 12 | Carbon_Trim | `0.04 × 0.04 × 0.04` |
| 17 | `Hood_Lift_Rod_L` | `Hood_Lid` | 34 | 64 | Chrome_Steel | `0.37 × 0.01 × 0.33` |
| 18 | `Hood_Lift_Rod_R` | `Hood_Lid` | 34 | 64 | Chrome_Steel | `0.37 × 0.01 × 0.33` |

## 4. Корма: задние крылья, съёмный бампер, C-фонари и силовой каркас (17 шт.)

| # | Объект | Родитель | Вершин | Трис | Материалы | Габариты X / Y / Z (м) |
|---:|---|---|---:|---:|---|---|
| 1 | `Exhaust_L` | `Rear_Bumper` | 390 | 756 | Chrome_Steel, Carbon_Trim | `0.06 × 0.15 × 0.07` |
| 2 | `Exhaust_R` | `Rear_Bumper` | 390 | 756 | Chrome_Steel, Carbon_Trim | `0.06 × 0.15 × 0.07` |
| 3 | `Fender_Rear_L` | `Body` | 7,738 | 15,468 | CarPaint_MetallicBlue | `1.36 × 0.46 × 0.71` |
| 4 | `Fender_Rear_R` | `Body` | 7,738 | 15,468 | CarPaint_MetallicBlue | `1.36 × 0.46 × 0.71` |
| 5 | `PDC_Sensors_Rear` | `Rear_Bumper` | 240 | 448 | Piano_Black, Bumper_Anthracite | `0.07 × 1.10 × 0.02` |
| 6 | `Rear_Bumper` | `Body` | 5,512 | 11,008 | Bumper_Anthracite | `0.66 × 1.71 × 0.45` |
| 7 | `Rear_Crash_Beam` | `Body` | 166 | 328 | Alloy_Silver | `0.11 × 1.04 × 0.12` |
| 8 | `Rear_Diffuser` | `Rear_Bumper` | 288 | 572 | Carbon_Trim | `0.11 × 1.16 × 0.11` |
| 9 | `Rear_Reflector_L` | `Rear_Bumper` | 128 | 248 | Caliper_Red, Carbon_Trim | `0.07 × 0.19 × 0.03` |
| 10 | `Rear_Reflector_R` | `Rear_Bumper` | 128 | 248 | Caliper_Red, Carbon_Trim | `0.07 × 0.19 × 0.03` |
| 11 | `Rear_Subframe` | `Body` | 1,196 | 2,324 | Carbon_Trim | `1.33 × 1.48 × 0.72` |
| 12 | `Tail_Housing_L` | `Rear_Subframe` | 290 | 576 | Tail_Housing_Red | `0.13 × 0.28 × 0.21` |
| 13 | `Tail_Housing_R` | `Rear_Subframe` | 290 | 576 | Tail_Housing_Red | `0.13 × 0.28 × 0.21` |
| 14 | `Tail_Lens_L` | `Rear_Subframe` | 290 | 576 | Lens_Clear | `0.11 × 0.29 × 0.22` |
| 15 | `Tail_Lens_R` | `Rear_Subframe` | 290 | 576 | Lens_Clear | `0.11 × 0.29 × 0.22` |
| 16 | `Tail_Light_L` | `Rear_Subframe` | 400 | 780 | Tail_Light, White_Studio | `0.09 × 0.25 × 0.18` |
| 17 | `Tail_Light_R` | `Rear_Subframe` | 400 | 780 | Tail_Light, White_Studio | `0.09 × 0.25 × 0.18` |

## 5. Багажный отсек, крышка, петли, газлифты и оснащение (19 шт.)

| # | Объект | Родитель | Вершин | Трис | Материалы | Габариты X / Y / Z (м) |
|---:|---|---|---:|---:|---|---|
| 1 | `License_Plate_Rear` | `Trunk_Lid` | 72 | 132 | White_Studio, Piano_Black, CarPaint_MetallicBlue | `0.01 × 0.49 × 0.11` |
| 2 | `Trunk_Badge` | `Trunk_Lid` | 512 | 996 | Chrome_Steel | `0.02 × 0.51 × 0.06` |
| 3 | `Trunk_Details` | `Body` | 802 | 1,344 | Chrome_Steel, Rubber_Tire, Piano_Black, Trunk_LED, Carbon_Trim | `1.02 × 0.95 × 0.35` |
| 4 | `Trunk_Floor` | `Body` | 8 | 12 | Carpet | `0.96 × 0.90 × 0.02` |
| 5 | `Trunk_Hinge_Arm_L` | `Trunk_Lid` | 24 | 44 | Carbon_Trim | `0.28 × 0.02 × 0.13` |
| 6 | `Trunk_Hinge_Arm_R` | `Trunk_Lid` | 24 | 44 | Carbon_Trim | `0.28 × 0.02 × 0.13` |
| 7 | `Trunk_Hinge_Mount_L` | `Body` | 38 | 68 | Carbon_Trim | `0.07 × 0.05 × 0.08` |
| 8 | `Trunk_Hinge_Mount_R` | `Body` | 38 | 68 | Carbon_Trim | `0.07 × 0.05 × 0.08` |
| 9 | `Trunk_Lid` | `Body` | 3,338 | 6,656 | CarPaint_MetallicBlue | `0.72 × 1.19 × 0.36` |
| 10 | `Trunk_Lid_Liner` | `Trunk_Lid` | 168 | 308 | Carpet, Piano_Black, Caliper_Red, Trunk_LED | `0.60 × 0.63 × 0.29` |
| 11 | `Trunk_Lift_Bracket_L` | `Trunk_Lid` | 8 | 12 | Carbon_Trim | `0.04 × 0.04 × 0.03` |
| 12 | `Trunk_Lift_Bracket_R` | `Trunk_Lid` | 8 | 12 | Carbon_Trim | `0.04 × 0.04 × 0.03` |
| 13 | `Trunk_Lift_Cyl_L` | `Body` | 38 | 72 | Carbon_Trim | `0.22 × 0.03 × 0.40` |
| 14 | `Trunk_Lift_Cyl_R` | `Body` | 38 | 72 | Carbon_Trim | `0.22 × 0.03 × 0.40` |
| 15 | `Trunk_Lift_Mount_L` | `Body` | 8 | 12 | Carbon_Trim | `0.04 × 0.04 × 0.04` |
| 16 | `Trunk_Lift_Mount_R` | `Body` | 8 | 12 | Carbon_Trim | `0.04 × 0.04 × 0.04` |
| 17 | `Trunk_Lift_Rod_L` | `Trunk_Lid` | 34 | 64 | Chrome_Steel | `0.20 × 0.01 × 0.37` |
| 18 | `Trunk_Lift_Rod_R` | `Trunk_Lid` | 34 | 64 | Chrome_Steel | `0.20 × 0.01 × 0.37` |
| 19 | `Trunk_Tub` | `Body` | 280 | 540 | Carpet | `1.05 × 1.03 × 0.41` |

## 6. Двери, остекление дверей, ручки, петли и дверные карты (24 шт.)

| # | Объект | Родитель | Вершин | Трис | Материалы | Габариты X / Y / Z (м) |
|---:|---|---|---:|---:|---|---|
| 1 | `Door_Card_Front_Left` | `Door_Front_Left` | 604 | 1,164 | Leather_Seat, Wood_Walnut, Piano_Black, Chrome_Steel | `0.86 × 0.07 × 0.49` |
| 2 | `Door_Card_Front_Right` | `Door_Front_Right` | 604 | 1,164 | Leather_Seat, Wood_Walnut, Piano_Black, Chrome_Steel | `0.86 × 0.07 × 0.49` |
| 3 | `Door_Card_Rear_Left` | `Door_Rear_Left` | 604 | 1,164 | Leather_Seat, Wood_Walnut, Piano_Black, Chrome_Steel | `0.83 × 0.07 × 0.44` |
| 4 | `Door_Card_Rear_Right` | `Door_Rear_Right` | 604 | 1,164 | Leather_Seat, Wood_Walnut, Piano_Black, Chrome_Steel | `0.83 × 0.07 × 0.44` |
| 5 | `Door_Front_Left` | `Body` | 3,262 | 6,512 | CarPaint_MetallicBlue, Piano_Black, Chrome_Steel | `0.96 × 0.32 × 1.16` |
| 6 | `Door_Front_Right` | `Body` | 3,262 | 6,512 | CarPaint_MetallicBlue, Piano_Black, Chrome_Steel | `0.96 × 0.32 × 1.16` |
| 7 | `Door_Glass_Front_Left` | `Door_Front_Left` | 386 | 768 | Glass_Window | `0.80 × 0.26 × 0.44` |
| 8 | `Door_Glass_Front_Right` | `Door_Front_Right` | 386 | 768 | Glass_Window | `0.80 × 0.26 × 0.44` |
| 9 | `Door_Glass_Rear_Left` | `Door_Rear_Left` | 386 | 768 | Glass_Window | `0.94 × 0.27 × 0.43` |
| 10 | `Door_Glass_Rear_Right` | `Door_Rear_Right` | 386 | 768 | Glass_Window | `0.94 × 0.27 × 0.43` |
| 11 | `Door_Handle_Base_Front_Left` | `Door_Front_Left` | 280 | 540 | CarPaint_MetallicBlue, Piano_Black, Chrome_Steel | `0.20 × 0.03 × 0.04` |
| 12 | `Door_Handle_Base_Front_Right` | `Door_Front_Right` | 280 | 540 | CarPaint_MetallicBlue, Piano_Black, Chrome_Steel | `0.20 × 0.03 × 0.04` |
| 13 | `Door_Handle_Base_Rear_Left` | `Door_Rear_Left` | 280 | 540 | CarPaint_MetallicBlue, Piano_Black, Chrome_Steel | `0.20 × 0.03 × 0.04` |
| 14 | `Door_Handle_Base_Rear_Right` | `Door_Rear_Right` | 280 | 540 | CarPaint_MetallicBlue, Piano_Black, Chrome_Steel | `0.20 × 0.03 × 0.04` |
| 15 | `Door_Handle_Front_Left` | `Door_Front_Left` | 355 | 694 | CarPaint_MetallicBlue, Chrome_Steel | `0.15 × 0.04 × 0.03` |
| 16 | `Door_Handle_Front_Right` | `Door_Front_Right` | 355 | 694 | CarPaint_MetallicBlue, Chrome_Steel | `0.15 × 0.04 × 0.03` |
| 17 | `Door_Handle_Rear_Left` | `Door_Rear_Left` | 355 | 694 | CarPaint_MetallicBlue, Chrome_Steel | `0.15 × 0.04 × 0.03` |
| 18 | `Door_Handle_Rear_Right` | `Door_Rear_Right` | 355 | 694 | CarPaint_MetallicBlue, Chrome_Steel | `0.15 × 0.04 × 0.03` |
| 19 | `Door_Hinge_Front_Left` | `Body` | 52 | 96 | Carbon_Trim | `0.02 × 0.02 × 0.41` |
| 20 | `Door_Hinge_Front_Right` | `Body` | 52 | 96 | Carbon_Trim | `0.02 × 0.02 × 0.41` |
| 21 | `Door_Hinge_Rear_Left` | `Body` | 52 | 96 | Carbon_Trim | `0.02 × 0.02 × 0.41` |
| 22 | `Door_Hinge_Rear_Right` | `Body` | 52 | 96 | Carbon_Trim | `0.02 × 0.02 × 0.41` |
| 23 | `Door_Rear_Left` | `Body` | 3,358 | 6,704 | CarPaint_MetallicBlue, Piano_Black, Chrome_Steel | `1.03 × 0.34 × 1.17` |
| 24 | `Door_Rear_Right` | `Body` | 3,358 | 6,704 | CarPaint_MetallicBlue, Piano_Black, Chrome_Steel | `1.03 × 0.34 × 1.17` |

## 7. Ходовая часть: шины, 10-спицевые диски, тормоза и подкрылки (24 шт.)

| # | Объект | Родитель | Вершин | Трис | Материалы | Габариты X / Y / Z (м) |
|---:|---|---|---:|---:|---|---|
| 1 | `Brake_Caliper_FL` | `Body` | 212 | 408 | Caliper_Red, Alloy_Silver | `0.07 × 0.06 × 0.13` |
| 2 | `Brake_Caliper_FR` | `Body` | 212 | 408 | Caliper_Red, Alloy_Silver | `0.07 × 0.06 × 0.13` |
| 3 | `Brake_Caliper_RL` | `Body` | 212 | 408 | Caliper_Red, Alloy_Silver | `0.07 × 0.06 × 0.13` |
| 4 | `Brake_Caliper_RR` | `Body` | 212 | 408 | Caliper_Red, Alloy_Silver | `0.07 × 0.06 × 0.13` |
| 5 | `Brake_Disc_FL` | `Wheel_FL` | 482 | 960 | Brake_Disc_Steel, Carbon_Trim | `0.34 × 0.03 × 0.34` |
| 6 | `Brake_Disc_FR` | `Wheel_FR` | 482 | 960 | Brake_Disc_Steel, Carbon_Trim | `0.34 × 0.03 × 0.34` |
| 7 | `Brake_Disc_RL` | `Wheel_RL` | 482 | 960 | Brake_Disc_Steel, Carbon_Trim | `0.34 × 0.03 × 0.34` |
| 8 | `Brake_Disc_RR` | `Wheel_RR` | 482 | 960 | Brake_Disc_Steel, Carbon_Trim | `0.34 × 0.03 × 0.34` |
| 9 | `Rim_FL` | `Wheel_FL` | 3,373 | 6,676 | Alloy_Silver, Piano_Black, Chrome_Steel | `0.49 × 0.21 × 0.49` |
| 10 | `Rim_FR` | `Wheel_FR` | 3,373 | 6,676 | Alloy_Silver, Piano_Black, Chrome_Steel | `0.49 × 0.20 × 0.49` |
| 11 | `Rim_RL` | `Wheel_RL` | 3,373 | 6,676 | Alloy_Silver, Piano_Black, Chrome_Steel | `0.49 × 0.21 × 0.49` |
| 12 | `Rim_RR` | `Wheel_RR` | 3,373 | 6,676 | Alloy_Silver, Piano_Black, Chrome_Steel | `0.49 × 0.20 × 0.49` |
| 13 | `Wheel_Arch_FL` | `Fender_Front_L` | 806 | 1,608 | CarPaint_MetallicBlue | `0.83 × 0.12 × 0.47` |
| 14 | `Wheel_Arch_FR` | `Fender_Front_R` | 806 | 1,608 | CarPaint_MetallicBlue | `0.83 × 0.12 × 0.47` |
| 15 | `Wheel_Arch_RL` | `Fender_Rear_L` | 806 | 1,608 | CarPaint_MetallicBlue | `0.82 × 0.11 × 0.47` |
| 16 | `Wheel_Arch_RR` | `Fender_Rear_R` | 806 | 1,608 | CarPaint_MetallicBlue | `0.82 × 0.11 × 0.47` |
| 17 | `Wheel_FL` | `Body` | 6,272 | 12,544 | Rubber_Tire | `0.70 × 0.24 × 0.70` |
| 18 | `Wheel_FR` | `Body` | 6,272 | 12,544 | Rubber_Tire | `0.70 × 0.24 × 0.70` |
| 19 | `Wheel_RL` | `Body` | 6,272 | 12,544 | Rubber_Tire | `0.70 × 0.24 × 0.70` |
| 20 | `Wheel_RR` | `Body` | 6,272 | 12,544 | Rubber_Tire | `0.70 × 0.24 × 0.70` |
| 21 | `Wheel_Well_FL` | `Body` | 132 | 260 | Carbon_Trim | `0.79 × 0.24 × 0.48` |
| 22 | `Wheel_Well_FR` | `Body` | 132 | 260 | Carbon_Trim | `0.79 × 0.24 × 0.48` |
| 23 | `Wheel_Well_RL` | `Body` | 132 | 260 | Carbon_Trim | `0.79 × 0.24 × 0.48` |
| 24 | `Wheel_Well_RR` | `Body` | 132 | 260 | Carbon_Trim | `0.79 × 0.24 × 0.48` |

## 8. Салон (Interior): торпеда, OLED 12.3"/9", кресла Nappa, ремни и органы управления (19 шт.)

| # | Объект | Родитель | Вершин | Трис | Материалы | Габариты X / Y / Z (м) |
|---:|---|---|---:|---:|---|---|
| 1 | `Air_Vents` | `Body` | 768 | 1,408 | Piano_Black, Chrome_Steel | `0.06 × 1.44 × 0.21` |
| 2 | `Cabin` | `Body` | 1,712 | 3,352 | Leather_Seat | `2.50 × 1.20 × 0.95` |
| 3 | `Cabin_Chrome` | `Body` | 368 | 704 | Chrome_Steel | `0.02 × 0.85 × 0.07` |
| 4 | `Cabin_Seats_Insert` | `Body` | 1,632 | 3,192 | Leather_Bolster | `1.41 × 1.10 × 0.62` |
| 5 | `Carpet` | `Body` | 8 | 12 | Carpet | `1.94 × 1.40 × 0.02` |
| 6 | `Center_Console` | `Body` | 474 | 904 | Wood_Walnut, Piano_Black, Chrome_Steel | `0.45 × 0.23 × 0.13` |
| 7 | `Dash_Trim` | `Body` | 280 | 540 | Wood_Walnut, Chrome_Steel | `0.04 × 1.35 × 0.14` |
| 8 | `Dashboard` | `Body` | 368 | 720 | Leather_Seat | `0.30 × 1.46 × 0.44` |
| 9 | `Headliner` | `Body` | 8 | 12 | Headliner_Fabric | `1.04 × 1.04 × 0.01` |
| 10 | `Instrument_Cluster` | `Body` | 496 | 984 | Piano_Black, Cluster_Digital_UI, Chrome_Steel | `0.05 × 0.34 × 0.13` |
| 11 | `Interior_Roof_Details` | `Body` | 256 | 492 | Headliner_Fabric, Piano_Black, White_Studio | `0.21 × 0.96 × 0.03` |
| 12 | `Pedals` | `Body` | 168 | 308 | Alloy_Silver, Rubber_Tire | `0.11 × 0.38 × 0.23` |
| 13 | `Rearview_Mirror` | `Body` | 230 | 444 | Piano_Black, Chrome_Steel | `0.06 × 0.20 × 0.09` |
| 14 | `Screen` | `Body` | 338 | 628 | Piano_Black, Screen_Sensus_UI, Chrome_Steel | `0.06 × 0.22 × 0.26` |
| 15 | `Seatbelts` | `Body` | 448 | 824 | Rubber_Tire, Piano_Black, Chrome_Steel, Caliper_Red | `1.01 × 1.50 × 0.82` |
| 16 | `Speaker_L` | `Door_Front_Left` | 26 | 48 | Alloy_Silver | `0.10 × 0.02 × 0.10` |
| 17 | `Speaker_R` | `Door_Front_Right` | 26 | 48 | Alloy_Silver | `0.10 × 0.02 × 0.10` |
| 18 | `SteeringWheel` | `Body` | 707 | 1,356 | Leather_Seat, Piano_Black, Chrome_Steel | `0.15 × 0.38 × 0.36` |
| 19 | `Steering_Column` | `Body` | 174 | 336 | Piano_Black | `0.10 × 0.31 × 0.09` |

## 9. Камеры и студийное освещение (21 шт.)

| # | Объект | Тип | Коллекция | Позиция (X, Y, Z) |
|---:|---|---|---|---|
| 1 | `Cam_Door_Cutout` | CAMERA | `Studio` | `(-1.35, -3.55, +1.38)` |
| 2 | `Cam_Exterior` | CAMERA | `Studio` | `(+6.05, -5.15, +1.66)` |
| 3 | `Cam_Hood` | CAMERA | `Studio` | `(+3.75, -1.85, +1.85)` |
| 4 | `Cam_Interior` | CAMERA | `Studio` | `(-0.85, -0.25, +1.16)` |
| 5 | `Cam_Interior2` | CAMERA | `Studio` | `(+0.45, -0.45, +1.15)` |
| 6 | `Cam_Rear` | CAMERA | `Studio` | `(-5.90, -4.70, +1.90)` |
| 7 | `Cam_Side` | CAMERA | `Studio` | `(+0.10, -10.20, +1.30)` |
| 8 | `Cam_Trunk` | CAMERA | `Studio` | `(-3.75, -1.75, +1.85)` |
| 9 | `Int_Ceiling` | LIGHT | `Studio` | `(-0.05, +0.00, +1.28)` |
| 10 | `Int_DashLight` | LIGHT | `Studio` | `(+0.35, +0.00, +1.08)` |
| 11 | `Int_EngineLight` | LIGHT | `Studio` | `(+1.65, +0.00, +1.12)` |
| 12 | `Int_RoofLight_F` | LIGHT | `Studio` | `(+0.35, +0.00, +1.30)` |
| 13 | `Int_RoofLight_M` | LIGHT | `Studio` | `(-0.45, +0.00, +1.30)` |
| 14 | `Int_SeatLight` | LIGHT | `Studio` | `(+0.05, +0.25, +0.85)` |
| 15 | `Int_TrunkLight` | LIGHT | `Studio` | `(-1.78, +0.00, +1.02)` |
| 16 | `Studio_Key_Front` | LIGHT | `Studio` | `(+5.40, -3.40, +2.45)` |
| 17 | `Studio_Overhead` | LIGHT | `Studio` | `(+0.15, +0.00, +3.85)` |
| 18 | `Studio_Rim_Rear` | LIGHT | `Studio` | `(-5.40, -3.00, +2.35)` |
| 19 | `Studio_Strip_L` | LIGHT | `Studio` | `(+0.15, +5.40, +2.25)` |
| 20 | `Studio_Strip_R` | LIGHT | `Studio` | `(+0.15, -5.20, +2.05)` |
| 21 | `Studio_Wheel_Fill` | LIGHT | `Studio` | `(+0.15, -6.50, +0.75)` |
