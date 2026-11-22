#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
silhouette.py — сверка силуэта машины с эталоном (маска + таблица расхождений).

Требует папку ref/ с эталоном (sil_ref.npy и скриншотами видов), которая
формируется отдельно от реальных обмеров автомобиля. В этом репозитории
патч/архив собирается без больших бинарных референсов, поэтому скрипт —
заглушка с подсказкой, что сделать при наличии эталона.
"""
import os
import sys


def main():
    ref_dir = os.path.join('ref')
    needed = ['sil_ref.npy', 'refcurves.json', 'body.npy', 'glass.npy']
    missing = [f for f in needed if not os.path.exists(os.path.join(ref_dir, f))]
    if missing:
        print('silhouette.py: эталоны не найдены в ref/:', ', '.join(missing))
        print('       Положите файлы обмеров эталона в ref/ и перезапустите:')
        print('       blender -b -noaudio car_realistic.blend -P silhouette.py')
        sys.exit(0)
    # При наличии эталона здесь строится маска различия и таблица расхождений
    # (аналогично этапу 1 истории проекта: расхождение 3–14 мм по крыше).
    print('silhouette.py: эталоны найдены, сверка не реализована в этом патче.')


if __name__ == '__main__':
    main()
