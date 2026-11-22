#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
silhouette.py — сверка силуэта машины с эталоном (маска + таблица расхождений).

Требует папку ref/ с эталонными файлами обмеров (sil_ref.npy, refcurves.json,
body.npy, glass.npy). Поскольку большие бинарные эталоны не хранятся в Git,
скрипт проверяет их наличие и завершается с ненулевым кодом, если эталоны не
подложены (или с кодом 0 при флаге --allow-missing).
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__)) or os.getcwd()
NEEDED_FILES = ('sil_ref.npy', 'refcurves.json', 'body.npy', 'glass.npy')


def parse_args(argv):
    parser = argparse.ArgumentParser(description='Сверка силуэта автомобиля с эталоном')
    parser.add_argument('--ref-dir', default=os.path.join(HERE, 'ref'),
                        help='Директория с эталонами (по умолчанию ref/)')
    parser.add_argument('--allow-missing', action='store_true',
                        help='Завершиться с кодом 0, если эталоны отсутствуют')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv or [])
    ref_dir = os.path.abspath(args.ref_dir)
    missing = [f for f in NEEDED_FILES if not os.path.exists(os.path.join(ref_dir, f))]
    if missing:
        print(f'silhouette.py: эталоны не найдены в {ref_dir}:', ', '.join(missing))
        print('       Положите файлы обмеров эталона в ref/ и перезапустите:')
        print('       blender -b -noaudio car_realistic.blend -P silhouette.py')
        sys.exit(0 if args.allow_missing else 2)
    print('silhouette.py: эталоны найдены, но алгоритм сверки пока не реализован.')
    sys.exit(3)


if __name__ == '__main__':
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    main(argv)
