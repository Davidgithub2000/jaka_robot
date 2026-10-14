import random
from pathlib import Path
from tqdm import tqdm
import shutil

# 定义原始数据存放路径dir和数据集分配路径path
dir = './origin_data'
dir = Path(dir)
path = './datasets/VOC'
path = Path(path)
origin_images_path = dir / 'images'
origin_labels_path = dir / 'labels'


# 定义随机数种子和分配比例
random.seed(0)
rate = 0.2
images_list = list(origin_images_path.iterdir())
val_num = int(rate * len(images_list))
val_list = random.sample(images_list, val_num)
for val in val_list:
    if val in images_list:
         del images_list[images_list.index(val)]
train_list = images_list
train_val = {'train': train_list, 'val': val_list}

# 数据集分配路径创建并分配数据集
for image_set in ('train', 'val'):
    images_path = path / 'images' / image_set
    labs_path = path / 'labels' / image_set
    images_path.mkdir(exist_ok=True, parents=True)
    labs_path.mkdir(exist_ok=True, parents=True)
    # 分配数据集
    for data in tqdm(train_val[image_set]):
        shutil.copy(data, images_path / data.name)
        print(data)
        shutil.copy(Path(str(data).replace('images', 'labels', 1).replace('jpg','txt')),
                    labs_path / Path(str(data.name).replace('jpg', 'txt')))
