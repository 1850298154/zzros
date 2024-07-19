

import os
import cv2
import numpy as np


import cv2
import glob


def resize(img_array, align_mode):
    _height = len(img_array[0])
    _width = len(img_array[0][0])
    for i in range(1, len(img_array)):
        img = img_array[i]
        height = len(img)
        width = len(img[0])
        if align_mode == 'smallest':
            if height < _height:
                _height = height
            if width < _width:
                _width = width
        else:
            if height > _height:
                _height = height
            if width > _width:
                _width = width

    for i in range(0, len(img_array)):
        img1 = cv2.resize(img_array[i], (_width, _height), interpolation=cv2.INTER_CUBIC)
        img_array[i] = img1

    return img_array, (_width, _height)


# def images_to_video(path):
def images_to_video(img_path_list):
    img_array = []

    # for filename in glob.glob(path + '/*.png'):  #图片格式png
    # for filename in glob.glob(path + '/*.jpg'):  #图片格式png
    for filename in img_path_list:  #图片格式png
        img = cv2.imread(filename)
        if img is None:
            print(filename + " is error!")
            continue
        img_array.append(img)

    # 图片的大小需要一致
    img_array, size = resize(img_array, 'largest')
    fps = 2    # 帧率设置
    
    # 获取当前文件所在的目录路径
    image_dir = os.path.dirname(os.path.abspath(img_path_list[0]))

    # 拼接目标文件的路径
    target_file_path = os.path.join(image_dir, "a_video.mp4")
    def create_file(file_path:str) -> None:
        """创建文件或文件夹：
            - 结尾是'/'，则创建文件夹；
            - 否则，创建普通文件。
        """
        import os
        # 获取目录路径
        dir_path = os.path.dirname(file_path)
        # 如果目录不存在，则创建目录
        if not os.path.exists(dir_path):
            # print('create dir_path ', dir_path)
            os.makedirs(dir_path)

        # 如果文件不存在，则创建文件
        if not os.path.exists(file_path):
            # print('create file_path ', file_path)
            open(file_path, 'w').close()
    create_file(target_file_path)
    out = cv2.VideoWriter(target_file_path, cv2.VideoWriter_fourcc(*'DIVX'), fps, size)

    for i in range(len(img_array)):
        out.write(img_array[i])
    out.release()

    print('视频生成完毕...')

def get_img_path_list(image_dir,start_index,end_index):
    img_path_list=[]
    # 遍历图片并将其添加到视频需要的图片路径list中
    for i in range(start_index, end_index+1):
        img_file = os.path.join(image_dir, f"episode-{i}.jpg")
        img_path_list.append(img_file)
    return img_path_list    

def produce_movie():
    import os

    # 获取当前工作目录
    current_work_dir = os.getcwd()
    print("当前工作目录：", current_work_dir)

    # # 获取当前文件所在的目录
    # file_path = os.path.dirname(os.path.abspath(__file__))
    # print("当前文件所在目录：", file_path)
    
    # image_dir = r'F:\project\2023\07\GuoMeng\MPC-MOTION-PLANING\github-share\MPC-MP\IMPC-BatTest-OB\002_nt\2023-09-08_16-36-43\savefig'
    # image_dir = r'F:\project\2023\07\GuoMeng\MPC-MOTION-PLANING\github-share\MPC-MP\IMPC-BatTest-OB\002_nt\2023-09-09_15-21-12\savefig'
    import output_filename as of 
    image_dir = of.path_dir +r'\savefig\\'
    start_index = 0
    end_index = 500
    img_path_list = get_img_path_list(image_dir,start_index,end_index)
    images_to_video(img_path_list)


def main():
    # path = ".Output/"    # 路径（不要带中文的路径！！！）
    # path = r'F:\project\2023\07\GuoMeng\MPC-MOTION-PLANING\github-share\MPC-MP\IMPC-BatTest-OB\002_nt\2023-09-08_16-36-43\savefig\\'  
    image_dir = r'F:\project\2023\07\GuoMeng\MPC-MOTION-PLANING\github-share\MPC-MP\IMPC-BatTest-OB\002_nt\2023-09-08_16-36-43\savefig'
    image_dir = r'F:\project\2023\07\GuoMeng\MPC-MOTION-PLANING\github-share\MPC-MP\IMPC-BatTest-OB\002_nt\2023-09-09_15-21-12\savefig'
    image_dir = r'004\2023-10-19_15-42-53\savefig'
    image_dir = r'F:\project\2023\07\GuoMeng\ZhuoZhi\github-collaborate\zzd\simpy\sim_figs2\\'
    image_dir = r'F:\project\2023\07\GuoMeng\ZhuoZhi\github-collaborate\zzd\simpy\sim_figs2-10\\'
    image_dir = r'F:\project\2023\07\GuoMeng\ZhuoZhi\github-collaborate\zzd\simpy\sim_figs2-2024-7-18\\'
    # 获取目录下所有文件和文件夹的名称
    img_path_list = os.listdir(image_dir)
    img_path_list = sorted(img_path_list)
    img_path_list = [
            file 
            for file 
            in img_path_list 
            if file.startswith('frame_') 
            and file.endswith('.png')
        ]

    img_path_list = [
        image_dir + i
        for i in img_path_list
    ]
    # start_index = 0
    # end_index = 133
    # img_path_list = get_img_path_list(image_dir,start_index,end_index)
#     img_path_list=[
#     '004/2023-10-19_15-54-59/savefig/episode-110.jpg',
# '004/2023-10-19_15-54-59/savefig/episode-111.jpg',
# '004/2023-10-19_15-54-59/savefig/episode-112.jpg',
# '004/2023-10-19_15-54-59/savefig/episode-113.jpg',
# ]
    print(*img_path_list, sep='\n')
    images_to_video(img_path_list)


if __name__ == "__main__":
    main()

# # exit()



# # path = r'F:\project\2023\07\GuoMeng\MPC-MOTION-PLANING\github-share\MPC-MP\IMPC-BatTest-OB\002_nt\2023-09-08_16-36-43\savefig\\'  # 路径设置（ps:路径不要出现中文！）
# # filelist = os.listdir(path)
# # filelist = sorted(filelist)  # 按照文件数字进行顺序排序

# # fps = 1  # 视频每秒24帧
# # # size = (512, 512)  # 需要转为视频的图片的尺寸
# # size = (831, 828)  # 需要转为视频的图片的尺寸

# # video = cv2.VideoWriter("VideoTest.avi", cv2.VideoWriter_fourcc(*'DIVX'), fps, size)
# # # 视频保存在当前目录下

# # for item in filelist:
# #     # if item.endswith('.png'):
# #     if item.endswith('.jpg'):
# #         # 找到路径中所有后缀名为.png的文件，可以更换为.jpg或其它
# #         item = path + item
# #         img = cv2.imread(item)
# #         video.write(img)

# # video.release()
# # cv2.destroyAllWindows()
# # exit()



# import cv2
# import os

# # 设置视频编码、帧率以及视频大小等参数
# # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
# # fourcc = cv2.VideoWriter_fourcc(*'MJPG')
# # fourcc = cv2.VideoWriter_fourcc(*'MJPG')
# # fourcc = cv2.VideoWriter_fourcc(*'XVID')
# fourcc = cv2.VideoWriter_fourcc(*'DIVX')

# fps = 30
# fps = 5
# fps = 1
# (831, 828)
# width = 640
# height = 480
# width = 831
# height = 828


# # 指定图片目录路径和起止编号
# image_dir = 'path'  # 请替换为实际的目录路径
# image_dir = r''
# image_dir = r''
# image_dir = r'F:\project\2023\07\GuoMeng\MPC-MOTION-PLANING\github-share\MPC-MP\IMPC-BatTest-OB\002_nt\2023-09-08_16-36-43\savefig'
# start_index = 1
# end_index = 330

# # # 获取当前文件所在的目录路径
# # current_dir = os.path.dirname(os.path.abspath(image_dir))

# # 拼接目标文件的路径
# # target_file_path = os.path.join(image_dir, "output.mp4")
# target_file_path = os.path.join(image_dir, "output.avi")

# # 创建 VideoWriter 对象
# out = cv2.VideoWriter(target_file_path, fourcc, fps, (width, height))

# # 遍历图片并将其添加到视频中
# for i in range(start_index, end_index+1):
#     img_file = os.path.join(image_dir, f"episode-{i}.jpg")
#     # print(img_file)
#     img = cv2.imread(img_file)
#     if img is not None:
#         # print('not null')
#         out.write(img)

# # 释放资源
# out.release()
# cv2.destroyAllWindows()
