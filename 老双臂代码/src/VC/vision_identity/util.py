import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

def get_aim_(boxes, classes, color_intrin_part, aligned_depth_frame, shape, object_classify):

    target_xyz_trues, track_cls, Priority, size_cls = [], [], [], []
    center_image = shape
    try:
        for box, cls in zip(boxes, classes):
            cx = int((box[0] + box[2]) / 2)
            cy = int((box[1] + box[3]) / 2)
            if cls == object_classify:
            #if object_classify and cy < 0.8 and cy > 0.2:
                cx = int((box[0] + box[2]) / 2)
                cy = int((box[1] + box[3]) / 2)
                #priority = (center_image[1]//2-cx)**2 + (center_image[0]//2-cy)**2
                target_depth = aligned_depth_frame.get_distance(int(cx), int(cy))
                if target_depth > 2: # 过滤深度大于2m的目标
                    continue
                priority = target_depth
                target_xyz_true = [(cx - color_intrin_part[0]) * target_depth / color_intrin_part[2],
                                   (cy - color_intrin_part[1]) * target_depth / color_intrin_part[3],
                                   target_depth]
                target_xyz_trues.append(target_xyz_true)
                track_cls.append(cls)
                Priority.append(priority)
#            elif:
#                cx = int((box[0] + box[2]) / 2)
#                cy = int((box[1] + box[3]) / 2)
#                priority = (center_image[1] // 2 - cx) ** 2 + (center_image[0] // 2 - cy) ** 2
#                target_depth = aligned_depth_frame.get_distance(int(cx), int(cy))
#                target_xyz_true = [(cx - color_intrin_part[0]) * target_depth / #color_intrin_part[2],
#                                   (cy - color_intrin_part[1]) * target_depth / #color_intrin_part[3],
#                                   target_depth]
#                target_xyz_trues.append(target_xyz_true)
#                track_cls.append(cls)
#                Priority.append(priority)
        # sort
        Priority = np.asarray(Priority, dtype=np.int_).reshape(1, -1)
        priority_index = np.arange(0, len(Priority[0])).reshape(1, -1)
        p_index_combine = np.vstack((Priority, priority_index))
        priority_sort_index = np.argsort(p_index_combine[0])
        #print("sorted idx:", priority_sort_index)
        target_xyz_trues = target_xyz_trues[priority_sort_index[0]]
        #print('target_xyz_trues: ', target_xyz_trues, '\n', 'track_cls: ', track_cls)
        track_cls = np.array(track_cls, np.int_)
        category = track_cls[priority_sort_index]
        #print("target:", target_xyz_trues, 'category:',category)
    except:
        target_xyz_trues = []
        category = np.array([4], np.int_)
    return target_xyz_trues, category


def read_yaml(path):
    infile = cv2.FileStorage(path,cv2.FILE_STORAGE_READ)
    rotate = infile.getNode("Rotate").mat()
    translation = infile.getNode("Translation").mat()
    temp = np.concatenate([rotate,translation],-1)
    eye = [
        [0,0,0,1]
    ]
    eye = np.array(eye)
    out = np.concatenate([temp,eye],0)
    return out


def imagexyz2wordxyz(target_xyz_trues,track_cls,yaml_path):
    data = []
    if target_xyz_trues==[]:
        c, x, y, z = 4, 0, 0, 0
        output = np.array([[0, 0, 0, 0,
                    0, 0, 0, 0,
                    0, 0, 0, 0,
                    0, 0, 0, 0]]).reshape(4,4)
        data.append([c,x,y,z])
        return data, output
    try:
        exter = read_yaml(yaml_path)
    except:
        raise ("eyeToand_calibration.yaml don't exit")
    
    aim = np.eye(3, 3)
    aim = np.concatenate([aim, np.array(target_xyz_trues).reshape(3,1)], 1)
    aim = np.concatenate([aim, np.array([0, 0, 0, 1]).reshape(1, 4)], 0)
    output = exter.dot(aim)
    x,y,z = output[0,3],output[1,3],output[2,3]
    data.append([int(track_cls[0]), x, y, z])
    return data, output

def imagexyz2wordxyz_eyeinhand(target_xyz_trues,track_cls,yaml_path):
    data = []
    if target_xyz_trues==[]:
        c, x, y, z = 4, 0, 0, 0
        output = np.array([[0, 0, 0, 0,
                    0, 0, 0, 0,
                    0, 0, 0, 0,
                    0, 0, 0, 0]]).reshape(4,4)
        data.append([c,x,y,z])
        return data, output
    try:
        exter = read_yaml(yaml_path)
    except:
        raise ("eyeToand_calibration.yaml don't exit")
    exter_array = np.array([-0.0623515, 0.113702, 0.203269, 0.562945, -0.064601, -132.302])
    theta = (np.deg2rad(exter_array[3]), np.deg2rad(exter_array[4]), np.deg2rad(exter_array[5]))
    exter_matrix, jacob = cv2.Rodrigues(theta)
    #print('exter_matrix:', exter_matrix) 
    exter_pos = np.array([exter_array[0], exter_array[1], exter_array[2]])
    #print('exter_pos: ', exter_pos)
    exter = np.concatenate([exter_matrix, exter_pos.reshape(3,1)], 1)
    exter = np.concatenate([exter, np.array([0, 0, 0, 1]).reshape(1, 4)], 0)
    #print('exter 1: ', exter)
    aim = np.eye(3, 3)
    aim = np.concatenate([aim, np.array(target_xyz_trues).reshape(3,1)], 1)
    aim = np.concatenate([aim, np.array([0, 0, 0, 1]).reshape(1, 4)], 0)
    output = exter.dot(aim)
    x,y,z = output[0,3],output[1,3],output[2,3]
    data.append([int(track_cls[0]), x, y, z])
    return data, output


def imagexyz2wordxyz_eyeinhand(target_xyz_trues,track_cls,yaml_path):
    data = []
    if target_xyz_trues==[]:
        c, x, y, z = 4, 0, 0, 0
        output = np.array([[0, 0, 0, 0,
                    0, 0, 0, 0,
                    0, 0, 0, 0,
                    0, 0, 0, 0]]).reshape(4,4)
        data.append([c,x,y,z])
        return data, output
    try:
        exter = read_yaml(yaml_path)
    except:
        raise ("eyeToand_calibration.yaml don't exit")
    #眼在手标定结果
    #exter_array = np.array([-0.0683912, 0.0926435, -0.0288465, 0.310724, -0.487696, -131.462])
    #theta = [exter_array[3], exter_array[4], exter_array[5]]
    #exter_matrix = R.from_euler('xyz', theta, degrees=True)
    #exter_matrix, jacob = cv2.Rodrigues(theta)
    #print('exter_matrix:', exter_matrix.as_matrix()) 
    #exter_pos = np.array([exter_array[0], exter_array[1], exter_array[2]])
    #print('exter_pos: ', exter_pos)
    #exter = np.concatenate([exter_matrix.as_matrix(), exter_pos.reshape(3,1)], 1)
    #exter = np.concatenate([exter, np.array([0, 0, 0, 1]).reshape(1, 4)], 0)
    #print('exter 1: ', exter)
    aim = np.eye(3, 3)
    aim = np.concatenate([aim, np.array(target_xyz_trues).reshape(3,1)], 1)
    aim = np.concatenate([aim, np.array([0, 0, 0, 1]).reshape(1, 4)], 0)
    output = exter.dot(aim)
    #output = np.linalg.inv(exter).dot(aim)
    print('eye2tool : ', output)
    x,y,z = output[0,3],output[1,3],output[2,3]
    data.append([int(track_cls[0]), x, y, z])
    return data, output









