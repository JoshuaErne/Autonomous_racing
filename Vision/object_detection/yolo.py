import torch
import torch.nn as nn
import cv2

import pycuda.driver as cuda
import pycuda.autoinit

import numpy as np
import tensorrt as trt

import matplotlib.patches as patches
import matplotlib.pyplot as plt


class F110_YOLO(torch.nn.Module):
    def __init__(self):
        super(F110_YOLO, self).__init__()
        self.conv1 = nn.Conv2d(3, 16, kernel_size = 4, padding = 1, stride = 2)
        self.batchnorm1 = nn.BatchNorm2d(16)
        self.relu1 = nn.ReLU(inplace = True)

        self.conv2 = nn.Conv2d(16, 32, kernel_size = 4, padding = 1, stride = 2)
        self.batchnorm2 = nn.BatchNorm2d(32)
        self.relu2 = nn.ReLU(inplace = True)

        self.conv3 = nn.Conv2d(32, 64, kernel_size = 4, padding = 1, stride = 2)
        self.batchnorm3 = nn.BatchNorm2d(64)
        self.relu3 = nn.ReLU(inplace = True)
        
        self.conv4 = nn.Conv2d(64, 128, kernel_size = 4, padding = 1, stride = 2)
        self.batchnorm4 = nn.BatchNorm2d(128)
        self.relu4 = nn.ReLU(inplace = True)

        self.conv5 = nn.Conv2d(128, 256, kernel_size = 4, padding = 1, stride = 2)
        self.batchnorm5 = nn.BatchNorm2d(256)
        self.relu5 = nn.ReLU(inplace = True)

        self.conv6 = nn.Conv2d(256, 512, kernel_size = 3, padding = 1, stride = 1)
        self.batchnorm6 = nn.BatchNorm2d(512)
        self.relu6 = nn.ReLU(inplace = True)

        self.conv7 = nn.ConvTranspose2d(512, 256, kernel_size = 3, padding = 1, stride = 1)
        self.batchnorm7 = nn.BatchNorm2d(256)
        self.relu7 = nn.ReLU(inplace = True)

        self.conv8 = nn.ConvTranspose2d(256, 128, kernel_size = 3, padding = 1, stride = 1)
        self.batchnorm8 = nn.BatchNorm2d(128)
        self.relu8 = nn.ReLU(inplace = True)

        self.conv9 = nn.Conv2d(128, 5, kernel_size = 1, padding = 0, stride = 1)
        self.relu9 = nn.ReLU()
    
    def forward(self, x):
        debug = 0 # change this to 1 if you want to check network dimensions
        if debug == 1: print(0, x.shape)
        x = torch.relu(self.batchnorm1(self.conv1(x)))
        if debug == 1: print(1, x.shape)
        x = torch.relu(self.batchnorm2(self.conv2(x)))
        if debug == 1: print(2, x.shape)
        x = torch.relu(self.batchnorm3(self.conv3(x)))
        if debug == 1: print(3, x.shape)
        x = torch.relu(self.batchnorm4(self.conv4(x)))
        if debug == 1: print(4, x.shape)
        x = torch.relu(self.batchnorm5(self.conv5(x)))
        if debug == 1: print(5, x.shape)
        x = torch.relu(self.batchnorm6(self.conv6(x)))
        if debug == 1: print(6, x.shape)
        x = torch.relu(self.batchnorm7(self.conv7(x)))
        if debug == 1: print(7, x.shape)
        x = torch.relu(self.batchnorm8(self.conv8(x)))
        if debug == 1: print(8, x.shape)
        x = self.conv9(x)
        if debug == 1: print(9, x.shape)
        x = torch.cat([x[:, 0:3, :, :], torch.sigmoid(x[:, 3:5, :, :])], dim=1)

        return x

    def get_loss(self, result, truth, lambda_coord = 5, lambda_noobj = 1):
        x_loss = (result[:, 1, :, :] - truth[:, 1, :, :]) ** 2
        y_loss = (result[:, 2, :, :] - truth[:, 2, :, :]) ** 2
        w_loss = (torch.sqrt(result[:, 3, :, :]) - torch.sqrt(truth[:, 3, :, :])) ** 2
        h_loss = (torch.sqrt(result[:, 4, :, :]) - torch.sqrt(truth[:, 4, :, :])) ** 2
        class_loss_obj = truth[:, 0, :, :] * (truth[:, 0, :, :] - result[:, 0, :, :]) ** 2
        class_loss_noobj = (1 - truth[:, 0, :, :]) * lambda_noobj * (truth[:, 0, :, :] - result[:, 0, :, :]) ** 2

        total_loss = torch.sum(lambda_coord * truth[:, 0, :, :] * (x_loss + y_loss + w_loss + h_loss) + class_loss_obj + class_loss_noobj)
        
        return total_loss


def load_trt_engine(trt_file_path):
    print("Loading TensorRT engine from file...")
    TRT_LOGGER = trt.Logger()
    runtime = trt.Runtime(TRT_LOGGER)
    engine, context = None, None
    with open(trt_file_path, 'rb') as f:
        engine_bytes = f.read()
        engine = runtime.deserialize_cuda_engine(engine_bytes)
        context = engine.create_execution_context()
        print("TensorRT engine loaded from file.")
    return engine, context

def inference(image, engine, context):
    # get sizes of input and output and allocate memory required for input data and for output data
    for binding in engine:
        if engine.binding_is_input(binding):
            input_shape = engine.get_binding_shape(binding)
            input_size = trt.volume(input_shape) * engine.max_batch_size * np.dtype(np.float32).itemsize  # in bytes
            device_input = cuda.mem_alloc(input_size)
        else:
            output_shape = engine.get_binding_shape(binding)
            # create page-locked memory buffers (i.e. won't be swapped to disk)
            host_output = cuda.pagelocked_empty(trt.volume(output_shape) * engine.max_batch_size, dtype=np.float32)
            device_output = cuda.mem_alloc(host_output.nbytes)

    # Create a stream in which to copy inputs/outputs and run inference.
    stream = cuda.Stream()

    # preprocess input data
    host_input = np.array(preprocess_image(image), dtype=np.float32, order='C')
    cuda.memcpy_htod_async(device_input, host_input, stream)

    # run inference
    context.execute_async(bindings=[int(device_input), int(device_output)], stream_handle=stream.handle)
    cuda.memcpy_dtoh_async(host_output, device_output, stream)
    stream.synchronize()

    # reshape model output
    output = host_output.reshape(output_shape)

    # post-process output
    output = postprocess_output(output)
    return output

def DisplayLabel(img, bboxs, color=[1,0,0]):
    image = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2RGB)
    fig, ax = plt.subplots(1)
    if len(bboxs) == 1:
        bbox = bboxs[0]
        ax.add_patch(patches.Rectangle((bbox[0] - bbox[2]/2, bbox[1] - bbox[3]/2), bbox[2], bbox[3], linewidth=1, edgecolor=color, facecolor='none'))
    elif len(bboxs) > 1:
        for bbox in bboxs:
            ax.add_patch(patches.Rectangle((bbox[0] - bbox[2]/2, bbox[1] - bbox[3]/2), bbox[2], bbox[3], linewidth=1, edgecolor=color, facecolor='none'))
    ax.imshow(image)
    plt.show()

def bbox_convert(c_x, c_y, w, h):
    """
    Convert from [c_x, c_y, w, h] to [x_l, y_l, x_r, y_r]
    """
    return [c_x - w/2, c_y - h/2, c_x + w/2, c_y + h/2]

def bbox_convert_r(x_l, y_l, x_r, y_r):
    """
    Convert from [x_l, y_l, x_r, x_r] to [c_x, c_y, w, h]
    """
    return [x_l/2 + x_r/2, y_l/2 + y_r/2, x_r - x_l, y_r - y_l]

def grid_cell(cell_indx, cell_indy):
    """
    Convert feature map coord to image coord
    """
    final_dim = [5, 10]
    input_dim = [180, 320]
    anchor_size = [(input_dim[0] / final_dim[0]), (input_dim[1] / final_dim[1])]
    stride_0 = anchor_size[1]
    stride_1 = anchor_size[0]
    return np.array([cell_indx * stride_0, cell_indy * stride_1, cell_indx * stride_0 + stride_0, cell_indy * stride_1 + stride_1])

def voting_suppression(result_box, iou_threshold = 0.5):
    votes = np.zeros(result_box.shape[0])
    for ind, box in enumerate(result_box):
        for box_validation in result_box:
            if IoU(box_validation, box) > iou_threshold:
                votes[ind] += 1
    return (-votes).argsort()

def IoU(a, b):
    """
    Calculate intersection-over-union.
    """
    inter_w = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    inter_h = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter_ab = inter_w * inter_h
    area_a = (a[3] - a[1]) * (a[2] - a[0])
    area_b = (b[3] - b[1]) * (b[2] - b[0])
    union_ab = area_a + area_b - inter_ab
    return inter_ab / union_ab

def preprocess_image(img):
    input_dim = [180, 320]
    img = cv2.resize(img, (input_dim[1], input_dim[0]))
    img = np.transpose(img, (2, 0, 1))
    return img

def label_to_box_xyxy(result, threshold = 0.9):
    final_dim = [5, 10]
    input_dim = [180, 320]
    anchor_size = [(input_dim[0] / final_dim[0]), (input_dim[1] / final_dim[1])]

    validation_result = []
    result_prob = []
    for ind_row in range(final_dim[0]):
        for ind_col in range(final_dim[1]):
            grid_info = grid_cell(ind_col, ind_row)
            validation_result_cell = []
            if result[0, ind_row, ind_col] >= threshold:
                c_x = grid_info[0] + anchor_size[1]/2 + result[1, ind_row, ind_col]
                c_y = grid_info[1] + anchor_size[0]/2 + result[2, ind_row, ind_col]
                w = result[3, ind_row, ind_col] * input_dim[1]
                h = result[4, ind_row, ind_col] * input_dim[0]
                x1, y1, x2, y2 = bbox_convert(c_x, c_y, w, h)
                x1 = np.clip(x1, 0, input_dim[1])
                x2 = np.clip(x2, 0, input_dim[1])
                y1 = np.clip(y1, 0, input_dim[0])
                y2 = np.clip(y2, 0, input_dim[0])
                validation_result_cell.append(x1)
                validation_result_cell.append(y1)
                validation_result_cell.append(x2)
                validation_result_cell.append(y2)
                result_prob.append(result[0, ind_row, ind_col])
                validation_result.append(validation_result_cell)
    validation_result = np.array(validation_result)
    result_prob = np.array(result_prob)
    return validation_result, result_prob

def display_postprocess_output(image, output):
    image = preprocess_image(image)
    image = torch.from_numpy(image).type('torch.FloatTensor')
    DisplayLabel(np.transpose(image.numpy(), (1, 2, 0)), output)

def postprocess_output(output):
    voting_iou_threshold = 0.5
    confi_threshold = 0.4

    bboxs, _ = label_to_box_xyxy(output[0], confi_threshold)
    vote_rank = voting_suppression(bboxs, voting_iou_threshold)

    if len(vote_rank) == 0:
        return []

    bbox = bboxs[vote_rank[0]]
    [c_x, c_y, w, h] = bbox_convert_r(bbox[0], bbox[1], bbox[2], bbox[3])
    bboxs_2 = np.array([[c_x, c_y, w, h]])
    return bboxs_2

if __name__ == "__main__":
    image = cv2.imread("images/test_car_x60cm.png") / 255.0
    engine, context = load_trt_engine("models/model.trt")
    output = inference(image, engine, context)
    display_postprocess_output(image, output)
