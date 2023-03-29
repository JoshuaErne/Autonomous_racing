import torch
import onnx
import tensorrt as trt

from yolo import F110_YOLO


def convert_torch_to_onnx(input_model_path, output_onnx_path):
    # load saved pytorch model
    model = F110_YOLO().cuda()
    model.load_state_dict(torch.load(input_model_path))

    # initialize a sample model input
    sample_input = torch.zeros(1, 3, 180, 320, dtype=torch.float, device=torch.device('cuda' if torch.cuda.is_available() else 'cpu'))

    # convert pytorch model to onnx
    print("Converting torch model to ONNX...")
    torch.onnx.export(model, sample_input, output_onnx_path, input_names=['input'], output_names=['output'], export_params=True)
    print("Conversion complete.")

    # check converted onnx model is valid
    print("Checking ONNX conversion output...")
    model_onnx = onnx.load(output_onnx_path)
    onnx.checker.check_model(model_onnx)
    print("No error detected.")


def convert_onnx_to_trt(input_onnx_path, output_trt_path):
    # init logger to capture errors and warnings
    TRT_LOGGER = trt.Logger()

    # init TensorRT engine and parse ONNX model
    builder = trt.Builder(TRT_LOGGER)
    explicit_batch = 1 << (int)(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    network = builder.create_network(explicit_batch)
    parser = trt.OnnxParser(network, TRT_LOGGER)

    # allow TensorRT to use up to 1GB of GPU memory for tactic selection
    builder.max_workspace_size = 1 << 30
    # we have only one image in batch
    builder.max_batch_size = 1
    # use FP16 mode if possible
    if builder.platform_has_fast_fp16:
        builder.fp16_mode = True

    # parse ONNX
    print('Parsing ONNX file..')
    with open(input_onnx_path, 'rb') as model:
        parser.parse(model.read())
    print('Completed parsing of ONNX file')

    # generate TensorRT engine optimized for the target platform
    print('Building TensorRT engine...')
    engine = builder.build_cuda_engine(network)
    print("Completed engine creation.")

    # save generated TensorRT engine
    with open(output_trt_path, 'wb') as f:
        f.write(bytearray(engine.serialize()))
    print("TensorRT engine saved.")


if __name__ == "__main__":
    convert_torch_to_onnx("models/model.pt", "models/model.onnx")
    convert_onnx_to_trt("models/model.onnx", "models/model.trt")
