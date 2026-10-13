"""Single-worker OSNet TensorRT 10 FP32 runtime; no engine building or fallback."""
import numpy as np


class OSNetTensorRT:
    def __init__(self, path):
        import tensorrt as trt
        import torch
        if not torch.cuda.is_available():
            raise ValueError('TensorRT ReID requires CUDA')
        self.torch = torch
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        self.engine = self.runtime.deserialize_cuda_engine(path.read_bytes())
        if self.engine is None:
            raise ValueError('Cannot load OSNet engine on this TensorRT/device')
        names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
        inputs = [n for n in names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT]
        outputs = [n for n in names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.OUTPUT]
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError('OSNet engine requires one image input and one descriptor output')
        self.input_name, self.output_name = inputs[0], outputs[0]
        if any(self.engine.get_tensor_dtype(n) != trt.DataType.FLOAT for n in names):
            raise ValueError('OSNet engine IO must be FP32')
        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise ValueError('Cannot create OSNet execution context')
        shape = (1, 3, 256, 128)
        declared = tuple(self.engine.get_tensor_shape(self.input_name))
        if -1 in declared:
            if not self.context.set_input_shape(self.input_name, shape):
                raise ValueError('OSNet image shape is not supported')
        elif declared != shape:
            raise ValueError('OSNet engine input must be 1x3x256x128')
        output_shape = tuple(self.context.get_tensor_shape(self.output_name))
        if any(v <= 0 for v in output_shape) or int(np.prod(output_shape)) != 512:
            raise ValueError('OSNet engine must return one 512-D descriptor')
        self.input = torch.empty(shape, device='cuda', dtype=torch.float32)
        self.output = torch.empty(output_shape, device='cuda', dtype=torch.float32)
        self.stream = torch.cuda.Stream()
        for name, tensor in ((self.input_name, self.input), (self.output_name, self.output)):
            if not self.context.set_tensor_address(name, tensor.data_ptr()):
                raise ValueError('OSNet engine buffer binding failed')

    def run(self, tensor):
        if tensor.shape != (1, 3, 256, 128) or tensor.dtype != np.float32:
            raise ValueError('OSNet requires the established ImageNet FP32 tensor')
        torch = self.torch
        with torch.inference_mode(), torch.cuda.stream(self.stream):
            self.input.copy_(torch.from_numpy(np.ascontiguousarray(tensor)))
            if not self.context.execute_async_v3(stream_handle=self.stream.cuda_stream):
                raise RuntimeError('OSNet TensorRT inference failed')
            self.stream.synchronize()
            return self.output.cpu().numpy().reshape(-1).copy()
