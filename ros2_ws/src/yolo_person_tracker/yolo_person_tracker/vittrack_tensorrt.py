"""Explicit local VitTrack TensorRT 10 FP32 runtime; single inference worker."""
import numpy as np


SHAPES = {'template': (1, 3, 128, 128), 'search': (1, 3, 256, 256),
          'output1': (1, 1, 16, 16), 'output2': (1, 2, 16, 16), 'output3': (1, 2, 16, 16)}


class VitTrackTensorRT:
    def __init__(self, path):
        import tensorrt as trt
        import torch
        if not torch.cuda.is_available():
            raise ValueError('TensorRT VitTrack requires CUDA')
        self.torch = torch
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        self.engine = self.runtime.deserialize_cuda_engine(path.read_bytes())
        if self.engine is None:
            raise ValueError('Cannot load VitTrack engine on this TensorRT/device')
        names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
        if set(names) != set(SHAPES):
            raise ValueError('VitTrack engine requires template/search and output1/2/3')
        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise ValueError('Cannot create VitTrack execution context')
        self.buffers = {}
        for name, shape in SHAPES.items():
            mode = trt.TensorIOMode.INPUT if name in ('template', 'search') else trt.TensorIOMode.OUTPUT
            if (self.engine.get_tensor_mode(name) != mode
                    or self.engine.get_tensor_dtype(name) != trt.DataType.FLOAT
                    or tuple(self.engine.get_tensor_shape(name)) != shape):
                raise ValueError('VitTrack engine IO must retain static FP32 shapes: '+name)
            self.buffers[name] = torch.empty(shape, device='cuda', dtype=torch.float32)
            if not self.context.set_tensor_address(name, self.buffers[name].data_ptr()):
                raise ValueError('VitTrack engine buffer binding failed')
        self.stream = torch.cuda.Stream()

    def __call__(self, template, search):
        for name, tensor in (('template', template), ('search', search)):
            if tensor.shape != SHAPES[name] or tensor.dtype != np.float32 or not np.isfinite(tensor).all():
                raise ValueError('VitTrack requires established finite FP32 '+name+' tensor')
        torch = self.torch
        with torch.inference_mode(), torch.cuda.stream(self.stream):
            self.buffers['template'].copy_(torch.from_numpy(np.ascontiguousarray(template)))
            self.buffers['search'].copy_(torch.from_numpy(np.ascontiguousarray(search)))
            if not self.context.execute_async_v3(stream_handle=self.stream.cuda_stream):
                raise RuntimeError('VitTrack TensorRT inference failed')
            self.stream.synchronize()
            return [self.buffers[name].cpu().numpy().copy() for name in ('output1', 'output2', 'output3')]
