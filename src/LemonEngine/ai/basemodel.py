import os
from loguru import logger
from typing import *
from abc import ABC, abstractmethod
from openvino.runtime import Core


class BaseModel(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def predict(self, x):
        raise NotImplementedError


class BaseOpenvinoModel(BaseModel, ABC):
    def __init__(self,
                 model_name: str,
                 device_name: str = "CPU",
                 models_dir: Optional[str] = None,
                 ) -> None:
        super().__init__()

        models_dir = models_dir or os.environ.get("OPENVINO_DIR") or "~/models/openvino"
        models_dir = os.path.expanduser(models_dir)
        model_path = f"{models_dir}/{model_name}/{model_name}.xml"

        if not os.path.exists(model_path):
            model_path = f"{models_dir}/{model_name}_openvino_model/{model_name}.xml"
            if not os.path.exists(model_path):
                logger.error(f"Could not find OpenVINO model at {model_path} with name {model_name}")

        logger.info(f"Loading model: {model_name}")
        logger.debug(f"Model Dir: {models_dir}")
        logger.debug(f"Model Path: {model_path}")

        core = Core()
        core.set_property({'CACHE_DIR': '/tmp/openvino_cache/'})

        logger.debug("Reading model...")
        model = core.read_model(model=model_path)

        logger.debug("Compiling model...")
        self.model = core.compile_model(model=model, device_name=device_name)

        logger.success("Successfully loaded model!")

    def predict(self, x):
        return self.model(inputs=[x])
