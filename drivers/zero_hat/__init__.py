import sys as _sys
from typing import Union as _Union

from loguru import logger

from .application import ZHAppLayer as __ZH_without_remote_layer__
from .remote import ZeroHatClient, ZeroHatServer


class ZeroHat(__ZH_without_remote_layer__):
    """
    本地模式下的ZeroHat类
    """

    pass  # 只是个别名


ZHLike = _Union[ZeroHat, ZeroHatClient, ZeroHatServer]  # type annotations
__all__ = ["ZeroHat", "ZeroHatClient", "ZeroHatServer", "logger", "ZHLike"]

logger.remove()
logger.add(
    "fc_log/{time}.log",
    retention="1day",
    level="DEBUG",
    backtrace=True,
    diagnose=True,
    filter=lambda record: "debug" not in record["extra"],
)
logger.add(
    "fc_log/navigation_debug_{time}.log",
    retention="1day",
    filter=lambda record: "debug" in record["extra"],
    level="DEBUG",
)
logger.add(
    _sys.stdout, filter=lambda record: "debug" not in record["extra"], level="DEBUG"
)
