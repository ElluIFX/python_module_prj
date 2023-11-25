from typing import Dict, List, Literal, Optional, Union, final

from loguru import logger

from .templates import (
    BaseInterfaceTemplate,
    GPIOInterfaceTemplate,
    I2CInterfaceTemplate,
    SPIInterfaceTemplate,
    UARTInterfaceTemplate,
)

__all__ = [
    "BaseInterfaceBuilder",
    "InterfaceManager",
]


AvailableTemplates = Union[
    I2CInterfaceTemplate,
    SPIInterfaceTemplate,
    UARTInterfaceTemplate,
    GPIOInterfaceTemplate,
]

AvailableDevTypes = Literal["i2c", "spi", "uart", "gpio"]


class BaseInterfaceBuilder:
    """
    Interface builder template
    """

    dev_type: Optional[AvailableDevTypes] = None

    def __init__(self, *args, **kwargs) -> None:
        """
        Initialize the interface builder

        Pass parameters that are globally shared
        """
        raise NotImplementedError()

    def build(self, *args, **kwargs) -> AvailableTemplates:
        """
        Build the interface

        Pass parameters that from module who requests the interface
        """
        raise NotImplementedError()

    @final
    def _internal_build(self, *args, **kwargs) -> AvailableTemplates:
        """
        Internal build function

        This function will be called by the interface manager
        """
        instance = self.build(*args, **kwargs)
        instance._on_destroy.append(self.destroy)
        return instance

    def destroy(self, instance: BaseInterfaceTemplate):
        """
        Destroy the interface
        """
        ...

    @final
    def register(self, module_name: Union[None, str, List[str]] = None):
        """
        Register the interface to the interface manager

        module_name: offer name if this interface is specific to a module
        """
        assert self.dev_type is not None, "Invalid interface builder"
        if module_name is None:
            InterfaceManager.register_global_interface(self.dev_type, self)
        else:
            InterfaceManager.register_specific_interface(
                self.dev_type, module_name, self
            )
        self._module_name = module_name
        return self

    @final
    def unregister(self):
        """
        Unregister the interface from the interface manager
        """
        assert self.dev_type is not None, "Invalid interface builder"
        if self._module_name is None:
            InterfaceManager.unregister_global_interface(self.dev_type)
        else:
            InterfaceManager.unregister_specific_interface(
                self.dev_type, self._module_name
            )
        return self


interface_dict: Dict[AvailableDevTypes, Optional[BaseInterfaceBuilder]] = {
    "i2c": None,
    "spi": None,
    "uart": None,
    "gpio": None,
}

specific_interface_dict: Dict[str, BaseInterfaceBuilder] = {}


class InterfaceManager:
    """
    Interface manager for the whole system
    """

    @staticmethod
    def register_global_interface(
        dev_type: AvailableDevTypes, interface: BaseInterfaceBuilder
    ):
        """
        Register a global interface
        """
        assert dev_type in interface_dict, f"Interface {dev_type} not supported"
        assert (
            interface_dict[dev_type] is None
        ), f"Interface {dev_type} already registered"
        interface_dict[dev_type] = interface
        logger.info(f"Registered a global {dev_type.upper()} interface")

    @staticmethod
    def register_specific_interface(
        dev_type: AvailableDevTypes,
        module_name: Union[str, List[str]],
        interface: BaseInterfaceBuilder,
    ):
        """
        Register a specific interface
        """

        def register(dev_type, module_name, interface):
            module_name = module_name.lower()
            fullname = f"{dev_type}_{module_name}"
            assert (
                fullname not in specific_interface_dict
            ), f"Interface {dev_type} for module {module_name} already registered"
            specific_interface_dict[fullname] = interface
            logger.info(
                f"Registered a specific {dev_type.upper()} interface for {module_name.upper()}"
            )

        if isinstance(module_name, str):
            register(dev_type, module_name, interface)
        elif isinstance(module_name, list):
            for name in module_name:
                register(dev_type, name, interface)
        else:
            raise ValueError("Invalid module name")

    @staticmethod
    def unregister_global_interface(dev_type: AvailableDevTypes):
        """
        Unregister a global interface
        """
        assert dev_type in interface_dict, f"Interface {dev_type} not supported"
        if interface_dict[dev_type] is not None:
            interface_dict[dev_type] = None
            logger.info(f"Global {dev_type.upper()} interface unregistered")

    @staticmethod
    def unregister_specific_interface(
        dev_type: AvailableDevTypes,
        module_name: Union[str, List[str]],
    ):
        """
        Unregister a specific interface
        """

        def unregister(dev_type, module_name):
            module_name = module_name.lower()
            fullname = f"{dev_type}_{module_name}"
            if fullname in specific_interface_dict:
                specific_interface_dict.pop(fullname)
                logger.info(
                    f"Specific {dev_type.upper()} interface for {module_name.upper()} unregistered"
                )

        if isinstance(module_name, str):
            unregister(dev_type, module_name)
        elif isinstance(module_name, list):
            for name in module_name:
                unregister(dev_type, name)

    @staticmethod
    def request_interface(dev_type, module_name, *args, **xargs) -> AvailableTemplates:
        module_name = module_name.lower()
        _fullname = f"{dev_type}_{module_name}"
        if _fullname in specific_interface_dict:
            dev = specific_interface_dict[_fullname]
        else:
            dev = interface_dict.get(dev_type, None)
        assert (
            dev is not None
        ), f"{dev_type.upper()} interface not registered, register it first"
        logger.info(
            f"Module {module_name.upper()} requested a {dev_type.upper()} interface"
        )
        return dev._internal_build(*args, **xargs)
