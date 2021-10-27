from typing import Any


_I_CLS: Any = None


class _InitMeta(type):
    def __init__(cls, name, bases, cls_dict: dict[str, Any]):
        if len(cls.mro()) > 2:
            objs = [v for v in cls_dict.values() if isinstance(v, _I_CLS)]

            if "init" in cls_dict:
                setattr(cls, "_init", cls.init)
            else:
                setattr(cls, "_init", lambda self: None)

            def init(self, sc_init=None):
                superclass = cls.mro()[1]
                if issubclass(superclass, _I_CLS) and superclass != _I_CLS:
                    superclass.init(self, superclass)
                for obj in objs:
                    obj.init()
                if sc_init is None:
                    self._init()
                else:
                    sc_init._init(self)

            setattr(cls, "init", init)
        super(_InitMeta, cls).__init__(name, bases, cls_dict)


class IInit(metaclass=_InitMeta):
    def init(self): ...


_I_CLS = IInit
