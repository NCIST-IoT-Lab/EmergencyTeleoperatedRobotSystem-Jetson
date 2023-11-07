"""
@generated by mypy-protobuf.  Do not edit manually!
isort:skip_file
"""
import builtins
import google.protobuf.descriptor
import google.protobuf.message
import sys

if sys.version_info >= (3, 8):
    import typing as typing_extensions
else:
    import typing_extensions

DESCRIPTOR: google.protobuf.descriptor.FileDescriptor

@typing_extensions.final
class TempAndHumi(google.protobuf.message.Message):
    DESCRIPTOR: google.protobuf.descriptor.Descriptor

    TEMP_FIELD_NUMBER: builtins.int
    HUMI_FIELD_NUMBER: builtins.int
    temp: builtins.float
    humi: builtins.float
    def __init__(
        self,
        *,
        temp: builtins.float = ...,
        humi: builtins.float = ...,
    ) -> None: ...
    def ClearField(self, field_name: typing_extensions.Literal["humi", b"humi", "temp", b"temp"]) -> None: ...

global___TempAndHumi = TempAndHumi