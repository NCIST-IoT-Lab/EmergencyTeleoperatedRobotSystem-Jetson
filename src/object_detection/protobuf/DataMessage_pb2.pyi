"""
@generated by mypy-protobuf.  Do not edit manually!
isort:skip_file
"""
import BotArm_pb2
import BotCar_pb2
import BotGripper_pb2
import BotLed_pb2
import BotMotor_pb2
import KinectMode_pb2
import Mesh_pb2
import ObjectDetection_pb2
import PointCloud_pb2
import SoundSource_pb2
import TempAndHumi_pb2
import builtins
import google.protobuf.descriptor
import google.protobuf.internal.enum_type_wrapper
import google.protobuf.message
import sys
import typing

if sys.version_info >= (3, 10):
    import typing as typing_extensions
else:
    import typing_extensions

DESCRIPTOR: google.protobuf.descriptor.FileDescriptor

@typing_extensions.final
class DataMessage(google.protobuf.message.Message):
    DESCRIPTOR: google.protobuf.descriptor.Descriptor

    class _Type:
        ValueType = typing.NewType("ValueType", builtins.int)
        V: typing_extensions.TypeAlias = ValueType

    class _TypeEnumTypeWrapper(google.protobuf.internal.enum_type_wrapper._EnumTypeWrapper[DataMessage._Type.ValueType], builtins.type):
        DESCRIPTOR: google.protobuf.descriptor.EnumDescriptor
        OTHER: DataMessage._Type.ValueType  # 0
        MESH: DataMessage._Type.ValueType  # 1
        EXIT_MESH: DataMessage._Type.ValueType  # -1
        SOUND_SOURCE: DataMessage._Type.ValueType  # 2
        BOT_CAR: DataMessage._Type.ValueType  # 3
        BOT_GRIPPER: DataMessage._Type.ValueType  # 4
        BOT_MOTOR: DataMessage._Type.ValueType  # 5
        BOT_ARM: DataMessage._Type.ValueType  # 6
        BOT_LED: DataMessage._Type.ValueType  # 7
        TEMP_AND_HUMI: DataMessage._Type.ValueType  # 8
        KINECT_MODE: DataMessage._Type.ValueType  # 9
        OBJECT_DETECTION: DataMessage._Type.ValueType  # 10
        POINT_CLOUD: DataMessage._Type.ValueType  # 11

    class Type(_Type, metaclass=_TypeEnumTypeWrapper): ...
    OTHER: DataMessage.Type.ValueType  # 0
    MESH: DataMessage.Type.ValueType  # 1
    EXIT_MESH: DataMessage.Type.ValueType  # -1
    SOUND_SOURCE: DataMessage.Type.ValueType  # 2
    BOT_CAR: DataMessage.Type.ValueType  # 3
    BOT_GRIPPER: DataMessage.Type.ValueType  # 4
    BOT_MOTOR: DataMessage.Type.ValueType  # 5
    BOT_ARM: DataMessage.Type.ValueType  # 6
    BOT_LED: DataMessage.Type.ValueType  # 7
    TEMP_AND_HUMI: DataMessage.Type.ValueType  # 8
    KINECT_MODE: DataMessage.Type.ValueType  # 9
    OBJECT_DETECTION: DataMessage.Type.ValueType  # 10
    POINT_CLOUD: DataMessage.Type.ValueType  # 11

    TYPE_FIELD_NUMBER: builtins.int
    MESH_FIELD_NUMBER: builtins.int
    SOUND_SOURCE_FIELD_NUMBER: builtins.int
    BOT_CAR_FIELD_NUMBER: builtins.int
    BOT_GRIPPER_FIELD_NUMBER: builtins.int
    BOT_MOTOR_FIELD_NUMBER: builtins.int
    BOT_ARM_FIELD_NUMBER: builtins.int
    BOT_LED_FIELD_NUMBER: builtins.int
    TEMP_AND_HUMI_FIELD_NUMBER: builtins.int
    KINECT_MODE_FIELD_NUMBER: builtins.int
    OBJECT_DETECTION_FIELD_NUMBER: builtins.int
    POINT_CLOUD_FIELD_NUMBER: builtins.int
    type: global___DataMessage.Type.ValueType
    @property
    def mesh(self) -> Mesh_pb2.Mesh: ...
    @property
    def sound_source(self) -> SoundSource_pb2.SoundSource: ...
    @property
    def bot_car(self) -> BotCar_pb2.BotCar: ...
    @property
    def bot_gripper(self) -> BotGripper_pb2.BotGripper: ...
    @property
    def bot_motor(self) -> BotMotor_pb2.BotMotor: ...
    @property
    def bot_arm(self) -> BotArm_pb2.BotArm: ...
    @property
    def bot_led(self) -> BotLed_pb2.BotLed: ...
    @property
    def temp_and_humi(self) -> TempAndHumi_pb2.TempAndHumi: ...
    @property
    def kinect_mode(self) -> KinectMode_pb2.KinectMode: ...
    @property
    def object_detection(self) -> ObjectDetection_pb2.ObjectDetection: ...
    @property
    def point_cloud(self) -> PointCloud_pb2.PointCloud: ...
    def __init__(
        self,
        *,
        type: global___DataMessage.Type.ValueType = ...,
        mesh: Mesh_pb2.Mesh | None = ...,
        sound_source: SoundSource_pb2.SoundSource | None = ...,
        bot_car: BotCar_pb2.BotCar | None = ...,
        bot_gripper: BotGripper_pb2.BotGripper | None = ...,
        bot_motor: BotMotor_pb2.BotMotor | None = ...,
        bot_arm: BotArm_pb2.BotArm | None = ...,
        bot_led: BotLed_pb2.BotLed | None = ...,
        temp_and_humi: TempAndHumi_pb2.TempAndHumi | None = ...,
        kinect_mode: KinectMode_pb2.KinectMode | None = ...,
        object_detection: ObjectDetection_pb2.ObjectDetection | None = ...,
        point_cloud: PointCloud_pb2.PointCloud | None = ...,
    ) -> None: ...
    def HasField(self, field_name: typing_extensions.Literal["bot_arm", b"bot_arm", "bot_car", b"bot_car", "bot_gripper", b"bot_gripper", "bot_led", b"bot_led", "bot_motor", b"bot_motor", "data", b"data", "kinect_mode", b"kinect_mode", "mesh", b"mesh", "object_detection", b"object_detection", "point_cloud", b"point_cloud", "sound_source", b"sound_source", "temp_and_humi", b"temp_and_humi"]) -> builtins.bool: ...
    def ClearField(self, field_name: typing_extensions.Literal["bot_arm", b"bot_arm", "bot_car", b"bot_car", "bot_gripper", b"bot_gripper", "bot_led", b"bot_led", "bot_motor", b"bot_motor", "data", b"data", "kinect_mode", b"kinect_mode", "mesh", b"mesh", "object_detection", b"object_detection", "point_cloud", b"point_cloud", "sound_source", b"sound_source", "temp_and_humi", b"temp_and_humi", "type", b"type"]) -> None: ...
    def WhichOneof(self, oneof_group: typing_extensions.Literal["data", b"data"]) -> typing_extensions.Literal["mesh", "sound_source", "bot_car", "bot_gripper", "bot_motor", "bot_arm", "bot_led", "temp_and_humi", "kinect_mode", "object_detection", "point_cloud"] | None: ...

global___DataMessage = DataMessage
