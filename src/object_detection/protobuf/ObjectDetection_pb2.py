# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ObjectDetection.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import builder as _builder
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x15ObjectDetection.proto\x12\netrs.proto\"6\n\x0fObjectDetection\x12#\n\x07objects\x18\x01 \x03(\x0b\x32\x12.etrs.proto.Object\"M\n\x06Object\x12\r\n\x05label\x18\x01 \x01(\x05\x12%\n\x04\x62\x62ox\x18\x02 \x01(\x0b\x32\x17.etrs.proto.BoundingBox\x12\r\n\x05score\x18\x03 \x01(\x02\"j\n\x0b\x42oundingBox\x12\t\n\x01x\x18\x01 \x01(\x02\x12\t\n\x01y\x18\x02 \x01(\x02\x12\t\n\x01z\x18\x03 \x01(\x02\x12\r\n\x05width\x18\x04 \x01(\x02\x12\x0e\n\x06height\x18\x05 \x01(\x02\x12\x0e\n\x06length\x18\x06 \x01(\x02\x12\x0b\n\x03yaw\x18\x07 \x01(\x02\x62\x06proto3')

_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, globals())
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'ObjectDetection_pb2', globals())
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _OBJECTDETECTION._serialized_start=37
  _OBJECTDETECTION._serialized_end=91
  _OBJECT._serialized_start=93
  _OBJECT._serialized_end=170
  _BOUNDINGBOX._serialized_start=172
  _BOUNDINGBOX._serialized_end=278
# @@protoc_insertion_point(module_scope)