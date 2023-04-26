// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: polygon.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_polygon_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_polygon_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3019000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3019004 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_polygon_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_polygon_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_polygon_2eproto;
class pg;
struct pgDefaultTypeInternal;
extern pgDefaultTypeInternal _pg_default_instance_;
PROTOBUF_NAMESPACE_OPEN
template<> ::pg* Arena::CreateMaybeMessage<::pg>(Arena*);
PROTOBUF_NAMESPACE_CLOSE

// ===================================================================

class pg final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pg) */ {
 public:
  inline pg() : pg(nullptr) {}
  ~pg() override;
  explicit constexpr pg(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  pg(const pg& from);
  pg(pg&& from) noexcept
    : pg() {
    *this = ::std::move(from);
  }

  inline pg& operator=(const pg& from) {
    CopyFrom(from);
    return *this;
  }
  inline pg& operator=(pg&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const pg& default_instance() {
    return *internal_default_instance();
  }
  static inline const pg* internal_default_instance() {
    return reinterpret_cast<const pg*>(
               &_pg_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(pg& a, pg& b) {
    a.Swap(&b);
  }
  inline void Swap(pg* other) {
    if (other == this) return;
  #ifdef PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() != nullptr &&
        GetOwningArena() == other->GetOwningArena()) {
   #else  // PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() == other->GetOwningArena()) {
  #endif  // !PROTOBUF_FORCE_COPY_IN_SWAP
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(pg* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  pg* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<pg>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const pg& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const pg& from);
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to, const ::PROTOBUF_NAMESPACE_ID::Message& from);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  uint8_t* _InternalSerialize(
      uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(pg* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pg";
  }
  protected:
  explicit pg(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kXFieldNumber = 1,
    kYFieldNumber = 2,
    kZFieldNumber = 3,
    kRFieldNumber = 4,
    kGFieldNumber = 5,
    kBFieldNumber = 6,
  };
  // repeated float x = 1;
  int x_size() const;
  private:
  int _internal_x_size() const;
  public:
  void clear_x();
  private:
  float _internal_x(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      _internal_x() const;
  void _internal_add_x(float value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      _internal_mutable_x();
  public:
  float x(int index) const;
  void set_x(int index, float value);
  void add_x(float value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      x() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      mutable_x();

  // repeated float y = 2;
  int y_size() const;
  private:
  int _internal_y_size() const;
  public:
  void clear_y();
  private:
  float _internal_y(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      _internal_y() const;
  void _internal_add_y(float value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      _internal_mutable_y();
  public:
  float y(int index) const;
  void set_y(int index, float value);
  void add_y(float value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      y() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      mutable_y();

  // repeated float z = 3;
  int z_size() const;
  private:
  int _internal_z_size() const;
  public:
  void clear_z();
  private:
  float _internal_z(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      _internal_z() const;
  void _internal_add_z(float value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      _internal_mutable_z();
  public:
  float z(int index) const;
  void set_z(int index, float value);
  void add_z(float value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      z() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      mutable_z();

  // repeated float r = 4;
  int r_size() const;
  private:
  int _internal_r_size() const;
  public:
  void clear_r();
  private:
  float _internal_r(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      _internal_r() const;
  void _internal_add_r(float value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      _internal_mutable_r();
  public:
  float r(int index) const;
  void set_r(int index, float value);
  void add_r(float value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      r() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      mutable_r();

  // repeated float g = 5;
  int g_size() const;
  private:
  int _internal_g_size() const;
  public:
  void clear_g();
  private:
  float _internal_g(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      _internal_g() const;
  void _internal_add_g(float value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      _internal_mutable_g();
  public:
  float g(int index) const;
  void set_g(int index, float value);
  void add_g(float value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      g() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      mutable_g();

  // repeated float b = 6;
  int b_size() const;
  private:
  int _internal_b_size() const;
  public:
  void clear_b();
  private:
  float _internal_b(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      _internal_b() const;
  void _internal_add_b(float value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      _internal_mutable_b();
  public:
  float b(int index) const;
  void set_b(int index, float value);
  void add_b(float value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      b() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      mutable_b();

  // @@protoc_insertion_point(class_scope:pg)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float > x_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float > y_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float > z_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float > r_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float > g_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float > b_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_polygon_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// pg

// repeated float x = 1;
inline int pg::_internal_x_size() const {
  return x_.size();
}
inline int pg::x_size() const {
  return _internal_x_size();
}
inline void pg::clear_x() {
  x_.Clear();
}
inline float pg::_internal_x(int index) const {
  return x_.Get(index);
}
inline float pg::x(int index) const {
  // @@protoc_insertion_point(field_get:pg.x)
  return _internal_x(index);
}
inline void pg::set_x(int index, float value) {
  x_.Set(index, value);
  // @@protoc_insertion_point(field_set:pg.x)
}
inline void pg::_internal_add_x(float value) {
  x_.Add(value);
}
inline void pg::add_x(float value) {
  _internal_add_x(value);
  // @@protoc_insertion_point(field_add:pg.x)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
pg::_internal_x() const {
  return x_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
pg::x() const {
  // @@protoc_insertion_point(field_list:pg.x)
  return _internal_x();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
pg::_internal_mutable_x() {
  return &x_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
pg::mutable_x() {
  // @@protoc_insertion_point(field_mutable_list:pg.x)
  return _internal_mutable_x();
}

// repeated float y = 2;
inline int pg::_internal_y_size() const {
  return y_.size();
}
inline int pg::y_size() const {
  return _internal_y_size();
}
inline void pg::clear_y() {
  y_.Clear();
}
inline float pg::_internal_y(int index) const {
  return y_.Get(index);
}
inline float pg::y(int index) const {
  // @@protoc_insertion_point(field_get:pg.y)
  return _internal_y(index);
}
inline void pg::set_y(int index, float value) {
  y_.Set(index, value);
  // @@protoc_insertion_point(field_set:pg.y)
}
inline void pg::_internal_add_y(float value) {
  y_.Add(value);
}
inline void pg::add_y(float value) {
  _internal_add_y(value);
  // @@protoc_insertion_point(field_add:pg.y)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
pg::_internal_y() const {
  return y_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
pg::y() const {
  // @@protoc_insertion_point(field_list:pg.y)
  return _internal_y();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
pg::_internal_mutable_y() {
  return &y_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
pg::mutable_y() {
  // @@protoc_insertion_point(field_mutable_list:pg.y)
  return _internal_mutable_y();
}

// repeated float z = 3;
inline int pg::_internal_z_size() const {
  return z_.size();
}
inline int pg::z_size() const {
  return _internal_z_size();
}
inline void pg::clear_z() {
  z_.Clear();
}
inline float pg::_internal_z(int index) const {
  return z_.Get(index);
}
inline float pg::z(int index) const {
  // @@protoc_insertion_point(field_get:pg.z)
  return _internal_z(index);
}
inline void pg::set_z(int index, float value) {
  z_.Set(index, value);
  // @@protoc_insertion_point(field_set:pg.z)
}
inline void pg::_internal_add_z(float value) {
  z_.Add(value);
}
inline void pg::add_z(float value) {
  _internal_add_z(value);
  // @@protoc_insertion_point(field_add:pg.z)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
pg::_internal_z() const {
  return z_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
pg::z() const {
  // @@protoc_insertion_point(field_list:pg.z)
  return _internal_z();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
pg::_internal_mutable_z() {
  return &z_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
pg::mutable_z() {
  // @@protoc_insertion_point(field_mutable_list:pg.z)
  return _internal_mutable_z();
}

// repeated float r = 4;
inline int pg::_internal_r_size() const {
  return r_.size();
}
inline int pg::r_size() const {
  return _internal_r_size();
}
inline void pg::clear_r() {
  r_.Clear();
}
inline float pg::_internal_r(int index) const {
  return r_.Get(index);
}
inline float pg::r(int index) const {
  // @@protoc_insertion_point(field_get:pg.r)
  return _internal_r(index);
}
inline void pg::set_r(int index, float value) {
  r_.Set(index, value);
  // @@protoc_insertion_point(field_set:pg.r)
}
inline void pg::_internal_add_r(float value) {
  r_.Add(value);
}
inline void pg::add_r(float value) {
  _internal_add_r(value);
  // @@protoc_insertion_point(field_add:pg.r)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
pg::_internal_r() const {
  return r_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
pg::r() const {
  // @@protoc_insertion_point(field_list:pg.r)
  return _internal_r();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
pg::_internal_mutable_r() {
  return &r_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
pg::mutable_r() {
  // @@protoc_insertion_point(field_mutable_list:pg.r)
  return _internal_mutable_r();
}

// repeated float g = 5;
inline int pg::_internal_g_size() const {
  return g_.size();
}
inline int pg::g_size() const {
  return _internal_g_size();
}
inline void pg::clear_g() {
  g_.Clear();
}
inline float pg::_internal_g(int index) const {
  return g_.Get(index);
}
inline float pg::g(int index) const {
  // @@protoc_insertion_point(field_get:pg.g)
  return _internal_g(index);
}
inline void pg::set_g(int index, float value) {
  g_.Set(index, value);
  // @@protoc_insertion_point(field_set:pg.g)
}
inline void pg::_internal_add_g(float value) {
  g_.Add(value);
}
inline void pg::add_g(float value) {
  _internal_add_g(value);
  // @@protoc_insertion_point(field_add:pg.g)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
pg::_internal_g() const {
  return g_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
pg::g() const {
  // @@protoc_insertion_point(field_list:pg.g)
  return _internal_g();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
pg::_internal_mutable_g() {
  return &g_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
pg::mutable_g() {
  // @@protoc_insertion_point(field_mutable_list:pg.g)
  return _internal_mutable_g();
}

// repeated float b = 6;
inline int pg::_internal_b_size() const {
  return b_.size();
}
inline int pg::b_size() const {
  return _internal_b_size();
}
inline void pg::clear_b() {
  b_.Clear();
}
inline float pg::_internal_b(int index) const {
  return b_.Get(index);
}
inline float pg::b(int index) const {
  // @@protoc_insertion_point(field_get:pg.b)
  return _internal_b(index);
}
inline void pg::set_b(int index, float value) {
  b_.Set(index, value);
  // @@protoc_insertion_point(field_set:pg.b)
}
inline void pg::_internal_add_b(float value) {
  b_.Add(value);
}
inline void pg::add_b(float value) {
  _internal_add_b(value);
  // @@protoc_insertion_point(field_add:pg.b)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
pg::_internal_b() const {
  return b_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
pg::b() const {
  // @@protoc_insertion_point(field_list:pg.b)
  return _internal_b();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
pg::_internal_mutable_b() {
  return &b_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
pg::mutable_b() {
  // @@protoc_insertion_point(field_mutable_list:pg.b)
  return _internal_mutable_b();
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)


// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_polygon_2eproto