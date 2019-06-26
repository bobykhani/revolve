// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: parameter.proto

#ifndef PROTOBUF_parameter_2eproto__INCLUDED
#define PROTOBUF_parameter_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3005000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3005000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace protobuf_parameter_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[1];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
void InitDefaultsParameterImpl();
void InitDefaultsParameter();
inline void InitDefaults() {
  InitDefaultsParameter();
}
}  // namespace protobuf_parameter_2eproto
namespace revolve {
namespace msgs {
class Parameter;
class ParameterDefaultTypeInternal;
extern ParameterDefaultTypeInternal _Parameter_default_instance_;
}  // namespace msgs
}  // namespace revolve
namespace revolve {
namespace msgs {

// ===================================================================

class Parameter : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:revolve.msgs.Parameter) */ {
 public:
  Parameter();
  virtual ~Parameter();

  Parameter(const Parameter& from);

  inline Parameter& operator=(const Parameter& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Parameter(Parameter&& from) noexcept
    : Parameter() {
    *this = ::std::move(from);
  }

  inline Parameter& operator=(Parameter&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Parameter& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Parameter* internal_default_instance() {
    return reinterpret_cast<const Parameter*>(
               &_Parameter_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(Parameter* other);
  friend void swap(Parameter& a, Parameter& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Parameter* New() const PROTOBUF_FINAL { return New(NULL); }

  Parameter* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const Parameter& from);
  void MergeFrom(const Parameter& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(Parameter* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required double value = 1;
  bool has_value() const;
  void clear_value();
  static const int kValueFieldNumber = 1;
  double value() const;
  void set_value(double value);

  // @@protoc_insertion_point(class_scope:revolve.msgs.Parameter)
 private:
  void set_has_value();
  void clear_has_value();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable int _cached_size_;
  double value_;
  friend struct ::protobuf_parameter_2eproto::TableStruct;
  friend void ::protobuf_parameter_2eproto::InitDefaultsParameterImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Parameter

// required double value = 1;
inline bool Parameter::has_value() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Parameter::set_has_value() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Parameter::clear_has_value() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Parameter::clear_value() {
  value_ = 0;
  clear_has_value();
}
inline double Parameter::value() const {
  // @@protoc_insertion_point(field_get:revolve.msgs.Parameter.value)
  return value_;
}
inline void Parameter::set_value(double value) {
  set_has_value();
  value_ = value;
  // @@protoc_insertion_point(field_set:revolve.msgs.Parameter.value)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace revolve

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_parameter_2eproto__INCLUDED
