// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: parameter.proto

#include "parameter.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)
namespace revolve {
namespace msgs {
class ParameterDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<Parameter>
      _instance;
} _Parameter_default_instance_;
}  // namespace msgs
}  // namespace revolve
namespace protobuf_parameter_2eproto {
void InitDefaultsParameterImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  ::google::protobuf::internal::InitProtobufDefaultsForceUnique();
#else
  ::google::protobuf::internal::InitProtobufDefaults();
#endif  // GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  {
    void* ptr = &::revolve::msgs::_Parameter_default_instance_;
    new (ptr) ::revolve::msgs::Parameter();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::revolve::msgs::Parameter::InitAsDefaultInstance();
}

void InitDefaultsParameter() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &InitDefaultsParameterImpl);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::revolve::msgs::Parameter, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::revolve::msgs::Parameter, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::revolve::msgs::Parameter, value_),
  0,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 6, sizeof(::revolve::msgs::Parameter)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::revolve::msgs::_Parameter_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "parameter.proto", schemas, file_default_instances, TableStruct::offsets, factory,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\017parameter.proto\022\014revolve.msgs\"\032\n\tParam"
      "eter\022\r\n\005value\030\001 \002(\001"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 59);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "parameter.proto", &protobuf_RegisterTypes);
}

void AddDescriptors() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_parameter_2eproto
namespace revolve {
namespace msgs {

// ===================================================================

void Parameter::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Parameter::kValueFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Parameter::Parameter()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    ::protobuf_parameter_2eproto::InitDefaultsParameter();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:revolve.msgs.Parameter)
}
Parameter::Parameter(const Parameter& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  value_ = from.value_;
  // @@protoc_insertion_point(copy_constructor:revolve.msgs.Parameter)
}

void Parameter::SharedCtor() {
  _cached_size_ = 0;
  value_ = 0;
}

Parameter::~Parameter() {
  // @@protoc_insertion_point(destructor:revolve.msgs.Parameter)
  SharedDtor();
}

void Parameter::SharedDtor() {
}

void Parameter::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Parameter::descriptor() {
  ::protobuf_parameter_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_parameter_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const Parameter& Parameter::default_instance() {
  ::protobuf_parameter_2eproto::InitDefaultsParameter();
  return *internal_default_instance();
}

Parameter* Parameter::New(::google::protobuf::Arena* arena) const {
  Parameter* n = new Parameter;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void Parameter::Clear() {
// @@protoc_insertion_point(message_clear_start:revolve.msgs.Parameter)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  value_ = 0;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool Parameter::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:revolve.msgs.Parameter)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required double value = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(9u /* 9 & 0xFF */)) {
          set_has_value();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &value_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:revolve.msgs.Parameter)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:revolve.msgs.Parameter)
  return false;
#undef DO_
}

void Parameter::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:revolve.msgs.Parameter)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required double value = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->value(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:revolve.msgs.Parameter)
}

::google::protobuf::uint8* Parameter::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:revolve.msgs.Parameter)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required double value = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->value(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:revolve.msgs.Parameter)
  return target;
}

size_t Parameter::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:revolve.msgs.Parameter)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  // required double value = 1;
  if (has_value()) {
    total_size += 1 + 8;
  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Parameter::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:revolve.msgs.Parameter)
  GOOGLE_DCHECK_NE(&from, this);
  const Parameter* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const Parameter>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:revolve.msgs.Parameter)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:revolve.msgs.Parameter)
    MergeFrom(*source);
  }
}

void Parameter::MergeFrom(const Parameter& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:revolve.msgs.Parameter)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.has_value()) {
    set_value(from.value());
  }
}

void Parameter::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:revolve.msgs.Parameter)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Parameter::CopyFrom(const Parameter& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:revolve.msgs.Parameter)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Parameter::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000001) != 0x00000001) return false;
  return true;
}

void Parameter::Swap(Parameter* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Parameter::InternalSwap(Parameter* other) {
  using std::swap;
  swap(value_, other->value_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata Parameter::GetMetadata() const {
  protobuf_parameter_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_parameter_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace msgs
}  // namespace revolve

// @@protoc_insertion_point(global_scope)
