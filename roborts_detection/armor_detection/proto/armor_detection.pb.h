// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: armor_detection.proto

#ifndef PROTOBUF_armor_5fdetection_2eproto__INCLUDED
#define PROTOBUF_armor_5fdetection_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2006000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace roborts_detection {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_armor_5fdetection_2eproto();
void protobuf_AssignDesc_armor_5fdetection_2eproto();
void protobuf_ShutdownFile_armor_5fdetection_2eproto();

class ArmorDetectionAlgorithms;

// ===================================================================

class ArmorDetectionAlgorithms : public ::google::protobuf::Message {
 public:
  ArmorDetectionAlgorithms();
  virtual ~ArmorDetectionAlgorithms();

  ArmorDetectionAlgorithms(const ArmorDetectionAlgorithms& from);

  inline ArmorDetectionAlgorithms& operator=(const ArmorDetectionAlgorithms& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const ArmorDetectionAlgorithms& default_instance();

  void Swap(ArmorDetectionAlgorithms* other);

  // implements Message ----------------------------------------------

  ArmorDetectionAlgorithms* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const ArmorDetectionAlgorithms& from);
  void MergeFrom(const ArmorDetectionAlgorithms& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated string name = 1;
  inline int name_size() const;
  inline void clear_name();
  static const int kNameFieldNumber = 1;
  inline const ::std::string& name(int index) const;
  inline ::std::string* mutable_name(int index);
  inline void set_name(int index, const ::std::string& value);
  inline void set_name(int index, const char* value);
  inline void set_name(int index, const char* value, size_t size);
  inline ::std::string* add_name();
  inline void add_name(const ::std::string& value);
  inline void add_name(const char* value);
  inline void add_name(const char* value, size_t size);
  inline const ::google::protobuf::RepeatedPtrField< ::std::string>& name() const;
  inline ::google::protobuf::RepeatedPtrField< ::std::string>* mutable_name();

  // optional string selected_algorithm = 2;
  inline bool has_selected_algorithm() const;
  inline void clear_selected_algorithm();
  static const int kSelectedAlgorithmFieldNumber = 2;
  inline const ::std::string& selected_algorithm() const;
  inline void set_selected_algorithm(const ::std::string& value);
  inline void set_selected_algorithm(const char* value);
  inline void set_selected_algorithm(const char* value, size_t size);
  inline ::std::string* mutable_selected_algorithm();
  inline ::std::string* release_selected_algorithm();
  inline void set_allocated_selected_algorithm(::std::string* selected_algorithm);

  // optional uint32 undetected_armor_delay = 3;
  inline bool has_undetected_armor_delay() const;
  inline void clear_undetected_armor_delay();
  static const int kUndetectedArmorDelayFieldNumber = 3;
  inline ::google::protobuf::uint32 undetected_armor_delay() const;
  inline void set_undetected_armor_delay(::google::protobuf::uint32 value);

  // optional string camera_name = 4;
  inline bool has_camera_name() const;
  inline void clear_camera_name();
  static const int kCameraNameFieldNumber = 4;
  inline const ::std::string& camera_name() const;
  inline void set_camera_name(const ::std::string& value);
  inline void set_camera_name(const char* value);
  inline void set_camera_name(const char* value, size_t size);
  inline ::std::string* mutable_camera_name();
  inline ::std::string* release_camera_name();
  inline void set_allocated_camera_name(::std::string* camera_name);

  // @@protoc_insertion_point(class_scope:roborts_detection.ArmorDetectionAlgorithms)
 private:
  inline void set_has_selected_algorithm();
  inline void clear_has_selected_algorithm();
  inline void set_has_undetected_armor_delay();
  inline void clear_has_undetected_armor_delay();
  inline void set_has_camera_name();
  inline void clear_has_camera_name();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::std::string> name_;
  ::std::string* selected_algorithm_;
  ::std::string* camera_name_;
  ::google::protobuf::uint32 undetected_armor_delay_;
  friend void  protobuf_AddDesc_armor_5fdetection_2eproto();
  friend void protobuf_AssignDesc_armor_5fdetection_2eproto();
  friend void protobuf_ShutdownFile_armor_5fdetection_2eproto();

  void InitAsDefaultInstance();
  static ArmorDetectionAlgorithms* default_instance_;
};
// ===================================================================


// ===================================================================

// ArmorDetectionAlgorithms

// repeated string name = 1;
inline int ArmorDetectionAlgorithms::name_size() const {
  return name_.size();
}
inline void ArmorDetectionAlgorithms::clear_name() {
  name_.Clear();
}
inline const ::std::string& ArmorDetectionAlgorithms::name(int index) const {
  // @@protoc_insertion_point(field_get:roborts_detection.ArmorDetectionAlgorithms.name)
  return name_.Get(index);
}
inline ::std::string* ArmorDetectionAlgorithms::mutable_name(int index) {
  // @@protoc_insertion_point(field_mutable:roborts_detection.ArmorDetectionAlgorithms.name)
  return name_.Mutable(index);
}
inline void ArmorDetectionAlgorithms::set_name(int index, const ::std::string& value) {
  // @@protoc_insertion_point(field_set:roborts_detection.ArmorDetectionAlgorithms.name)
  name_.Mutable(index)->assign(value);
}
inline void ArmorDetectionAlgorithms::set_name(int index, const char* value) {
  name_.Mutable(index)->assign(value);
  // @@protoc_insertion_point(field_set_char:roborts_detection.ArmorDetectionAlgorithms.name)
}
inline void ArmorDetectionAlgorithms::set_name(int index, const char* value, size_t size) {
  name_.Mutable(index)->assign(
    reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:roborts_detection.ArmorDetectionAlgorithms.name)
}
inline ::std::string* ArmorDetectionAlgorithms::add_name() {
  return name_.Add();
}
inline void ArmorDetectionAlgorithms::add_name(const ::std::string& value) {
  name_.Add()->assign(value);
  // @@protoc_insertion_point(field_add:roborts_detection.ArmorDetectionAlgorithms.name)
}
inline void ArmorDetectionAlgorithms::add_name(const char* value) {
  name_.Add()->assign(value);
  // @@protoc_insertion_point(field_add_char:roborts_detection.ArmorDetectionAlgorithms.name)
}
inline void ArmorDetectionAlgorithms::add_name(const char* value, size_t size) {
  name_.Add()->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_add_pointer:roborts_detection.ArmorDetectionAlgorithms.name)
}
inline const ::google::protobuf::RepeatedPtrField< ::std::string>&
ArmorDetectionAlgorithms::name() const {
  // @@protoc_insertion_point(field_list:roborts_detection.ArmorDetectionAlgorithms.name)
  return name_;
}
inline ::google::protobuf::RepeatedPtrField< ::std::string>*
ArmorDetectionAlgorithms::mutable_name() {
  // @@protoc_insertion_point(field_mutable_list:roborts_detection.ArmorDetectionAlgorithms.name)
  return &name_;
}

// optional string selected_algorithm = 2;
inline bool ArmorDetectionAlgorithms::has_selected_algorithm() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void ArmorDetectionAlgorithms::set_has_selected_algorithm() {
  _has_bits_[0] |= 0x00000002u;
}
inline void ArmorDetectionAlgorithms::clear_has_selected_algorithm() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void ArmorDetectionAlgorithms::clear_selected_algorithm() {
  if (selected_algorithm_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    selected_algorithm_->clear();
  }
  clear_has_selected_algorithm();
}
inline const ::std::string& ArmorDetectionAlgorithms::selected_algorithm() const {
  // @@protoc_insertion_point(field_get:roborts_detection.ArmorDetectionAlgorithms.selected_algorithm)
  return *selected_algorithm_;
}
inline void ArmorDetectionAlgorithms::set_selected_algorithm(const ::std::string& value) {
  set_has_selected_algorithm();
  if (selected_algorithm_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    selected_algorithm_ = new ::std::string;
  }
  selected_algorithm_->assign(value);
  // @@protoc_insertion_point(field_set:roborts_detection.ArmorDetectionAlgorithms.selected_algorithm)
}
inline void ArmorDetectionAlgorithms::set_selected_algorithm(const char* value) {
  set_has_selected_algorithm();
  if (selected_algorithm_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    selected_algorithm_ = new ::std::string;
  }
  selected_algorithm_->assign(value);
  // @@protoc_insertion_point(field_set_char:roborts_detection.ArmorDetectionAlgorithms.selected_algorithm)
}
inline void ArmorDetectionAlgorithms::set_selected_algorithm(const char* value, size_t size) {
  set_has_selected_algorithm();
  if (selected_algorithm_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    selected_algorithm_ = new ::std::string;
  }
  selected_algorithm_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:roborts_detection.ArmorDetectionAlgorithms.selected_algorithm)
}
inline ::std::string* ArmorDetectionAlgorithms::mutable_selected_algorithm() {
  set_has_selected_algorithm();
  if (selected_algorithm_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    selected_algorithm_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:roborts_detection.ArmorDetectionAlgorithms.selected_algorithm)
  return selected_algorithm_;
}
inline ::std::string* ArmorDetectionAlgorithms::release_selected_algorithm() {
  clear_has_selected_algorithm();
  if (selected_algorithm_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = selected_algorithm_;
    selected_algorithm_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void ArmorDetectionAlgorithms::set_allocated_selected_algorithm(::std::string* selected_algorithm) {
  if (selected_algorithm_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete selected_algorithm_;
  }
  if (selected_algorithm) {
    set_has_selected_algorithm();
    selected_algorithm_ = selected_algorithm;
  } else {
    clear_has_selected_algorithm();
    selected_algorithm_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:roborts_detection.ArmorDetectionAlgorithms.selected_algorithm)
}

// optional uint32 undetected_armor_delay = 3;
inline bool ArmorDetectionAlgorithms::has_undetected_armor_delay() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void ArmorDetectionAlgorithms::set_has_undetected_armor_delay() {
  _has_bits_[0] |= 0x00000004u;
}
inline void ArmorDetectionAlgorithms::clear_has_undetected_armor_delay() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void ArmorDetectionAlgorithms::clear_undetected_armor_delay() {
  undetected_armor_delay_ = 0u;
  clear_has_undetected_armor_delay();
}
inline ::google::protobuf::uint32 ArmorDetectionAlgorithms::undetected_armor_delay() const {
  // @@protoc_insertion_point(field_get:roborts_detection.ArmorDetectionAlgorithms.undetected_armor_delay)
  return undetected_armor_delay_;
}
inline void ArmorDetectionAlgorithms::set_undetected_armor_delay(::google::protobuf::uint32 value) {
  set_has_undetected_armor_delay();
  undetected_armor_delay_ = value;
  // @@protoc_insertion_point(field_set:roborts_detection.ArmorDetectionAlgorithms.undetected_armor_delay)
}

// optional string camera_name = 4;
inline bool ArmorDetectionAlgorithms::has_camera_name() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void ArmorDetectionAlgorithms::set_has_camera_name() {
  _has_bits_[0] |= 0x00000008u;
}
inline void ArmorDetectionAlgorithms::clear_has_camera_name() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void ArmorDetectionAlgorithms::clear_camera_name() {
  if (camera_name_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    camera_name_->clear();
  }
  clear_has_camera_name();
}
inline const ::std::string& ArmorDetectionAlgorithms::camera_name() const {
  // @@protoc_insertion_point(field_get:roborts_detection.ArmorDetectionAlgorithms.camera_name)
  return *camera_name_;
}
inline void ArmorDetectionAlgorithms::set_camera_name(const ::std::string& value) {
  set_has_camera_name();
  if (camera_name_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    camera_name_ = new ::std::string;
  }
  camera_name_->assign(value);
  // @@protoc_insertion_point(field_set:roborts_detection.ArmorDetectionAlgorithms.camera_name)
}
inline void ArmorDetectionAlgorithms::set_camera_name(const char* value) {
  set_has_camera_name();
  if (camera_name_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    camera_name_ = new ::std::string;
  }
  camera_name_->assign(value);
  // @@protoc_insertion_point(field_set_char:roborts_detection.ArmorDetectionAlgorithms.camera_name)
}
inline void ArmorDetectionAlgorithms::set_camera_name(const char* value, size_t size) {
  set_has_camera_name();
  if (camera_name_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    camera_name_ = new ::std::string;
  }
  camera_name_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:roborts_detection.ArmorDetectionAlgorithms.camera_name)
}
inline ::std::string* ArmorDetectionAlgorithms::mutable_camera_name() {
  set_has_camera_name();
  if (camera_name_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    camera_name_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:roborts_detection.ArmorDetectionAlgorithms.camera_name)
  return camera_name_;
}
inline ::std::string* ArmorDetectionAlgorithms::release_camera_name() {
  clear_has_camera_name();
  if (camera_name_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = camera_name_;
    camera_name_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void ArmorDetectionAlgorithms::set_allocated_camera_name(::std::string* camera_name) {
  if (camera_name_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete camera_name_;
  }
  if (camera_name) {
    set_has_camera_name();
    camera_name_ = camera_name;
  } else {
    clear_has_camera_name();
    camera_name_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:roborts_detection.ArmorDetectionAlgorithms.camera_name)
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace roborts_detection

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_armor_5fdetection_2eproto__INCLUDED
