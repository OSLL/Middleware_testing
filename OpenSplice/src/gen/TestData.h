#ifndef _TestData_H_
#define _TestData_H_

#include <dds/core/ddscore.hpp>

class TestDataType OSPL_DDS_FINAL
{
public:
    typedef std::vector<char>  _data_seq;

    int16_t id_;
    uint64_t timestamp_;
    std::vector<char>  data_;

public:
    TestDataType() :
            id_(0),
            timestamp_(0) {}

    explicit TestDataType(
        int16_t id,
        uint64_t timestamp,
        const std::vector<char> & data) : 
            id_(id),
            timestamp_(timestamp),
            data_(data) {}

    TestDataType(const TestDataType &_other) : 
            id_(_other.id_),
            timestamp_(_other.timestamp_),
            data_(_other.data_) {}

#ifdef OSPL_DDS_CXX11
    TestDataType(TestDataType &&_other) : 
            id_(::std::move(_other.id_)),
            timestamp_(::std::move(_other.timestamp_)),
            data_(::std::move(_other.data_)) {}

    TestDataType& operator=(TestDataType &&_other)
    {
        if (this != &_other) {
            id_ = ::std::move(_other.id_);
            timestamp_ = ::std::move(_other.timestamp_);
            data_ = ::std::move(_other.data_);
        }
        return *this;
    }
#endif

    TestDataType& operator=(const TestDataType &_other)
    {
        if (this != &_other) {
            id_ = _other.id_;
            timestamp_ = _other.timestamp_;
            data_ = _other.data_;
        }
        return *this;
    }

    bool operator==(const TestDataType& _other) const
    {
        return id_ == _other.id_ &&
            timestamp_ == _other.timestamp_ &&
            data_ == _other.data_;
    }

    bool operator!=(const TestDataType& _other) const
    {
        return !(*this == _other);
    }

    int16_t id() const { return this->id_; }
    int16_t& id() { return this->id_; }
    void id(int16_t _val_) { this->id_ = _val_; }
    uint64_t timestamp() const { return this->timestamp_; }
    uint64_t& timestamp() { return this->timestamp_; }
    void timestamp(uint64_t _val_) { this->timestamp_ = _val_; }
    const std::vector<char> & data() const { return this->data_; }
    std::vector<char> & data() { return this->data_; }
    void data(const std::vector<char> & _val_) { this->data_ = _val_; }
#ifdef OSPL_DDS_CXX11
    void data(std::vector<char> && _val_) { this->data_ = _val_; }
#endif
};

#endif /* _TestData_H_ */
