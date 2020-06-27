#ifndef ISOCPP2_TestData_H
#define ISOCPP2_TestData_H

#include "dds/dds.hpp"

#include "TestData.h"
#include "TestDataSplDcps.h"

#include "org/opensplice/topic/TopicTraits.hpp"
#include "org/opensplice/topic/DataRepresentation.hpp"

namespace org { namespace opensplice { namespace topic {
template <>
class TopicTraits<TestDataType>
{
public:
    static ::org::opensplice::topic::DataRepresentationId_t getDataRepresentationId()
    {
        return ::org::opensplice::topic::OSPL_REPRESENTATION;
    }

    static ::std::vector<os_uchar> getMetaData()
    {
        return ::std::vector<os_uchar>();
    }

    static ::std::vector<os_uchar> getTypeHash()
    {
        return ::std::vector<os_uchar>();
    }

    static ::std::vector<os_uchar> getExtentions()
    {
        return ::std::vector<os_uchar>();
    }

    static const char *getKeyList()
    {
        return "id";
    }

    static const char *getTypeName()
    {
        return "TestDataType";
    }

    static std::string getDescriptor()
    {
        const char *elements[] = {
            "<MetaData version=\"1.0.0\"><Struct name=\"TestDataType\"><Member name=\"id\"><Short/></Member><Member name=\"timestamp\">",
"<ULongLong/></Member><Member name=\"data\"><Sequence><Char/></Sequence></Member></Struct></MetaData>"
        };
        std::string descriptor;
        descriptor.reserve(222);
        for (int i = 0; i < 2; i++) {
            descriptor.append(elements[i]);
        }

        return descriptor;
    }

    static copyInFunction getCopyIn()
    {
        return (copyInFunction) __TestDataType__copyIn;
    }

    static copyOutFunction getCopyOut()
    {
        return (copyOutFunction) __TestDataType__copyOut;
    }
};
}}}

namespace dds { namespace topic {
template <>
struct topic_type_name<TestDataType>
{
    static std::string value()
    {
        return org::opensplice::topic::TopicTraits<TestDataType>::getTypeName();
    }
};
}}

REGISTER_TOPIC_TYPE(TestDataType)

#endif /* ISOCPP2_TestData_H */
