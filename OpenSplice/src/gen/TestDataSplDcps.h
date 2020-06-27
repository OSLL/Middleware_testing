#ifndef TestDataSPLDCPS_H
#define TestDataSPLDCPS_H

#include <c_base.h>
#include <c_misc.h>
#include <c_sync.h>
#include <c_collection.h>
#include <c_field.h>
#include <v_copyIn.h>

#include "TestData.h"


extern const char *TestDataType_metaDescriptor[];
extern const c_ulong TestDataType_metaDescriptorArrLength;
extern const c_ulong TestDataType_metaDescriptorLength;
extern c_metaObject __TestDataType__load (c_base base);
struct _TestDataType ;
extern  v_copyin_result __TestDataType__copyIn(c_type dbType, const class TestDataType *from, struct _TestDataType *to);
extern  void __TestDataType__copyOut(const void *_from, void *_to);
struct _TestDataType {
    c_short id;
    c_ulonglong timestamp;
    c_sequence data;
};

#undef OS_API
#endif
