#define data_type_binary_encoding_id 1

typedef struct {
    UA_String data;
    UA_Int16 id;
    UA_UInt64 timestamp;
} TestData;

#define id_padding offsetof(TestData, id) - offsetof(TestData, data) - sizeof(UA_String)
#define timestamp_padding offsetof(TestData, timestamp) - offsetof(TestData, id) - sizeof(UA_Int16)

static UA_DataTypeMember DataType_members[3] = {
        {
                UA_TYPENAME("data") /* .memberName */
                UA_TYPES_STRING,  /* .memberTypeIndex, points into UA_TYPES since namespaceZero is true */
                0,               /* .padding */
                true,            /* .namespaceZero, see .memberTypeIndex */
                false           /* .isArray */
        },
        {
                UA_TYPENAME("id") /* .memberName */
                UA_TYPES_INT16,  /* .memberTypeIndex, points into UA_TYPES since namespaceZero is true */
                id_padding,               /* .padding */
                true,            /* .namespaceZero, see .memberTypeIndex */
                false           /* .isArray */
        },
        {
                UA_TYPENAME("timestamp") /* .memberName */
                UA_TYPES_UINT64,  /* .memberTypeIndex, points into UA_TYPES since namespaceZero is true */
                timestamp_padding,                /* .padding */
                true,            /* .namespaceZero, see .memberTypeIndex */
                false           /* .isArray */
        }
};

static const UA_DataType DataType = {
        UA_TYPENAME("TestData")                /* .tyspeName */
        {1, UA_NODEIDTYPE_NUMERIC, {4242}}, /* .typeId */
        sizeof(TestData),                      /* .memSize */
        0,                                  /* .typeIndex, in the array of custom types */
        UA_DATATYPEKIND_STRUCTURE,          /* .typeKind */
        false,                               /* .pointerFree */
        false,                              /* .overlayable (depends on endianness and
                                            the absence of padding) */
        3,                                 /* .membersSize */
        data_type_binary_encoding_id,           /* .binaryEncodingId, the numeric
                                            identifier used on the wire (the
                                            namespaceindex is from .typeId) */
        DataType_members
};
