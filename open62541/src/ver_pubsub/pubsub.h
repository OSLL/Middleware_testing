#pragma once
#include <open62541/server.h>
#include <open62541/plugin/pubsub_ethernet.h>
#include <open62541/plugin/pubsub_udp.h>
#include "../DataType.h"

UA_NodeId connectionIdent, publishedDataSetIdent, writerGroupIdent;

void
addPubSubConnection(UA_Server *server, UA_String *transportProfile,
                    UA_NetworkAddressUrlDataType *networkAddressUrl);

void
addPublishedDataSet(UA_Server *server);

void
addDataSetField(UA_Server *server);

void
addDataSetWriter(UA_Server *server);

void
addWriterGroup(UA_Server *server);


