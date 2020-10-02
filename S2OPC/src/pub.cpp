extern "C"{
#include <assert.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

//#include"sopc_pubsub_helpers.h"
#include "sopc_pub_scheduler.h"

#include "sopc_helper_uri.h"
#include "sopc_pubsub_helpers.h"
#include "sopc_time.h"
#include "sopc_filesystem.h"
#include "sopc_xml_loader.h"
#include "string.h"
#include "errno.h"

#include "config.h"
#include "pubsub.h"
#include "server.h"
}

#include "pub.h"

const char* pub_config_xml = "<PubSub publisherId=\"41\">\n\
    <!-- one to many -->\n\
    <connection address=\"opc.udp://232.1.2.100:4841/%s\" mode=\"publisher\" >\n\
        <!-- one to many -->\n\
        <message id=\"14\" version=\"1\" publishingInterval=\"%d\">\n\
          <!-- one to many -->\n\
            <variable nodeId=\"ns=1;s=PubBool\" displayName=\"varBool\" dataType=\"Boolean\"/>\n\
            <variable nodeId=\"ns=1;s=PubString\" displayName=\"varString\" dataType=\"String\"/>\n\
            <!-- the nodeId is used to retrieve the variable in the adresse space -->\n\
        </message>\n\
        <message id=\"15\" version=\"1\" publishingInterval=\"%d\">\n\
            <!-- one to many -->\n\
            <variable nodeId=\"ns=1;s=PubInt\" displayName=\"varInt\" dataType=\"Int64\"/>\n\
            <variable nodeId=\"ns=1;s=PubUInt\" displayName=\"varUInt\" dataType=\"UInt64\"/>\n\
        </message>\n\
    </connection>\n\
</PubSub>\0";

SOPC_S2OPC_Config s2opcConfig;

extern "C" void configure_server(std::string& topic, long int delay_ms){
    SOPC_ReturnStatus status = SOPC_STATUS_OK;
    /* Initialize S2OPC Server */
    status = Server_Initialize();
    if (SOPC_STATUS_OK != status)
    {
        printf("# Error: Could not initialize the server.\n");
    }

    /* Configure the Server */
    SOPC_S2OPC_Config_Initialize(&s2opcConfig);

    if (SOPC_STATUS_OK == status)
    {
        status = Server_CreateServerConfig(&s2opcConfig);
        if (SOPC_STATUS_OK != status)
        {
            printf("# Error: Could not create the server configuration.\n");
        }
    }

    if (SOPC_STATUS_OK == status)
    {
        status = Server_LoadAddressSpace();
    }

    /* Configuration of the PubSub module is done upon PubSub start through the local service */

    /* Start the Server */
    s2opcConfig.serverConfig.endpoints[0].endpointURL = "opc.udp://232.1.2.100:2200";
    if (SOPC_STATUS_OK == status)
    {
        status = Server_ConfigureStartServer(&s2opcConfig.serverConfig.endpoints[0]);
    }

    /* Write in PubSub nodes, which starts the PubSub */
    //if (SOPC_STATUS_OK == status)
    //{
    //    status = Server_WritePubSubNodes();
    //}


    //Load Config
    /*SOPC_Array* configBuffers = Server_GetConfigurationPaths();
    if (NULL == configBuffers || SOPC_Array_Size(configBuffers) != 1)
    {
        printf("# Error: Multiple configuration paths.\n");
        SOPC_Array_Delete(configBuffers);
        return 2;
    }

    char* configBuffer = SOPC_Array_Get(configBuffers, char*, 0);*/
    char* configBuffer = (char*)malloc((strlen(pub_config_xml)+topic.length()+10)*sizeof(char));
    if(NULL == configBuffer){
        printf("# Error: Can't alloc memory\n");
        return;
    }
    sprintf(configBuffer, pub_config_xml, topic.c_str(), delay_ms, delay_ms);
    //printf("config: %s\n", configBuffer);
    FILE* fd = SOPC_FileSystem_fmemopen((void*) configBuffer, strlen(configBuffer), "r");
    SOPC_PubSubConfiguration* pPubSubConfig = NULL;

    if (NULL == fd)
    {
        printf("# Error: Cannot open \"%s\": %s.\n", configBuffer, strerror(errno));
        status = SOPC_STATUS_NOK;
    }
    if (SOPC_STATUS_OK == status)
    {
        pPubSubConfig = SOPC_PubSubConfig_ParseXML(fd);
        if (NULL == pPubSubConfig)
        {
            printf("# Error: Cannot parse PubSub configuration file \"%s\".\n", configBuffer);
            status = SOPC_STATUS_NOK;
        }
        else
        {
            printf("# Info: PubSub XML configuration loaded.\n");
        }
    }
    if (NULL != fd)
    {
        fclose(fd);
        fd = NULL;
    }
    free(configBuffer);

    SOPC_PubSourceVariableConfig* pSourceConfig = NULL;
    if (SOPC_STATUS_OK == status)
    {
        pSourceConfig = SOPC_PubSourceVariableConfig_Create(Server_GetSourceVariables);
        if (NULL == pSourceConfig)
        {
            printf("# Error: Cannot create Pub configuration.\n");
            status = SOPC_STATUS_NOK;
        }
    }

    uint32_t pub_nb_connections = SOPC_PubSubConfiguration_Nb_PubConnection(pPubSubConfig);

    bool pubOK = 0;
    if (pub_nb_connections > 0){
        pubOK = SOPC_PubScheduler_Start(pPubSubConfig, pSourceConfig);
    }
    if(!pubOK){
        printf("Error in pub\n");
        return;
    }

}

extern "C" void stop_server(){
    PubSub_StopAndClear();
    Server_StopAndClear(&s2opcConfig);
}
