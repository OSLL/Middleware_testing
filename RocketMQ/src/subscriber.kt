import org.apache.rocketmq.client.consumer.DefaultMQPushConsumer
import org.apache.rocketmq.client.consumer.listener.ConsumeConcurrentlyStatus
import org.apache.rocketmq.client.consumer.listener.MessageListenerConcurrently
import org.apache.rocketmq.client.exception.MQClientException
import org.apache.rocketmq.common.consumer.ConsumeFromWhere

class subscriber {
    fun main() {

        val consumer = DefaultMQPushConsumer("please_rename_unique_group_name_4")
        consumer.setNamesrvAddr("localhost:9876");
        /*
             * Specify name server addresses.
             * <p/>
             *
             * Alternatively, you may specify name server addresses via exporting environmental variable: NAMESRV_ADDR
             * <pre>
             * {@code
             * consumer.setNamesrvAddr("name-server1-ip:9876;name-server2-ip:9876");
             * }
             * </pre>
             */

        /*
             * Specify where to start in case the specified consumer group is a brand new one.
             */consumer.consumeFromWhere = ConsumeFromWhere.CONSUME_FROM_FIRST_OFFSET

        /*
             * Subscribe one more more topics to consume.
             */consumer.subscribe("TopicTest", "*")

        /*
             *  Register callback to execute on arrival of messages fetched from brokers.
             */consumer.registerMessageListener(MessageListenerConcurrently { msgs, context ->
            System.out.printf("%s Receive New Messages: %s %n", Thread.currentThread().name, msgs)
            ConsumeConcurrentlyStatus.CONSUME_SUCCESS
        })

        /*
             *  Launch the consumer instance.
             */consumer.start()
        System.out.printf("Consumer Started.%n")
    }
}