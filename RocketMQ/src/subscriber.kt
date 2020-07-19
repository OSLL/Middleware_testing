import com.github.ajalt.clikt.output.TermUi.echo
import org.apache.rocketmq.client.consumer.DefaultMQPushConsumer
import org.apache.rocketmq.client.consumer.listener.ConsumeConcurrentlyStatus
import org.apache.rocketmq.client.consumer.listener.MessageListenerConcurrently
import org.apache.rocketmq.common.consumer.ConsumeFromWhere
import java_interface.SubscriberInterface

import org.apache.rocketmq.common.message.MessageExt

class Subscriber(val topic: String, val msgCount: Int, val prior: Int, val cpu_index: Int,
                 val filename: String, val topic_prior: Int):
        SubscriberInterface<MessageExt>(topic, msgCount, prior, cpu_index, filename, topic_prior){
    val consumer = DefaultMQPushConsumer("subscribers")
    init {

        consumer.setNamesrvAddr("localhost:9876")
        consumer.consumeFromWhere = ConsumeFromWhere.CONSUME_FROM_FIRST_OFFSET
        consumer.subscribe("TopicTest", "*")
        consumer.registerMessageListener(MessageListenerConcurrently { msgs, context ->
            echo ("Received New Messages: $msgs")
            ConsumeConcurrentlyStatus.CONSUME_SUCCESS
        })
        consumer.start()
        echo("Consumer Started.")
    }

    override fun get_id(msg: MessageExt?): Int {
        val msg = msg!!.body.toString()
        val id = msg.subSequence(0, msg.indexOf("ts:")).toString().toInt()
        return id
    }

    override fun get_timestamp(msg: MessageExt?): Long {
        val msg = msg!!.body.toString()
        val timestamp = msg.subSequence(msg.indexOf("ts:") + 3, msg.indexOf("data:")).toString().toLong()
        return timestamp
    }

    override fun receive(): Boolean {
        TODO("Not yet implemented")
    }
}