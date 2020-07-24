package org.middleware

import com.github.ajalt.clikt.output.TermUi.echo
import org.apache.rocketmq.client.consumer.DefaultMQPushConsumer
import org.apache.rocketmq.client.consumer.listener.ConsumeConcurrentlyStatus
import org.apache.rocketmq.client.consumer.listener.MessageListenerConcurrently
import org.apache.rocketmq.common.consumer.ConsumeFromWhere
import java_interface.SubscriberInterface
import org.apache.rocketmq.client.consumer.listener.ConsumeConcurrentlyContext

import org.apache.rocketmq.common.message.MessageExt


class Listener: MessageListenerConcurrently{
    var isReceived = false
    override fun consumeMessage(msgs: MutableList<MessageExt>?, context: ConsumeConcurrentlyContext): ConsumeConcurrentlyStatus {
        return if (msgs == null || msgs.isEmpty()) {
            ConsumeConcurrentlyStatus.RECONSUME_LATER
        }
        else {
            //echo("Received New Messages: ${String(msgs[0].body)}")
            isReceived = true
            echo(this.isReceived)
            ConsumeConcurrentlyStatus.CONSUME_SUCCESS
        }
    }
}

class Subscriber(topic: String, msgCount: Int, prior: Int, cpu_index: Int,
                 filename: String, topic_prior: Int):
        SubscriberInterface<MessageExt>(topic, msgCount, prior, cpu_index, filename, topic_prior){
    private val consumer = DefaultMQPushConsumer("subscribers")
    private val listener = Listener()
    init {
        consumer.namesrvAddr = "localhost:9876"
        consumer.consumeFromWhere = ConsumeFromWhere.CONSUME_FROM_FIRST_OFFSET
        consumer.subscribe("topic1", "*")
        consumer.registerMessageListener(listener)
        consumer.start()
        echo("Consumer Started.")
    }

    override fun get_id(message: MessageExt?): Int {
        val msg = String(message!!.body)
        return msg.subSequence(0, msg.indexOf("ts:")).toString().toInt()
    }

    override fun get_timestamp(message: MessageExt?): Long {
        val msg = String(message!!.body)
        return msg.subSequence(msg.indexOf("ts:") + 3, msg.indexOf("data:")).toString().toLong()
    }

    override fun receive(): Boolean {
        listener.isReceived = false
        Thread.sleep(5)
        return listener.isReceived
    }

    fun stopNode(){
        consumer.shutdown()
    }
}