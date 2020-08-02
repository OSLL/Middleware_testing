package org.middleware

import org.apache.rocketmq.client.consumer.DefaultMQPushConsumer
import org.apache.rocketmq.common.consumer.ConsumeFromWhere
import org.apache.rocketmq.common.message.MessageExt
import org.apache.rocketmq.client.consumer.listener.*
import java_interface.SubscriberInterface

class Subscriber(topic: String, msgCount: Int, prior: Int, cpu_index: Int,
                 filename: String, topic_prior: Int):
        SubscriberInterface<MessageExt>(topic, msgCount, prior, cpu_index, filename, topic_prior, MessageExt::class.java){
    private val consumer = DefaultMQPushConsumer("subscribers")
    private var isReceived = false
    val listener = Listener()
    inner class Listener: MessageListenerOrderly{
        override fun consumeMessage(msgs: List<MessageExt>, context: ConsumeOrderlyContext): ConsumeOrderlyStatus {
            for (msg in msgs) {
                this@Subscriber.write_received_msg(msg, 0)
            }
            this@Subscriber.isReceived = true
            return ConsumeOrderlyStatus.SUCCESS
        }
    }

    init {
        consumer.namesrvAddr = "localhost:9876"
        consumer.consumeFromWhere = ConsumeFromWhere.CONSUME_FROM_LAST_OFFSET
        consumer.subscribe(topic, "*")
        consumer.registerMessageListener(listener)
        consumer.start()
    }

    override fun get_id(message: MessageExt?): Int {
        val msg = String(message!!.body)
        return msg.subSequence(0, msg.indexOf("ts:")).toString().toInt()
    }

    override fun get_timestamp(message: MessageExt?): Long {
        val msg = String(message!!.body)
        return msg.subSequence(msg.indexOf("ts:") + "ts:".length, msg.indexOf("data:")).toString().toLong()
    }

    override fun receive(): Boolean {
        isReceived = false
        Thread.sleep(5)
        return isReceived
    }

    fun stopNode(){
        // for (el in _msgs)
        //    print("${String(el.body)} \n")
        consumer.shutdown()
    }
}