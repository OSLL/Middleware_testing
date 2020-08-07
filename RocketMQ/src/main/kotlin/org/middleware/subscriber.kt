package org.middleware

import java_interface.SubscriberInterface
import org.apache.rocketmq.client.consumer.DefaultMQPushConsumer
import org.apache.rocketmq.common.consumer.ConsumeFromWhere
import org.apache.rocketmq.common.message.MessageExt
import org.apache.rocketmq.client.consumer.listener.*
import java.util.concurrent.atomic.AtomicBoolean

class Subscriber(topic: String, msgCount: Int, prior: Int, cpu_index: Int,
                 filename: String, topic_prior: Int):
        SubscriberInterface<MessageExt>(topic, msgCount, prior, cpu_index, filename, topic_prior, MessageExt::class.java){
    private val consumer = DefaultMQPushConsumer("subscribers")
    protected var isReceived: AtomicBoolean = AtomicBoolean(false)
    val listener = Listener()
    inner class Listener: MessageListenerOrderly{
        @Synchronized override fun consumeMessage(msgs: List<MessageExt>, context: ConsumeOrderlyContext): ConsumeOrderlyStatus {
            this@Subscriber.isReceived.set(true)
            for (msg in msgs) {
                if (msg.reconsumeTimes < 1) {
                    print("${String(msg.body)} ${msg.topic} \n")
                    this@Subscriber.write_received_msg(msg, 0)
                }
            }
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
        val msg = message?.body?.let { String(it) }
        return msg?.subSequence(0, msg.indexOf("ts:")).toString().toInt()
    }

    override fun get_timestamp(message: MessageExt?): Long {
        val msg = message?.body?.let { String(it) }
        return msg?.subSequence(msg.indexOf("ts:") + "ts:".length, msg.indexOf("data:")).toString().toLong()
    }

    override fun receive(): Boolean {
        return isReceived.compareAndSet(true, false)
    }

    fun stopNode(){
        // for (el in _msgs)
        //    print("${String(el.body)} \n")
        consumer.shutdown()
    }
}