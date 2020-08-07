package org.middleware

import java_interface.PingPongInterface
import org.apache.rocketmq.client.consumer.DefaultMQPushConsumer
import org.apache.rocketmq.client.producer.DefaultMQProducer
import org.apache.rocketmq.client.producer.SendCallback
import org.apache.rocketmq.client.producer.SendResult
import org.apache.rocketmq.common.consumer.ConsumeFromWhere
import org.apache.rocketmq.common.message.Message
import org.apache.rocketmq.common.message.MessageExt
import org.apache.rocketmq.remoting.common.RemotingHelper
import org.apache.rocketmq.client.consumer.listener.ConsumeOrderlyContext
import org.apache.rocketmq.client.consumer.listener.ConsumeOrderlyStatus
import org.apache.rocketmq.client.consumer.listener.MessageListenerOrderly
import java.util.concurrent.atomic.AtomicBoolean


class PingPong(topic1: String, topic2: String, msgCount: Int, prior: Int, cpu_index: Int,
               filename: String, topic_priority: Int, msInterval: Int, msgSize: Int, isFirst: Boolean):
        PingPongInterface<MessageExt>(topic1, topic2, msgCount, prior, cpu_index,
                filename, topic_priority, msInterval, msgSize, isFirst, MessageExt::class.java){

    val producer = DefaultMQProducer("publishers")
    private val consumer = DefaultMQPushConsumer("subscribers")
    protected var isReceived: AtomicBoolean = AtomicBoolean(false)
    val listener = Listener()
    inner class Listener: MessageListenerOrderly {
        @Synchronized override fun consumeMessage(msgs: List<MessageExt>, context: ConsumeOrderlyContext): ConsumeOrderlyStatus {
            this@PingPong.isReceived.set(true)
            for (msg in msgs) {
                if (msg.reconsumeTimes < 1) {
                    print("${String(msg.body)} ${msg.topic} \n")
                    this@PingPong.write_received_msg(msg)
                }
            }
            return ConsumeOrderlyStatus.SUCCESS
        }
    }
    init {
        producer.namesrvAddr = "localhost:9876"
        producer.start()
        producer.retryTimesWhenSendAsyncFailed = 0
        consumer.namesrvAddr = "localhost:9876"
        consumer.consumeFromWhere = ConsumeFromWhere.CONSUME_FROM_LAST_OFFSET
        consumer.subscribe(_topic_name2, "*")
        consumer.registerMessageListener(listener)
        consumer.start()
    }

    override fun publish(id: Int, size: Int): Unit {
        val data = "a".padEnd(size, 'a')
        try {
            var curTime = System.currentTimeMillis() * TIME_SCALE + System.nanoTime()
            print("Publishing to $_topic_name1 \n")
            val msg = Message(_topic_name1,
                    "TagA",
                    "OrderID1",
                    "${id}ts:${curTime}data:$data".toByteArray(charset(RemotingHelper.DEFAULT_CHARSET)))
            producer.send(msg, object : SendCallback {
                override fun onSuccess(sendResult: SendResult) {
                    // print ("$id OK")
                }

                override fun onException(e: Throwable) {
                    //print ("$id Exception!")
                    e.printStackTrace()
                }
            })
        } catch (e: Exception) {
            e.printStackTrace()
        }
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
        //if (isReceived.get())
        //    print("true\n")
        return isReceived.compareAndSet(true, false)
    }

    fun stopNode(){
        producer.shutdown()
        consumer.shutdown()
    }
}
