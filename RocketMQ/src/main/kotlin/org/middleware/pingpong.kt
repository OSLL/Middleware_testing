package org.middleware

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
import java_interface.PingPongInterface


class PingPong(topic1: String, topic2: String, msgCount: Int, prior: Int, cpu_index: Int,
               filename: String, topic_priority: Int, msInterval: Int, msgSize: Int, isFirst: Boolean):
        PingPongInterface<MessageExt>(topic1, topic2, msgCount, prior, cpu_index,
                filename, topic_priority, msInterval, msgSize, isFirst, MessageExt::class.java){

    val producer = DefaultMQProducer("publishers")
    private val consumer = DefaultMQPushConsumer("subscribers")
    protected var isReceived = false
    val listener = Listener()
    inner class Listener: MessageListenerOrderly {
        override fun consumeMessage(msgs: List<MessageExt>, context: ConsumeOrderlyContext): ConsumeOrderlyStatus {
            for (msg in msgs) {
                // print(String(msg.body))
                this@PingPong.write_received_msg(msg)
            }
            this@PingPong.isReceived = true
            return ConsumeOrderlyStatus.SUCCESS
        }
    }
    init {
        producer.namesrvAddr = "localhost:9876"
        producer.start()
        producer.retryTimesWhenSendAsyncFailed = 0
        consumer.namesrvAddr = "localhost:9876"
        consumer.consumeFromWhere = ConsumeFromWhere.CONSUME_FROM_LAST_OFFSET
        consumer.subscribe(_topic_name1, "*")
        consumer.registerMessageListener(listener)
        consumer.start()
    }

    override fun publish(id: Int, size: Int): Unit {
        val data = "a".padEnd(size, 'a')
        try {
            var curTime = System.nanoTime()
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
        val msg = String(message!!.body)
        return msg.subSequence(msg.indexOf("ts:") + "ts:".length, msg.indexOf("data:")).toString().toLong()
    }

    override fun receive(): Boolean {
        isReceived = false
        Thread.sleep(5)
        return isReceived
    }

    fun stopNode(){
        producer.shutdown()
        consumer.shutdown()
    }
}
