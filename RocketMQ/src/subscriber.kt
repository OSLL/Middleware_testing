package rocketmq_test

import com.github.ajalt.clikt.output.TermUi.echo
import org.apache.rocketmq.client.consumer.DefaultMQPushConsumer
import org.apache.rocketmq.client.consumer.listener.ConsumeConcurrentlyStatus
import org.apache.rocketmq.client.consumer.listener.MessageListenerConcurrently
import org.apache.rocketmq.common.consumer.ConsumeFromWhere
import java_interface.SubscriberInterface
import org.apache.rocketmq.client.consumer.listener.ConsumeConcurrentlyContext

import org.apache.rocketmq.common.message.MessageExt
import java.util.*

class SetFlagTimer(private val listener: Listener): TimerTask(){
    override fun run() {
        echo("timer")
        listener.isReceived = false
    }
}

class Listener: MessageListenerConcurrently{
    var isReceived = false
    private val timerTask = SetFlagTimer(this)
    private val timer = Timer(false)
    override fun consumeMessage(msgs: MutableList<MessageExt>?, context: ConsumeConcurrentlyContext): ConsumeConcurrentlyStatus {
        return if (msgs == null || msgs.isEmpty()) {
            timer.schedule(timerTask, 1)
            echo("Not rec")
            ConsumeConcurrentlyStatus.RECONSUME_LATER
        }
        else {
            echo("Received New Messages: $msgs")
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
        consumer.subscribe("TopicTest", "*")
        consumer.registerMessageListener(listener)
        consumer.start()
        echo("Consumer Started.")
    }

    override fun get_id(message: MessageExt?): Int {
        val msg = message!!.body.toString()
        return msg.subSequence(0, msg.indexOf("ts:")).toString().toInt()
    }

    override fun get_timestamp(message: MessageExt?): Long {
        val msg = message!!.body.toString()
        return msg.subSequence(msg.indexOf("ts:") + 3, msg.indexOf("data:")).toString().toLong()
    }

    override fun receive(): Boolean {
        return listener.isReceived
    }

    fun stopNode(){
        consumer.shutdown()
    }
}