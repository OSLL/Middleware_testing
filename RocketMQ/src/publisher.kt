import org.apache.rocketmq.client.exception.MQClientException
import org.apache.rocketmq.client.producer.DefaultMQProducer
import org.apache.rocketmq.client.producer.SendCallback
import org.apache.rocketmq.client.producer.SendResult
import org.apache.rocketmq.common.message.Message
import org.apache.rocketmq.remoting.common.RemotingHelper
import java.io.UnsupportedEncodingException
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit

fun main(
        args: Array<String>) {
    val producer = DefaultMQProducer("Jodie_Daily_test")
    producer.setNamesrvAddr("localhost:9876");
    producer.start()
    producer.retryTimesWhenSendAsyncFailed = 0
    val messageCount = 100
    val countDownLatch = CountDownLatch(messageCount)
    for (i in 0 until messageCount) {
        try {
            val msg = Message("TopicTest",
                    "TagA",
                    "OrderID188",
                    "Hello world".toByteArray(charset(RemotingHelper.DEFAULT_CHARSET)))
            producer.send(msg, object : SendCallback {
                override fun onSuccess(sendResult: SendResult) {
                    countDownLatch.countDown()
                    System.out.printf("%-10d OK %s %n", i, sendResult.msgId)
                }

                override fun onException(e: Throwable) {
                    countDownLatch.countDown()
                    System.out.printf("%-10d Exception %s %n", i, e)
                    e.printStackTrace()
                }
            })
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }
    countDownLatch.await(5, TimeUnit.SECONDS)
    producer.shutdown()
}