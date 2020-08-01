package org.middleware

import com.github.ajalt.clikt.core.CliktCommand
import com.github.ajalt.clikt.parameters.options.flag
import com.github.ajalt.clikt.parameters.options.option
import com.github.ajalt.clikt.parameters.options.required
import com.github.ajalt.clikt.parameters.types.choice
import com.github.ajalt.clikt.parameters.types.file
import org.json.simple.JSONArray
import org.json.simple.JSONObject
import org.json.simple.parser.JSONParser
import java.io.FileReader
import kotlin.system.exitProcess


class RocketmqNode: CliktCommand() {
    private val configFilename by option("-c", "--config", help = "Path to test configuration file").file(mustExist = true).required()
    private val type: String by option("-t", "--type", help = "").choice("publisher", "subscriber", "ping_pong").required()
    private val isFirst: Boolean by option("--first").flag()
    override fun run() {
        val parser = JSONParser()
        try {
            val jsonObj = parser.parse(FileReader(configFilename)) as JSONObject
            val topic1 = (jsonObj["topic"] as JSONArray)[0] as String
            val topic2 = (jsonObj["topic"] as JSONArray)[1] as String
            val pubFilename = (jsonObj["res_filenames"] as JSONArray)[0] as String
            val subFilename = (jsonObj["res_filenames"] as JSONArray)[1] as String
            val msgCount = (jsonObj["m_count"] as Long).toInt()
            val pubPriority = ((jsonObj["priority"] as JSONArray)[0] as Long).toInt()
            val subPriority = ((jsonObj["priority"] as JSONArray)[1] as Long).toInt()
            val pubCpuIndex = ((jsonObj["cpu_index"] as JSONArray)[0] as Long).toInt()
            val subCpuIndex = ((jsonObj["cpu_index"] as JSONArray)[1] as Long).toInt()
            val minMsgSize = (jsonObj["min_msg_size"] as Long).toInt()
            val maxMsgSize = (jsonObj["max_msg_size"] as Long).toInt()
            val step = (jsonObj["step"] as Long).toInt()
            val msgsBeforeStep = (jsonObj["msgs_before_step"] as Long).toInt()
            val interval = (jsonObj["interval"] as Long).toInt()
            val topicPrior = (jsonObj["topic_priority"] as Long).toInt()
            when (type){
                "publisher" -> {
                    val publisher = Publisher(topic1, msgCount, pubPriority, pubCpuIndex, minMsgSize, maxMsgSize, step,
                            interval, msgsBeforeStep, pubFilename, topicPrior)
                    echo("pub")
                    publisher.startTest()
                    echo("test ends")
                    publisher.stopNode()
                }
                "subscriber" -> {
                    echo("sub")
                    val subscriber = Subscriber(topic1, msgCount, subPriority, subCpuIndex, subFilename, topicPrior)
                    subscriber.startTest()
                    echo("test ends")
                    subscriber.stopNode()
                }
                "ping_pong" -> {
                    TODO("Implement ping_pong node class")
                }
            }

        }
        catch (e: Exception){
            echo(e.message)
            e.printStackTrace()
            exitProcess(1)
        }
        echo("$type $configFilename $isFirst")

        exitProcess(0)
    }
}

fun main(args: Array<String>) = RocketmqNode().main(args)