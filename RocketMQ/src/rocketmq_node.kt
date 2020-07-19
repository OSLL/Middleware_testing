package rocketmq_test

import com.github.ajalt.clikt.core.CliktCommand
import com.github.ajalt.clikt.parameters.options.flag
import com.github.ajalt.clikt.parameters.options.option
import com.github.ajalt.clikt.parameters.options.required
import com.github.ajalt.clikt.parameters.types.choice
import com.github.ajalt.clikt.parameters.types.file

class RocketmqNode: CliktCommand() {
    val configFilename by option("-c", "--config", help = "Path to test configuration file").file(mustExist = true).required()
    val type: String by option("-t", "--type", help = "").choice("publisher", "subscriber", "ping_pong").required()
    val isFirst: Boolean by option("--first").flag()
    override fun run() {
        echo("$type $configFilename $isFirst")
    }
}

fun main(args: Array<String>) = RocketmqNode().main(args)