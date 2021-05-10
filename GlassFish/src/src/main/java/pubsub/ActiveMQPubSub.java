package pubsub;

import java_interface.*;

import java.util.concurrent.TimeUnit;
import java.util.Scanner;
import java.util.Arrays;
import java.io.FileReader;
import java.io.IOException;

//import org.apache.activemq.artemis.jms.client.ActiveMQTopicConnectionFactory;
//import com.sun.messaging.ConnectionFactory;
import org.apache.commons.cli.*;

import javax.jms.TopicConnectionFactory;
import javax.jms.TopicSubscriber;
import javax.jms.TopicPublisher;
import javax.jms.MapMessage;
import javax.jms.TopicConnection;
import javax.jms.Topic;
import javax.jms.TopicSession;
import javax.jms.DeliveryMode;
import javax.jms.JMSException;

import org.json.simple.JSONObject;
import org.json.simple.JSONArray;
import org.json.simple.parser.JSONParser;

//import javax.naming.*;
//import javax.jms.ConnectionFactory;
import com.sun.messaging.ConnectionFactory;
import com.sun.messaging.ConnectionConfiguration;
//import com.sun.messaging.jmq.jmsserver.Globals;

class Message{
    public int id;
    public long timestamp;
    
    public Message(int id, long timestamp){
        this.id = id;
        this.timestamp = timestamp;
    }
}

class ActiveMQPublisher extends PublisherInterface{

    //ActiveMQTopicConnectionFactory cf;
    ConnectionFactory cf;
    TopicConnection con;
    TopicSession session;
    Topic topic;
    TopicPublisher pub;
    private long TIME_SCALE = 1_000_000;

    public ActiveMQPublisher(String address, String topic_name, int msgCount, int prior, 
            int cpu_index, int min_msg_size, int max_msg_size, int step, int interval,
            int msgs_before_step, String filename,  int topic_prior) throws Exception {
        super(topic_name, msgCount, prior, cpu_index, min_msg_size, max_msg_size, step,
                interval, msgs_before_step, filename, topic_prior);
        //cf = new ActiveMQTopicConnectionFactory(address);
        cf = new ConnectionFactory();
        cf.setProperty(ConnectionConfiguration.imqAddressList, address);
        con = cf.createTopicConnection();
        session = con.createTopicSession(false, TopicSession.AUTO_ACKNOWLEDGE);
        topic = session.createTopic(topic_name);
        pub = session.createPublisher(topic);
        pub.setDeliveryMode(DeliveryMode.NON_PERSISTENT);
        con.start();
    }

    public long publish(int id, int size) {
        try{
            byte[] bytes = new byte[size];
            Arrays.fill(bytes, (byte)65);
            //System.out.println("Send Message: " + new String(bytes));
            MapMessage msg = session.createMapMessage();
            long time = System.currentTimeMillis() * TIME_SCALE + System.nanoTime();
            msg.setInt("id",id);
            msg.setLong("time", time);
            msg.setBytes("bytes", bytes);
            pub.send(msg);
            time = System.currentTimeMillis() * TIME_SCALE + System.nanoTime() - time;
            return time;
        } catch (JMSException e){
            e.printStackTrace();
            return 0;
        }
    }

    public void close() throws JMSException {
        if(pub != null)
            pub.close();
        if(session != null)
            session.close();
        if(con != null){
            con.close();
        }
    }
}

class ActiveMQSubscriber extends SubscriberInterface<Message>{

    //ActiveMQTopicConnectionFactory cf;
    ConnectionFactory cf;
    TopicConnection con;
    TopicSession session;
    Topic topic;
    TopicSubscriber sub;

    public ActiveMQSubscriber(String address, String topic_name, int msgCount, int prior,
            int cpu_index, String filename, int topic_prior) throws Exception {
        super(topic_name, msgCount, prior, cpu_index, filename, topic_prior, Message.class);
        //cf = new ActiveMQTopicConnectionFactory(address);
        cf = new ConnectionFactory();
        cf.setProperty(ConnectionConfiguration.imqAddressList, address);
        con = cf.createTopicConnection();
        session = con.createTopicSession(false, TopicSession.AUTO_ACKNOWLEDGE);
        topic = session.createTopic(topic_name);
        sub = session.createSubscriber(topic);
        con.start();
    }

    public boolean receive(){
        try{
            long time = System.currentTimeMillis() * TIME_SCALE + System.nanoTime();
            MapMessage msg = (MapMessage) sub.receiveNoWait();
            if(msg == null)
                return false;
            Message m = new Message(msg.getInt("id"), msg.getLong("time"));
            //System.out.println("Receive message:");
            //System.out.println("\tTime: " + msg.getLong("time"));
            //System.out.println("\tId: " + msg.getInt("id"));
            //System.out.println("\tMessage: " + new String(msg.getBytes("bytes")));
            String s = new String(msg.getBytes("bytes"));
            time = System.currentTimeMillis() * TIME_SCALE + System.nanoTime() - time;
            write_received_msg(m, time);
            return true;
        }catch(JMSException e){
            e.printStackTrace();
            return false;
        }
    }

    public int get_id(Message msg){
        if(msg == null) return 0;
        return msg.id;
    }

    public long get_timestamp(Message msg){
        if(msg == null) return 0;
        return msg.timestamp;
    }
        
    public void close() throws JMSException {
        if(sub != null)
            sub.close();
        if(session != null)
            session.close();
        if(con != null){
            con.close();
        }
    }
}

class ActiveMQPingPong extends PingPongInterface<Message>{

    //ActiveMQTopicConnectionFactory cf;
    ConnectionFactory cf;
    TopicConnection con;
    TopicSession session;
    Topic topic1, topic2;
    TopicPublisher pub;
    TopicSubscriber sub;
    private long TIME_SCALE = 1_000_000;

    ActiveMQPingPong(String address, String topic1_name, String topic2_name, int msgCount,
            int prior, int cpu_index, String filename, int topic_prior, int interval,
            int msg_size, boolean isFirst) throws Exception{
        super(topic1_name, topic2_name, msgCount, prior, cpu_index, filename,
                topic_prior, interval, msg_size, isFirst, Message.class);
        //cf = new ActiveMQTopicConnectionFactory(address);
        cf = new ConnectionFactory();
        cf.setProperty(ConnectionConfiguration.imqAddressList, address);
        con = cf.createTopicConnection();
        session = con.createTopicSession(false, TopicSession.AUTO_ACKNOWLEDGE);
        topic1 = session.createTopic(topic1_name);
        topic2 = session.createTopic(topic2_name);
        if(_isFirst) pub = session.createPublisher(topic1);
        else pub = session.createPublisher(topic2);
        pub.setDeliveryMode(DeliveryMode.NON_PERSISTENT);
        if(_isFirst) sub = session.createSubscriber(topic2);
        else sub = session.createSubscriber(topic1);
        con.start();
    }

    ActiveMQPingPong(String address, String topic1_name, String topic2_name, int msgCount,
            int prior, int cpu_index, String filename, int topic_prior, int interval,
            int msg_size_min, int msg_size_max, int step, int before_step, boolean isFirst) throws Exception{
        super(topic1_name, topic2_name, msgCount, prior, cpu_index, filename,
                topic_prior, interval, msg_size_min, msg_size_max, step, before_step, isFirst, Message.class);
        //cf = new ActiveMQTopicConnectionFactory(address);
        cf = new ConnectionFactory();
        cf.setProperty(ConnectionConfiguration.imqAddressList, address);
        con = cf.createTopicConnection();
        session = con.createTopicSession(false, TopicSession.AUTO_ACKNOWLEDGE);
        topic1 = session.createTopic(topic1_name);
        topic2 = session.createTopic(topic2_name);
        if(_isFirst) pub = session.createPublisher(topic1);
        else pub = session.createPublisher(topic2);
        pub.setDeliveryMode(DeliveryMode.NON_PERSISTENT);
        if(_isFirst) sub = session.createSubscriber(topic2);
        else sub = session.createSubscriber(topic1);
        con.start();
    }
    
    public void publish(int id, int size) {
        try{
            long time = System.currentTimeMillis() * TIME_SCALE + System.nanoTime();
            byte[] bytes = new byte[size];
            Arrays.fill(bytes, (byte)65);
            System.out.println("Send Message: " + id);
            MapMessage msg = session.createMapMessage();
            //long time = System.currentTimeMillis() * TIME_SCALE + System.nanoTime();
            msg.setInt("id",id);
            msg.setLong("time", time);
            msg.setBytes("bytes", bytes);
            pub.send(msg);
            time = System.currentTimeMillis() * TIME_SCALE + System.nanoTime();
            _write_msg_time[id] = time;
        } catch (JMSException e){
            e.printStackTrace();
        }
    }
    
    public boolean receive(){
        try{
            MapMessage msg = (MapMessage) sub.receiveNoWait();
            if(msg == null)
                return false;
            Message m = new Message(msg.getInt("id"), msg.getLong("time"));
            System.out.println("Receive message:");
            System.out.println("\tTime: " + msg.getLong("time"));
            System.out.println("\tId: " + msg.getInt("id"));
            System.out.println("\tMessage: " + new String(msg.getBytes("bytes")));
            String s = new String(msg.getBytes("bytes"));
            write_received_msg(m);
            return true;
        }catch(JMSException e){
            e.printStackTrace();
            return false;
        }
    }

    public int get_id(Message msg){
        if(msg == null) return 0;
        return msg.id;
    }

    public long get_timestamp(Message msg){
        if(msg == null) return 0;
        return msg.timestamp;
    }

    public void close() throws JMSException {
        if(pub != null)
            pub.close();
        if(sub != null)
            sub.close();
        if(session != null)
            session.close();
        if(con != null)
            con.close();
    }
}

public class ActiveMQPubSub{
    
    public static void main(String[] args) throws Exception {

        //for build version
        Options options = new Options();
        
        Option opt_type = new Option("t","type", true, " node type: publisher, subscriber, ping_pong");
        opt_type.setRequired(true);
        options.addOption(opt_type);
        
        Option opt_config = new Option("c","config", true, " config file for test");
        opt_config.setRequired(true);
        options.addOption(opt_config);
        
        Option opt_first = new Option("", "first", false, " parameter first for ping_pong test");
        opt_first.setRequired(false);
        options.addOption(opt_first);
        
        Option opt_address = new Option("a","address", true, " server address");
        opt_address.setRequired(true);
        options.addOption(opt_address);

        CommandLineParser parser = new DefaultParser();
        HelpFormatter formatter = new HelpFormatter();
        CommandLine cmd;

        String address = new String();
        String type = new String();
        String config = new String();
        boolean isFirst = false;
        try{
            cmd = parser.parse(options, args);
            address = cmd.getOptionValue("a");
            type = cmd.getOptionValue("t");
            config = cmd.getOptionValue("c");
            isFirst = cmd.hasOption("first");
        } catch (Exception e){
            System.out.println(e.getMessage());
            formatter.printHelp("utility-name", options);

            System.exit(1);
        }


        JSONObject json = new JSONObject();
        try(FileReader fr = new FileReader(config)){
            JSONParser jparser = new JSONParser();
            json = (JSONObject) jparser.parse(fr);
        } catch (Exception e){
            e.printStackTrace();
            System.exit(2);
        }

        String topic1 =(String) ((JSONArray) json.get("topic")).get(0);
        String topic2 =(String) ((JSONArray) json.get("topic")).get(1);
        String filename1 =(String) ((JSONArray) json.get("res_filenames")).get(0);
        String filename2 =(String) ((JSONArray) json.get("res_filenames")).get(1);
        int m_count = ((Long) json.get("m_count")).intValue();
        int prior1 = ((Long) ((JSONArray) json.get("priority")).get(0)).intValue();
        int prior2 = ((Long) ((JSONArray) json.get("priority")).get(1)).intValue();
        int cpu1 = ((Long) ((JSONArray) json.get("cpu_index")).get(0)).intValue();
        int cpu2 = ((Long) ((JSONArray) json.get("cpu_index")).get(1)).intValue();
        int min_size = ((Long) json.get("min_msg_size")).intValue();
        int max_size = ((Long) json.get("max_msg_size")).intValue();
        int step = ((Long) json.get("step")).intValue();
        int before_step = ((Long) json.get("msgs_before_step")).intValue();
        int interval = ((Long) json.get("interval")).intValue();
        int topic_prior = ((Long) json.get("topic_priority")).intValue();

        if(type.equals("publisher")){
            try{
                ActiveMQPublisher pub = new ActiveMQPublisher(address, topic1, m_count, prior1, cpu1,
                        min_size, max_size, step, interval, before_step, filename1, topic_prior);
                pub.startTest();
                pub.close();
            }catch(JMSException e){
                e.printStackTrace();
                System.exit(2);
            }
        } else if(type.equals("subscriber")){
            try{
                ActiveMQSubscriber sub = new ActiveMQSubscriber(address, topic1, m_count, prior2, cpu2,
                        filename2, topic_prior);
                sub.startTest();
                sub.close();
            }catch(JMSException e){
                e.printStackTrace();
                System.exit(2);
            }
        } else if(type.equals("ping_pong")){
            if(isFirst){
                try{
                    ActiveMQPingPong ping_pong = (interval == 0? 
                        new ActiveMQPingPong(address, topic1, topic2, m_count,
                                prior1, cpu1, filename1, topic_prior, interval, min_size, isFirst) :
                        new ActiveMQPingPong(address, topic1, topic2, m_count,
                                prior1, cpu1, filename1, topic_prior, interval, min_size, max_size, step, before_step, isFirst));
                    ping_pong.startTest();
                    ping_pong.close();
                }catch(JMSException e){
                    e.printStackTrace();
                    System.exit(2);
                }
            }else{
                try{
                    ActiveMQPingPong ping_pong = (interval == 0?
                        new ActiveMQPingPong(address, topic1, topic2, m_count,
                                prior2, cpu2, filename2, topic_prior, interval, min_size, isFirst) :
                        new ActiveMQPingPong(address, topic1, topic2, m_count,
                                prior2, cpu2, filename2, topic_prior, interval, min_size, max_size, step, before_step, isFirst));
                    ping_pong.startTest();
                    ping_pong.close();
                }catch(JMSException e){
                    e.printStackTrace();
                    System.exit(2);
                }
            }
        }else{
            System.out.println("Wrong type");
            System.exit(3);
        }
        System.out.println("End main");
    }
}

