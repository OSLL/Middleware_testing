package java_interface;

import org.json.simple.JSONObject;
import org.json.simple.JSONArray;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;


public abstract class SubscriberInterface<T> extends TestMiddlewareInterface{

    protected String _topic_name;
    protected ArrayList<Long> _recieve_timestamps;
    protected ArrayList<Long> _read_msg_time;
    protected ArrayList<T> _msgs;
    protected int _msgCount;
    protected String _filename;
    protected int _topic_prior;
    protected int _cpu_index;
    protected long TIME_SCALE = 1_000_000L;
    protected long TIMEOUT = 20_000_000_000L;

    public SubscriberInterface(String topic, int msgCount, int prior, int cpu_index,
            String filename, int topic_prior){
        super(cpu_index, prior, true);
        this._topic_name = topic;
        this._msgCount = msgCount;
        this._cpu_index = cpu_index;
        this._filename = filename;
        this._topic_prior = topic_prior;
        this._recieve_timestamps = new ArrayList<Long>(msgCount);
        this._read_msg_time = new ArrayList<Long>(msgCount);
        this._msgs = new ArrayList<T>(msgCount);
    }

    public void write_received_msg(T msg, long proc_time){
        _msgs.add(get_id(msg), msg);
        _recieve_timestamps.add(get_id(msg), System.currentTimeMillis() * 
                TIME_SCALE + System.nanoTime());
        _read_msg_time.add(get_id(msg), proc_time);
    }

    public int startTest() {
        try{
            boolean isTimeoutEx = false;

            long start_timeout, end_timeout = System.currentTimeMillis() * 
                TIME_SCALE + System.nanoTime();
            start_timeout = end_timeout;

            while(true){
                if(receive()) {
                    start_timeout = System.currentTimeMillis() * TIME_SCALE + System.nanoTime();
                }else{
                    end_timeout = System.currentTimeMillis() * TIME_SCALE + System.nanoTime();
                    if(end_timeout - start_timeout > TIMEOUT){
                        isTimeoutEx = true;
                        break;
                    }
                }

                Thread.currentThread().sleep(1);
            }

            to_Json();

            if(isTimeoutEx)
                return 7; //TIMEOUT_ERROR
            return 0;
        } catch (Exception e){
            e.printStackTrace();
            return -1;
        }
    }           

    public void to_Json(){
        JSONArray json = new JSONArray();
        for(int i=0; i < _msgs.size(); i++){
            JSONObject obj = new JSONObject();
            T msg = _msgs.get(i);
            Map map = new HashMap();
            map.put("id", get_id(msg));
            map.put("sent_time", get_timestamp(msg));
            map.put("recieve_timestamp", _recieve_timestamps.get(i));
            map.put("delay", _recieve_timestamps.get(i) - get_timestamp(msg));
            map.put("read_proc_time", _read_msg_time.get(i));
            obj.put("msg", map);
            json.add(obj);
        }
        try (FileWriter file = new FileWriter(_filename)){
            file.write(json.toJSONString());
            file.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public abstract int get_id(T msg);

    public abstract long get_timestamp(T msg);

    public abstract boolean receive();
}
