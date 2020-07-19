package java_interface;

import org.json.simple.JSONObject;
import org.json.simple.JSONArray;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;


public abstract class PingPongInterface<T> extends TestMiddlewareInterface{

    protected ArrayList<Long> _recieve_timestamps;
    protected ArrayList<T> _msgs;
    protected int _msgSize;
    protected String _filename;
    protected String _topic_name1;
    protected String _topic_name2;
    protected int _msInterval;
    protected int _cpu_index;
    protected int _msgCount;
    protected int _topic_priority;
    protected boolean _isFirst;
    protected long TIME_SCALE = 1_000_000L;
    protected long TIMEOUT = 20_000_000_000L;

    public PingPongInterface(String topic1, String topic2, int msgCount, int prior, int cpu_index,
            String filename, int topic_priority, int msInterval, int msgSize, boolean isFirst){
        super(prior, cpu_index, false);
        this._filename = filename;
        this._topic_name1 = topic1;
        this._topic_name2 = topic2;
        this._msInterval = msInterval;
        this._cpu_index = cpu_index;
        this._msgCount = msgCount;
        this._msgSize = msgSize;
        this._isFirst = isFirst;
        this._topic_priority = topic_priority;
        this._recieve_timestamps = new ArrayList<Long>(msgCount);
        this._msgs = new ArrayList<T>(msgCount);
    }
    public void write_received_msg(T msg){
        _msgs.add(get_id(msg), msg);
        _recieve_timestamps.add(get_id(msg), System.currentTimeMillis() * 
                TIME_SCALE + System.nanoTime());
    }

    public int startTest() {
        try{
            boolean isTimeoutEx = false;
            long start_timeout, end_timeout;
            Thread.currentThread().sleep(4 * 1000);

            for(int i=0; i<_msgCount; i++){
                if(_isFirst)
                    publish(i, _msgSize);

                end_timeout = System.currentTimeMillis() * TIME_SCALE + System.nanoTime();
                start_timeout = end_timeout;

                boolean notReceived = true;
                while(notReceived){
                    if(receive()) {
                        notReceived = false;
                    }else{
                        end_timeout = System.currentTimeMillis() * TIME_SCALE + System.nanoTime();
                        if(end_timeout - start_timeout > TIMEOUT){
                            isTimeoutEx = true;
                            break;
                        }
                    }

                    Thread.currentThread().sleep(1);
                }
                if(isTimeoutEx)
                    break;
                if(!_isFirst)
                    publish(i, _msgSize);
            }

            to_Json();

            if(isTimeoutEx)
                return 7;
            return 0;
        } catch (Exception e){
            e.printStackTrace();
            return -1;
        }
    }           

    public void to_Json(){
        JSONArray json = new JSONArray();

        for(int i=0; i<_msgCount; i++){
            JSONObject obj = new JSONObject();
            T msg = _msgs.get(i);
            Map map = new HashMap();
            map.put("id", get_id(msg));
            map.put("sent_time", get_timestamp(msg));
            map.put("recieve_timestamp", _recieve_timestamps.get(i));
            map.put("delay", _recieve_timestamps.get(i) - get_timestamp(msg));
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

    public abstract void publish(int id, int size);
}
