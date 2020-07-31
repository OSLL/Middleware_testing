package java_interface;

import org.json.simple.JSONObject;
import org.json.simple.JSONArray;
import java.io.*;
import java.util.*;

public abstract class PublisherInterface extends TestMiddlewareInterface{

    protected String _filename;
    protected String _topic_name;
    protected int _msInterval;
    protected int _cpu_index;
    protected int _msgCount;
    protected int _byteSizeMin;
    protected int _byteSizeMax;
    protected int _step;
    protected int _msg_count_befor_step;
    protected int _topic_priority;
    ArrayList< Long > _write_msg_time = new ArrayList<Long>();

    public PublisherInterface(String topic, int msgCount, int prior, int cpu_index,
            int min_msg_size, int max_msg_size, int step, int interval, int msgs_before_step,
            String filename, int topic_priority){
        super(prior, cpu_index, false);
        this._filename = filename;
        this._topic_name = topic;
        this._msInterval = interval;
        this._cpu_index = cpu_index;
        this._msgCount = msgCount;
        this._byteSizeMin = min_msg_size;
        this._byteSizeMax = max_msg_size;
        this._step = step;
        this._msg_count_befor_step = msgs_before_step;
        this._topic_priority = topic_priority;
        this._write_msg_time = new ArrayList<Long>(msgCount);
    }

    public int startTest() {
        try{
            long proc_time = 0;
            Thread.currentThread().sleep(4*1000);

            int cur_size = _byteSizeMin;
            for(int i=0; i<_write_msg_time.size(); i++){
                if(i % (_msg_count_befor_step - 1) == 0 && cur_size<=_byteSizeMax)
                    cur_size += _step;

                proc_time = publish(i, cur_size);

                if(proc_time == 0){
                    return 10;
                }
                _write_msg_time.add(i, proc_time);

                Thread.currentThread().sleep(_msInterval);
            }

            Scanner sc = new Scanner(System.in);
            if(sc.hasNextLine()){
                if(sc.nextLine().equals("end")){
                    to_Json();
                    return 0;
                }
            }

            Thread.currentThread().sleep(20*1000);

        } catch (Exception e){
            e.printStackTrace();
            System.exit(-1);
        }
        return -1;
    }

    public void to_Json(){
        JSONArray json = new JSONArray();

        for(int i=0; i<_msgCount; i++){
            JSONObject obj = new JSONObject();
            Map map = new HashMap();
            map.put("id", i);
            map.put("proc_time", _write_msg_time.get(i));
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

    public abstract long publish(int id, int size);

}
