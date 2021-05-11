package java_interface;

import org.json.simple.JSONObject;
import org.json.simple.JSONArray;
import java.io.FileWriter;
import java.io.IOException;
import java.util.concurrent.Semaphore;
import java.util.*;
import java.util.concurrent.Future;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Callable;


public abstract class PingPongInterface<T> extends TestMiddlewareInterface{

    protected long[] _recieve_timestamps;
    protected T[] _msgs;
    protected long[] _read_msg_time;
    protected long[] _write_msg_time;
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

    protected boolean _isNew;
    protected boolean _isTestStarted;
    protected boolean _const_queue;
    protected int _msgs_before_step;
    protected int _step;
    protected int _cur_size;
    protected int _msgSizeMax;
    protected int _msgSizeMin;

    protected long WATERMARK = 50;
    protected Semaphore sem = new Semaphore(1);
    protected int _last_rec_msg_id = -1;
    protected ExecutorService executor = Executors.newSingleThreadExecutor();

    public PingPongInterface(String topic1, String topic2, int msgCount, int prior, int cpu_index,
            String filename, int topic_priority, int msInterval, int msgSize, boolean isFirst,  Class<T>dataType){
        super(prior, cpu_index, false);
        this._filename = filename;
        if (isFirst) {
            this._topic_name1 = topic1;
            this._topic_name2 = topic2;
        }
        else {
            this._topic_name1 = topic2;
            this._topic_name2 = topic1;
        }
        this._msInterval = msInterval;
        this._cpu_index = cpu_index;
        this._msgCount = msgCount;
        this._msgSize = msgSize;
        this._cur_size = _msgSizeMin;
        this._isFirst = isFirst;
        this._topic_priority = topic_priority;
        this._recieve_timestamps = new long[msgCount];
        this._read_msg_time = new long[msgCount];
        this._write_msg_time = new long[msgCount];
        this._msgs = (T[]) java.lang.reflect.Array.newInstance(dataType, msgCount);

        this._isNew = false;
    }

    public PingPongInterface(String topic1, String topic2, int msgCount, int prior, int cpu_index,
            String filename, int topic_priority, int msInterval, int msgSizeMin, int msgSizeMax, int step,
            int before_step, boolean isFirst,  Class<T>dataType){
        super(prior, cpu_index, false);
        _topic_name1 = topic1;
        _topic_name2 = topic2;
        _filename = filename;
        this._recieve_timestamps = new long[msgCount];
        this._read_msg_time = new long[msgCount];
        this._write_msg_time = new long[msgCount];
        this._msgs = (T[]) java.lang.reflect.Array.newInstance(dataType, msgCount);
        _isFirst = isFirst;
        _isNew = true;
        _topic_priority = topic_priority;
        _msgCount = msgCount;
        _cpu_index = cpu_index;
        _msInterval = msInterval;
        _msgSizeMin = msgSizeMin;
        _msgSizeMax = msgSizeMax;
        _step = step;
        _msgs_before_step = before_step;
        _msgSize = msgSizeMin;
        _cur_size = msgSizeMin;

        if(_msgSize == 0){
            _msInterval = 0;
            _msgSize = _msgSizeMax;
            _msgSizeMin = _msgSize;
            _const_queue = true;
        }
    }

    public boolean wait_for_msg(){
        long start_timeout, end_timeout;
        end_timeout = System.currentTimeMillis() * TIME_SCALE + System.nanoTime();
        start_timeout = end_timeout;
        try{
            while(_isFirst && _isNew){
                sem.acquire();
                boolean is_started = _isTestStarted;
                sem.release();
                if(!is_started) Thread.currentThread().sleep(1);
                else break;
            }
            boolean is_not_received = true;
            while(is_not_received){
                if(_last_rec_msg_id - 1 == _msgCount && _isNew)
                    break;
                
                sem.acquire();
                if(receive()){
                    if(!_isNew || !_isFirst)
                        is_not_received = false;
                    start_timeout = System.currentTimeMillis() * TIME_SCALE + System.nanoTime();
                }else{
                    end_timeout = System.currentTimeMillis() * TIME_SCALE + System.nanoTime();
                    if(end_timeout - start_timeout > TIMEOUT){
                        sem.release();
                        return true;
                    }
                }
                sem.release();

                Thread.currentThread().sleep(1);
            }
        }catch(InterruptedException e){
            return true;
        }
        return false;
    }

    public int startNewTest(){
        try{
            Thread.currentThread().sleep(4*1000);
            long start_timeout, end_timeout;
            end_timeout = System.currentTimeMillis() * TIME_SCALE + System.nanoTime();
            start_timeout = end_timeout;
            if(_isFirst){
                Future<Boolean> future = executor.submit(new Callable<Boolean>() {
                    public Boolean call(){
                        return wait_for_msg();
                }});
                for(int i = 0; i < _msgCount; i++){
                    if(_const_queue){
                        sem.acquire();
                        if(i - _last_rec_msg_id > WATERMARK){
                            i--;
                            sem.release();
                            end_timeout = System.currentTimeMillis() * TIME_SCALE + System.nanoTime();
                            if(end_timeout - start_timeout > TIMEOUT)
                                break;
                            continue;
                        }
                        sem.release();
                    }
                    if(i == 0){
                        sem.acquire();
                        _isTestStarted = true;
                        sem.release();
                    }
                    start_timeout = System.currentTimeMillis() * TIME_SCALE + System.nanoTime();
                    if( i % (_msgs_before_step - 1) == 0 && _cur_size <= _msgSizeMax)
                        _cur_size += _step;

                    sem.acquire();
                    publish(i, _cur_size);
                    sem.release();

                    Thread.currentThread().sleep(_msInterval);
                }
                System.out.println("Waitung for second thread!");
                future.get();
                System.out.println("All threads done!");
            }else{
                while(!wait_for_msg());
                Thread.currentThread().sleep(4*1000);
            }
            System.out.println("Test ended");

            to_Json();
            return 0;
        }catch(Exception e){
            e.printStackTrace();
            return -1;
        }
    }

    public int startTest(){
        if(_isNew) return startNewTest();
        else return startTestOld();
    }

    public void write_received_msg(T msg){
        _last_rec_msg_id = get_id(msg);
        _msgs[_last_rec_msg_id] = msg;
        _recieve_timestamps[get_id(msg)] = System.currentTimeMillis() *
                TIME_SCALE + System.nanoTime();
        if(!_isFirst && _isNew){
            if( _last_rec_msg_id % (_msgs_before_step - 1) == 0 && _cur_size <= _msgSizeMax)
                _cur_size += _step;

            publish(_last_rec_msg_id, _cur_size);
        }
    }

    public int startTestOld() {
        try{
            boolean isTimeoutEx = false;
            Thread.currentThread().sleep(4 * 1000);

            for(int i=0; i<_msgCount; i++){
                if(_isFirst)
                    publish(i, _msgSize);

                isTimeoutEx = wait_for_msg();
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
            T msg = _msgs[i];
            Map map = new HashMap();
            map.put("id", get_id(msg));
            map.put("sent_time", get_timestamp(msg));
            map.put("recieve_timestamp", _recieve_timestamps[i]);
            map.put("delay", _recieve_timestamps[i] - get_timestamp(msg));
            map.put("read_proc_time", _read_msg_time[i]);
            map.put("proc_time", _write_msg_time[i]);
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
