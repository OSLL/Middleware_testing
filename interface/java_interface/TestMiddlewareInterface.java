package java_interface;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

enum Operation{
    ADD_CPUS,
    SET_EXCLUSIVE,
    SET_MEM,
    SET_PID
}

public abstract class TestMiddlewareInterface {
    TestMiddlewareInterface(int cpuIndex, int priority, boolean isSub) {
        this._priority = priority;
        if(cpuIndex >= 0) {
            try {
                this.cpusetAdd(cpuIndex, isSub);
            } catch (IOException e) {
                e.printStackTrace();
                System.exit(-1);
            }
        }
        // TODO: Setting process priority if possible
        /* if(priority >= 0){
            ProcessBuilder pb = new ProcessBuilder("nice", "-n", "-19", "java main");
        }*/
    }
    abstract int startTest();

    private void cpusetAdd(int cpuIndex, boolean isSub) throws IOException {
        Path cpusetPath;
        if (isSub) {
            cpusetPath = Paths.get("/sys/fs/cgroup/cpuset/sub_cpuset");
        }
        else{
            cpusetPath = Paths.get("/sys/fs/cgroup/cpuset/pub_cpuset");
        }
        if (!Files.exists(cpusetPath)) {
            Files.createDirectories(cpusetPath);
        }
        this.setParam(Operation.ADD_CPUS, cpusetPath, cpuIndex);
        this.setParam(Operation.SET_EXCLUSIVE, cpusetPath, cpuIndex);
        this.setParam(Operation.SET_MEM, cpusetPath, cpuIndex);
        this.setParam(Operation.SET_PID, cpusetPath, cpuIndex);

    }

    private void setParam(Operation op, Path cpusetPath, int cpuIndex) throws IOException{
        Path path;
        int param;
        switch (op){
            case ADD_CPUS:
                path = Paths.get(cpusetPath.toString() + "/cpuset.cpus");
                param = cpuIndex;
                break;
            case SET_EXCLUSIVE:
                path = Paths.get(cpusetPath.toString() + "/cpuset.cpu_exclusive");
                param = 1;
                break;
            case SET_MEM:
                path = Paths.get(cpusetPath.toString() + "/cpuset.mems");
                param = 0;
                break;
            case SET_PID:
                path = Paths.get(cpusetPath.toString() + "/tasks");
                long pid = ProcessHandle.current().pid();
                System.out.println(pid);
                Files.write(path, Long.toString(pid).getBytes());
                return;
            default:
                throw new IOException("Wrong operation!");
        }

        Files.write(path, Integer.toString(param).getBytes());
    }

    private int _priority;
}
