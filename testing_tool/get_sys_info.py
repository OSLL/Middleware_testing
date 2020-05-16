import platform
import cpuinfo
import json

class system:
    sys_platform = platform.platform()
    cpu = cpuinfo.get_cpu_info()

    def getInfo(self):
        res = {"System": self.sys_platform}
        res.update(self.cpu)
        return json.dumps(res)