from general_funcs import get_configs
from test0_config import test0_config
from test3_config import test3_config
from test5_config import test5_config
from test6_config import test6_config
from test7_config import test7_config

def generate_config():
    c = test0_config()
    print(c)
    print(get_configs(0))
    c = test3_config()
    print(c)
    print(get_configs(3))
    c = test5_config()
    print(c)
    print(get_configs(5))
    c = test6_config()
    print(c)
    print(get_configs(6))
    c = test7_config()
    print(c)
    print(get_configs(7))

if __name__ == '__main__':
    generate_config()
