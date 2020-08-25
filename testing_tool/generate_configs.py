from test0_config import test0_config
from test3_config import test3_config
from test5_config import test5_config
from test6_config import test6_config
from test7_config import test7_config


def generate_config():
    c = test0_config()
    c = test3_config()
    c = test5_config()
    c = test6_config()
    c = test7_config()


if __name__ == '__main__':
    generate_config()
