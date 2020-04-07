#pragma once
#include <exception>
#include <cstdlib>
#define MIDDLEWARE_ERROR 1
#define TIMEOUT_ERROR 2
#define CPUSET_ERROR 3
#define THREAD_PRIOR_ERROR 4


class test_exception: public std::exception{
    int _ret_code;
    std::string _err_info;
public:
    test_exception(){}

    test_exception(std::exception& other):
    std::exception(other),
    _ret_code(MIDDLEWARE_ERROR),
    _err_info(other.what()){}

    test_exception(std:: string err_info, int ret_code):
    _ret_code(ret_code),
    _err_info(err_info){}

    virtual char const *what() const noexcept { return _err_info.c_str(); }

    int get_ret_code() const {
        return _ret_code;
    }
};
