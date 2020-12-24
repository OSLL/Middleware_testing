load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "PubSub",
    srcs = ["pubsub.cc", "pub_interface.hpp", "sub_interface.hpp","ping_pong_interface.hpp", "test_errors.hpp"],
    deps = [
        "//cyber",
        "//testing/proto:data_cc_proto",
    ],
)

cpplint()
