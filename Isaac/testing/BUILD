load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_binary")
load("@com_nvidia_isaac_sdk//bzl:module.bzl", "isaac_app", "isaac_cc_module")
load("messages.bzl", "create_message_proto_libraries", "message_proto_library_names")

filegroup(
    name = "capnp",
    srcs = glob(["*.capnp"]),
    visibility = ["//visibility:public"],
)

create_message_proto_libraries()

isaac_cc_binary(
     name = "main",
     srcs = ["main.cpp"],
     deps = [
         "@com_nvidia_isaac_engine//engine/alice/tools:parse_command_line"
     ],
)

isaac_app(
    name = "testing",
    modules = [
        "//packages/testing:nodes",
    ],
)

isaac_cc_module(
    name = "nodes",
    srcs = ["subscriber.cpp", "publisher.cpp", "ping_pong.cpp", "control.cpp"],
    hdrs = ["subscriber.hpp", "publisher.hpp", "ping_pong.hpp", "control.hpp"],
    deps = ["interfaces", "message_proto"]
)

cc_library(
    name = "interfaces",
    hdrs = ["msg.h", "pub_comp.hpp", "sub_comp.hpp", "ping_pong_comp.hpp"],
    deps = ["@test_interface//:interface"]
)

