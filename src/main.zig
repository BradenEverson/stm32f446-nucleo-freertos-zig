//! Zig Entrypoint

const std = @import("std");
const c = @cImport({
    @cDefine("USE_HAL_DRIVER", {});
    @cDefine("STM32F446xx", {});
    @cInclude("main.h");
});

const os = @cImport({
    @cInclude("FreeRTOS.h");
    @cInclude("task.h");
});

const pvParameters: ?*anyopaque = null;
const pxCreatedTask: ?*os.TaskHandle_t = null;

export fn entry() callconv(.c) void {
    os.vTaskStartScheduler();
    unreachable;
}
