# Printf 浮点数打印问题修复

## 问题现象

使用 `printf` 或 `vsnprintf` 打印浮点数时，输出为空：

```
zmor:,,
test 141: ,,
```

- 整数 `%d` 正常打印：`test 141`
- 浮点数 `%.2f` 输出为空：`,,`

## 根本原因

**ARM GCC 工具链在使用 `newlib-nano` 时，默认不包含浮点数格式化支持**，以节省 Flash 空间。

配置中启用了：
```yaml
use-newlib-nano: true
```

这会使用精简版 C 标准库，默认移除了 `printf` 的浮点数格式化功能。

## 解决方案

在链接器选项中添加 `-u _printf_float` 标志，强制链接浮点数格式化代码。

### 修改文件：`.eide/eide.yml`

在 `targets.Debug.toolchainConfigMap.GCC.options.linker` 部分修改：

```yaml
linker:
  $outputTaskExcludes:
    - .bin
  $toolName: auto
  LIB_FLAGS: -lm -u _printf_float  # 添加 -u _printf_float
  misc-controls: ""
  output-format: elf
  remove-unused-input-sections: true
```

**注意：** 必须添加到 `LIB_FLAGS` 而不是 `misc-controls`，EIDE 才会正确传递给链接器。

## 验证步骤

1. 修改 `.eide/eide.yml` 配置文件
2. **执行 rebuild（重新构建）** - 必须 rebuild，build 不会重新链接
3. 烧录程序
4. 检查串口输出

成功后输出示例：
```
zmor:0.10,1.00,0.10
test 1: 0.10,1.00,0.10
```

## 代码空间影响

添加浮点数格式化支持会增加约 **9-10KB** Flash 使用量：

| 配置 | ROM 使用 |
|------|----------|
| 修复前 | 17888 B (13.65%) |
| 修复后 | 27456 B (20.95%) |
| 增加 | ~9.5 KB |

## 相关代码

- `User/utils/vofa.c` - 使用 `vsnprintf` 格式化浮点数
- `User/bsp/usart1.c` - `printf` 重定向实现
- `User/app/main.c` - 测试代码

## 技术细节

`-u _printf_float` 的作用：
- `-u symbol`: 强制链接器包含指定符号，即使它看起来未被引用
- `_printf_float`: newlib-nano 中浮点数格式化的入口符号
- 该符号会引入完整的浮点数格式化代码到最终固件中

## 适用场景

此问题适用于所有使用以下配置的 ARM 嵌入式项目：
- 使用 ARM GCC 工具链
- 启用 `--specs=nano.specs` 或 `use-newlib-nano`
- 需要使用 `printf`/`sprintf`/`snprintf`/`vsnprintf` 打印浮点数

---
**修复日期：** 2025-12-15  
**相关链接：** [Newlib documentation](https://sourceware.org/newlib/)
