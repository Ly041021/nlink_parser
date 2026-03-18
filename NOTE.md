# TOFSense-M 级联查询改动说明

## 1. 这次改了什么

本次在 `TOFSense-M` 中新增了和 `TOFSense` 同风格的 `inquire_mode` 级联查询逻辑，用于 6 个传感器轮询采集。

- 新增查询模式开关参数：`~inquire_mode`（默认 `false`）
- 查询模式开启后，按 ID 轮询 `0~5` 共 6 个节点
- 每轮查询频率固定为 `15Hz`
- 单轮内每个查询命令间隔：`0.006s`
- 新增聚合话题：`/nlink_tofsensem_cascade`
- 保留原有单帧话题：`/nlink_tofsensem_frame0`

关键实现文件：

- `src/tofsensem/init.cpp`
- `src/tofsensem/init.h`
- `src/tofsensem/main.cpp`
- `msg/TofsenseMCascade.msg`
- `launch/tofsensem.launch`

## 2. 查询逻辑（和 TOFSense 一致）

1. `timer_scan_` 每 `1/15s` 启动一次新一轮查询。
2. 轮询开始时清空缓存 `frame0_map_`，并把 `node_index_` 置 0。
3. `timer_read_` 每 `0.006s` 发送一次查询命令，依次查询 ID `0,1,2,3,4,5`。
4. 收到回包后按 `id` 存入 `frame0_map_`。
5. 当一轮查询结束后，把缓存中的多个 `TofsenseMFrame0` 打包成 `TofsenseMCascade` 发布。

查询命令格式（与 TOFSense inquire 逻辑一致）：

- 帧头：`0x57 0x10`
- 目标 ID：`id`
- 校验：`NLink_UpdateCheckSum(...)`

## 3. 话题与参数

运行参数（`launch/tofsensem.launch`）：

- `port_name`，默认 `/dev/ttyTHS0`
- `baud_rate`，默认 `115200`
- `inquire_mode`，默认 `false`

发布话题：

- 非查询模式：`/nlink_tofsensem_frame0`（`nlink_parser/TofsenseMFrame0`）
- 查询模式：`/nlink_tofsensem_cascade`（`nlink_parser/TofsenseMCascade`）

## 4. 完整编译与运行指令

下面这组命令是从零开始的一套完整流程（真机执行）。

```bash
# 1) 进入工作空间
cd ~/catkin_ws/src

# 2) 拉取仓库（如果已存在可跳过）
# git clone https://github.com/Ly041021/nlink_parser.git

# 3) 进入仓库并拉齐子模块
cd ~/catkin_ws/src/nlink_parser
git submodule sync --recursive
git submodule update --init --recursive --jobs 8

# 4) 回到工作空间编译
cd ~/catkin_ws
catkin_make

# 5) 加载环境（bash 用 setup.bash，zsh 用 setup.zsh）
source ~/catkin_ws/devel/setup.zsh
```

运行 `TOFSense-M` 查询模式（6路级联，15Hz）：

```bash
roslaunch nlink_parser tofsensem.launch inquire_mode:=true
```

查看级联数据：

```bash
rostopic echo /nlink_tofsensem_cascade
```

如果你想回到普通单设备输出模式：

```bash
roslaunch nlink_parser tofsensem.launch inquire_mode:=false
rostopic echo /nlink_tofsensem_frame0
```

