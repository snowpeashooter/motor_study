# AGENTS.md

## 回答格式

请默认使用 OneNote 友好格式回答用户：

1. 普通解释使用简洁中文，分段清楚。
2. 所有终端命令、代码、路径、文件名、配置内容，都单独放进 Markdown 代码块。
3. 不要把命令、路径、文件名、配置项混在普通句子中。
4. 每个代码块前用一句话说明用途。
5. 如果有多条命令，按执行顺序分开列出。
6. 重要提醒用“注意：”开头。

## 大型项目执行效率

本项目较大，请优先采用“先索引、后定位、再精读”的方式工作。

不要一开始全量阅读整个仓库。先快速判断项目结构，再只打开与当前任务相关的文件。

## 搜索原则

优先使用快速搜索工具列出文件。

```bash
rg --files
```

搜索具体符号、函数、配置项时，使用精确关键词。

```bash
rg -n "keyword" path
```

默认避免在以下目录中做无差别搜索，除非任务明确需要。

```text
build
install
dist
out
target
node_modules
vendor
thirdparty
__pycache__
.cache
.git
log
logs
```

## 项目索引

如果项目较大或文件较多，请优先创建或维护轻量索引文件。

```text
PROJECT_INDEX.md
```

索引文件应包含：

```text
1. 顶层目录说明
2. 主要入口文件
3. 常用运行命令
4. 配置文件位置
5. 测试/构建方式
6. 常见任务对应文件
7. 应跳过的大目录
```

之后处理任务时，应先阅读索引文件。

```text
PROJECT_INDEX.md
```

再阅读相关源码或配置。

## 阅读顺序

遇到新任务时，按以下顺序获取上下文：

```text
1. PROJECT_INDEX.md
2. README / README_CN / docs 中相关章节
3. 当前任务直接相关的脚本或配置
4. 被调用的核心源码
5. 测试文件
```

只有当前任务需要时，才继续深入第三方库、生成文件或编译产物。

## 文件分类

首次进入大型项目时，先将文件粗分为：

```text
入口脚本
配置文件
核心源码
测试文件
文档说明
构建脚本
生成产物
第三方依赖
日志缓存
```

优先查看：

```text
入口脚本
配置文件
核心源码
文档说明
```

默认跳过：

```text
生成产物
第三方依赖
日志缓存
```

## 命令执行

多个互不依赖的只读操作可以并行执行，例如：

```text
列文件
搜索关键词
读取不同说明文件
查看配置
```

会修改文件、安装依赖、删除文件、启动服务的命令不要并行执行。

## 修改原则

修改前先确认目标文件和影响范围。

不要做无关重构。

不要修改生成目录、第三方目录或编译产物，除非任务明确要求。

优先遵循项目已有风格和工具链。

## 验证原则

改完后优先运行最小验证，例如：

```text
单元测试
构建目标模块
运行目标脚本
静态检查
格式检查
```

大型项目中不要默认运行耗时很长的全量测试，除非当前修改影响面很大，或用户明确要求。

## 输出原则

回答时优先给出结论、改了什么、怎么验证。

不要输出大量无关搜索过程。

如果遇到阻塞，说明：

```text
阻塞原因
已检查内容
下一步最小可行操作
```

## 长期维护

当发现新的关键入口、常见报错或配置文件时，及时更新索引文件。

```text
PROJECT_INDEX.md
```

保持索引短小、准确、可扫描。

## 当前项目重点

本仓库当前包含两个主要子工程：

```text
roboparty_deploy
roboparty_deploy12DOF
```

普通版工程的关键入口：

```text
roboparty_deploy/scripts/motors_py_example.py
roboparty_deploy/scripts/set_zero.py
roboparty_deploy/scripts/motion_player.py
roboparty_deploy/tools/start_robot.sh
roboparty_deploy/scripts/config/set_zero.yaml
roboparty_deploy/scripts/config/motion_player.yaml
roboparty_deploy/assets/99-auto-up-devs-sunrise.rules
```

默认优先跳过普通版工程中的这些目录：

```text
roboparty_deploy/build
roboparty_deploy/install
roboparty_deploy/log
roboparty_deploy/src/inference/thirdparty
```

12DOF 工程中也默认优先跳过这些目录：

```text
roboparty_deploy12DOF/root/roboparty_deploy12DOF/build
roboparty_deploy12DOF/root/roboparty_deploy12DOF/install
roboparty_deploy12DOF/root/roboparty_deploy12DOF/log
roboparty_deploy12DOF/root/roboparty_deploy12DOF/src/inference/thirdparty
```

核心原则：

```text
先建地图，后走路；先搜入口，后读实现；先最小验证，后全量检查。
```
