# 私密仓库与文件范围

个人维护仓库：`https://github.com/zzyhp1-code/sp-vision-autoaim`，可见性为私密。
上游来源：`https://github.com/kukuaxing/sp-vision-2026-integration-archive`。

本次上传包含本机现有自瞄源码及修改、配置、标定与调试脚本、文档和源码子模块引用。保留原有许可证。现有源码目录和调用路径保持一致，分类入口见根目录 README。

以下内容留在本地：运行日志、采集图片、录像、模型权重、相机 SDK 动态库、编译输出、Python 缓存、SSH 密钥和环境凭据。`simulation/RESULTS.csv` 与 `simulation/SUMMARY.csv` 是原项目已有仿真报告数据，不是本地运行日志。

本地 `origin` 保留原上游；个人仓库使用独立远程名 `private`。更新个人仓库时使用：

```bash
git push private HEAD:main
```

新克隆的仓库默认将个人仓库命名为 `origin`，此时使用 `git push origin main`。不要混淆两种工作区。

本次发布检查覆盖文件清单、忽略规则、脚本语法与提交内容，不执行相机、云台或发射测试，也不将未进行的 Linux 硬件验证描述为通过。
