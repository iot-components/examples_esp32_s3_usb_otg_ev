## 编译示例代码

1. 确认 ESP-IDF 环境成功搭建（Demo 测试推荐使用 master 分支）
2. 安装 `ESP-IDF Component Manager`
   1. `. ./export.sh` to add ESP-IDF environment values
   2. `pip install idf-component-manager --upgrade`
3. 设置编译目标为 `esp32-s2`, `idf.py set-target esp32s2`
4. 编译、下载、并查看输出, `idf.py build flash monitor`
5. 如果 `ESP-IDF Component Manager` 安装成功，编译过程中项目所需的组件将自动下载
