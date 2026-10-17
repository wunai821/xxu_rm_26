# EGA 车模显示网格转换

仅提取保存的显示网格、装配位姿和名称，再作展示上色；不求解 SolidWorks 配合，
不生成质量/惯量或驱动关节，也不求解折叠角度。当前缓存网格不能替代折叠后的动力学模型。
需在 SolidWorks 2025 中切换至折叠过洞配合、重建并保存，再导出保留零件层级的整车 STEP AP214/AP242；仅导出可见实体，排除参考球和平面。
原始 CAD 输入保留在本地 `local_assets/cad_source/`，不纳入 Git；转换输出应写入新的本地目录。

读取器来自 [XRTC5/sldprt-export](https://github.com/XRTC5/sldprt-export)，MIT 许可，
固定提交 `ebde7524b3876c5d78c7e2248f2eba9b5725a6c0`。
本目录的补丁针对这批 CAD 的目录跳号、缓存选择、覆盖计数和父隐藏状态作适配。
只在独立读取器 checkout 应用，不修改用户 CAD。

准备独立读取器和 Python 环境后：

```bash
git -C <reader_checkout> checkout ebde7524b3876c5d78c7e2248f2eba9b5725a6c0
git -C <reader_checkout> apply <project>/xxu_nav_2026/tools/navigation/cad/sldprt-export-ega.patch
<python_env>/bin/python -m pip install -r requirements.txt
<python_env>/bin/python convert_ega_sentry.py <extracted_cad> <reader_checkout> <output>
```

`extracted_cad` 应包含哨兵主装配及配套 SLDPRT/SLDASM，保持原文件名。
输出 GLB 明确标为 draft；JSON 给出未解析实例、显示块 CRC、原始与可视边界、
零件数量和来源散列。NPZ 保留原始提取网格。文件读取和转换在本机完成。

取消了破坏开槽薄板的减面；UW01 的大球/平面参考体按其随文件预览排除，
不使用这些参考体作车体碰撞或雷达遮挡。黑/灰/浅色是展示风格，不是材料密度判断。
